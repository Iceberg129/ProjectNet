import asyncio, struct, serial, serial.tools.list_ports, threading, json, time, os
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from collections import defaultdict, deque

# ════════════════════ 帧格式常量 ════════════════════
FRAME_FORMAT = '<2sBBBBB8sB'
FRAME_SIZE   = struct.calcsize(FRAME_FORMAT)  # 16

MSG_RREQ = 0x10; MSG_RREP = 0x11; MSG_DATA = 0x01; MSG_ACK = 0x02
MSG_HEARTBEAT = 0x03; MSG_CMD = 0x04; MSG_CMD_ACK = 0x05
MSG_ACK_CONFIRM = 0x06
MSG_KICK = 0x08; MSG_UNKICK = 0x09
MSG_JOIN_REQ = 0x20; MSG_JOIN_ACK = 0x21; MSG_JOIN_REJ = 0x22
MSG_CMD_PREPARE = 0x0A; MSG_CMD_READY = 0x0B; MSG_CMD_COMMIT = 0x0C
MSG_NODE_STATUS = 0x07  # 节点状态上报
BROADCAST_ID    = 0xFF  # 全网广播地址，不应出现在拓扑中
MSG_NAMES = {MSG_RREQ:"RREQ",MSG_RREP:"RREP",MSG_DATA:"DATA",MSG_ACK:"ACK",
             MSG_HEARTBEAT:"HB",MSG_CMD:"CMD",MSG_CMD_ACK:"CMD_ACK",
             MSG_KICK:"KICK",MSG_UNKICK:"UNKICK",
             MSG_JOIN_REQ:"JOIN",MSG_JOIN_ACK:"JOIN_OK",MSG_JOIN_REJ:"JOIN_NO",
             MSG_ACK_CONFIRM:"ACK_CFM",
             MSG_CMD_PREPARE:"PREPARE",MSG_CMD_READY:"READY",MSG_CMD_COMMIT:"COMMIT",
             MSG_NODE_STATUS:"STATUS"}
MSG_ZH    = {MSG_RREQ:"路由发现",MSG_RREP:"路由应答",MSG_DATA:"数据传输",MSG_ACK:"确认应答",
             MSG_HEARTBEAT:"心跳",MSG_CMD:"下行命令",MSG_CMD_ACK:"命令应答",
             MSG_KICK:"踢出命令",MSG_UNKICK:"恢复命令",
             MSG_JOIN_REQ:"入网请求",MSG_JOIN_ACK:"入网允许",MSG_JOIN_REJ:"入网拒绝",
             MSG_ACK_CONFIRM:"ACK确认回执",
             MSG_CMD_PREPARE:"信道准备",MSG_CMD_READY:"信道就绪",MSG_CMD_COMMIT:"信道切换",
             MSG_NODE_STATUS:"节点状态"}

NODE_ROLES = {
    0x10: {"role":"主节点","en":"mainTerm","color":"#f97316","size":60},
    0x21: {"role":"中继节点","en":"relay","color":"#22d3ee","size":50},
    0x22: {"role":"中继节点","en":"relay","color":"#22d3ee","size":50},
    0x30: {"role":"终端节点","en":"sender","color":"#a78bfa","size":50},
}

def to_int8(b): return b - 256 if b >= 128 else b
def u8h(v):    return f"{v & 0xFF:02X}"

# ════════════════════ 统计追踪器 ════════════════════
class StatsTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.total_frames = 0
        self.rreq = self.rrep = self.data = self.ack = self.bad = 0
        self.nodes_seen = set()
        # 每节点统计
        self.node_stats = defaultdict(lambda: {"sent":0,"received":0,"firstSeen":0,"lastSeen":0})
        # 链路 RSSI 历史: key → deque of (timestamp, rssi)
        self.rssi_history = defaultdict(lambda: deque(maxlen=120))
        # 链路最新 RSSI
        self.link_rssi = {}
        # 当前路由
        self.current_route = None
        # 当前通信阶段
        self.current_phase = "idle"  # idle | rreq | rrep | data | ack
        self._last_offline = None
        self._last_online = None
        self.phase_updated = time.time()
        # ★ 通信成功率统计
        self.cycle_attempts = 0     # 尝试建立通信的总次数（周期结束时计入）
        self.cycle_successes = 0    # 成功完成 ACK_CONFIRM 的次数
        self._cycle_pending = False # 当前是否有未决的通信周期
        # ★ RREQ 周期节点追踪（用于快速离线检测）
        self._current_rreq_nodes = set()    # 本轮 RREQ 窗口中发现的节点
        # ★ 逐链路 RSSI 失败追踪（容忍偶发丢失，连续多次失败才断开）
        self._link_fail_count = {}          # link_key -> 连续未刷新次数
        self._LINK_FAIL_THRESHOLD = 1       # 一个周期内 sender 已重试 3 次，周期末即可判定
        self._current_rreq_links = set()    # 本轮 RREQ 中刷新了 RSSI 的链路
        self._known_links = set()           # 历史上所有出现过 RSSI 的链路（作为比较基准）
        self._pending_events = []           # 待广播的事件列表

    def _link_key(self, a: int, b: int) -> str:
        lo, hi = (a, b) if a < b else (b, a)
        return f"0x{lo:02X}-0x{hi:02X}"

    def record_frame(self, src: int, dest: int, msg_type: int,
                     path_id: int, count: int, stamps: list, relays: list):
        self.total_frames += 1
        self.nodes_seen.update([src, dest])
        now = time.time()

        # 每节点计数
        self.node_stats[src]["sent"] += 1
        self.node_stats[src]["lastSeen"] = now
        if self.node_stats[src]["firstSeen"] == 0:
            self.node_stats[src]["firstSeen"] = now
        self.node_stats[dest]["received"] += 1
        self.node_stats[dest]["lastSeen"] = now
        if self.node_stats[dest]["firstSeen"] == 0:
            self.node_stats[dest]["firstSeen"] = now

        # 消息类型计数
        if   msg_type == MSG_RREQ:      self.rreq += 1
        elif msg_type == MSG_RREP:      self.rrep += 1
        elif msg_type == MSG_DATA:      self.data += 1
        elif msg_type == MSG_ACK:       self.ack  += 1
        elif msg_type == MSG_ACK_CONFIRM: pass  # 不计入 ack 计数
        elif msg_type == MSG_CMD:      pass  # 下行命令不计数
        elif msg_type == MSG_CMD_ACK:  pass  # 命令应答不计数
        elif msg_type == MSG_JOIN_REQ:  pass  # 入网请求不计数主类型
        elif msg_type == MSG_JOIN_ACK:  pass  # 入网允许不计数
        elif msg_type == MSG_JOIN_REJ:  pass  # 入网拒绝不计数
        elif msg_type == MSG_HEARTBEAT:  self._handle_heartbeat(src, dest, count); pass
        elif msg_type in (MSG_CMD_PREPARE, MSG_CMD_READY, MSG_CMD_COMMIT): pass
        elif msg_type == MSG_NODE_STATUS:  pass  # 节点状态上报，单独处理
        else:                           self.bad  += 1

        # 链路 RSSI（从印章逐跳解析）
        if stamps:
            prev = src
            for s in stamps:
                rid = int(s["id"], 16)
                key = self._link_key(prev, rid)
                # ★ 链路 RSSI 刷新 → 重置失败计数 + 加入已知链路
                was_broken = self._link_fail_count.get(key, 0) >= self._LINK_FAIL_THRESHOLD
                self.link_rssi[key] = s["rssi"]
                self.rssi_history[key].append((now, s["rssi"]))
                self._link_fail_count[key] = 0
                self._known_links.add(key)
                if was_broken:
                    self._pending_events.append({
                        "type": "link_restored",
                        "linkKey": key,
                        "msg": f"链路 {key} RSSI 已恢复"
                    })
                # 更新中继节点统计
                self.nodes_seen.add(rid)
                ns = self.node_stats[rid]
                ns["lastSeen"] = now
                if ns["firstSeen"] == 0:
                    ns["firstSeen"] = now
                # ★ 记录本轮 RREQ 发现的中继（用于离线检测）
                if msg_type == MSG_RREQ:
                    self._current_rreq_nodes.add(rid)
                    self._current_rreq_links.add(key)
                prev = rid
            # 最终跳 → dest（也追踪链路，写入 _known_links 作为全量基准）
            if prev != dest:
                key = self._link_key(prev, dest)
                self.rssi_history[key].append((now, None))
                self._known_links.add(key)
                self._link_fail_count[key] = 0
                if msg_type == MSG_RREQ:
                    self._current_rreq_links.add(key)
        # ★ 记录 RREQ 的源节点（sender）进入本轮集合
        if msg_type == MSG_RREQ:
            self._current_rreq_nodes.add(src)

        # 更新路由信息 + 从 relay 路径推导活跃链路
        if msg_type == MSG_RREP:
            self.current_route = {"pathId": path_id, "relays": relays, "updated": now}
            # ★ 将 RREP 携带的 relay 路径链路也写入 _current_rreq_links
            #   （防止主节点只转发直达 RREQ 导致 _current_rreq_links 为空）
            if relays:
                prev = src
                for r in relays:
                    key = self._link_key(prev, r)
                    self._known_links.add(key)
                    self._link_fail_count[key] = 0
                    if self.current_phase == "rreq":
                        self._current_rreq_links.add(key)
                    prev = r
                key = self._link_key(prev, dest)
                self._known_links.add(key)
                self._link_fail_count[key] = 0
                if self.current_phase == "rreq":
                    self._current_rreq_links.add(key)

        # 更新通信阶段
        if   msg_type == MSG_RREQ: self._set_phase("rreq", now)
        elif msg_type == MSG_RREP: self._set_phase("rrep", now)
        elif msg_type == MSG_DATA: self._set_phase("data", now)
        elif msg_type == MSG_ACK:  self._set_phase("ack", now)
        elif msg_type == MSG_ACK_CONFIRM: self._set_phase("ack_done", now)

    def _set_phase(self, phase, now):
        old_phase = self.current_phase
        self.current_phase = phase
        self.phase_updated = now

        # ★ 新周期 RREQ 开始（非重试）→ 重置收集器
        #    sender 重试时 old_phase=="rrep"，此时不重置，累积所有重试的 RREQ 数据
        if phase == "rreq" and old_phase not in ("rreq", "rrep"):
            self._cycle_pending = True
            self._current_rreq_nodes = set()
            self._current_rreq_links = set()

        # ★ RREP 到达 → 早期预警（不递增 fail_count，最终判定在 ACK_CONFIRM）
        #    sender 可能还会重试，_current_rreq_links 在本周期内持续累积
        if phase == "rrep" and old_phase == "rreq":
            if self._known_links:
                missing = self._known_links - self._current_rreq_links
                for lk in missing:
                    if self._link_fail_count.get(lk, 0) == 0:
                        self._pending_events.append({
                            "type": "link_retry",
                            "linkKey": lk,
                            "msg": f"链路 {lk} 本轮暂未发现，sender 正在重试"
                        })

        # ★ ACK_CONFIRM → 周期结束，最终判定（sender 已耗尽重试，数据收集完整）
        if phase == "ack_done":
            self._finalize_cycle(success=True)
            if self._known_links:
                missing = self._known_links - self._current_rreq_links
                for lk in missing:
                    cnt = self._link_fail_count.get(lk, 0) + 1
                    self._link_fail_count[lk] = cnt
                    if cnt >= self._LINK_FAIL_THRESHOLD:
                        self._pending_events.append({
                            "type": "link_broken",
                            "linkKey": lk,
                            "missCount": cnt,
                            "msg": f"链路 {lk} sender 多次重试仍未发现，判定不可达"
                        })
                    if cnt >= 5:
                        self._known_links.discard(lk)

                # ★ 节点离线判定
                for nid in (0x21, 0x22, 0x30):
                    node_hex = f"0x{nid:02X}"
                    node_links = [l for l in self._known_links if node_hex in l]
                    if not node_links:
                        continue
                    all_broken = all(
                        self._link_fail_count.get(l, 0) >= self._LINK_FAIL_THRESHOLD
                        for l in node_links
                    )
                    if all_broken:
                        role = NODE_ROLES.get(nid, {}).get("role", "未知")
                        self._pending_events.append({
                            "type": "node_offline",
                            "nodeId": nid,
                            "nodeHex": node_hex,
                            "role": role,
                            "msg": f"所有链路({len(node_links)}条)均不可达，判定离线"
                        })

    def _finalize_cycle(self, success: bool):
        """周期结束时调用：统一计入 attempts（和 successes）"""
        if self._cycle_pending:
            self.cycle_attempts += 1
            if success:
                self.cycle_successes += 1
            self._cycle_pending = False

    def mark_cycle_failed(self, fail_phase: str):
        """外部（periodic_stats / simulate）调用的失败标记"""
        self._finalize_cycle(success=False)
        self.current_phase = f"{fail_phase}_fail"
        self.phase_updated = time.time()

    def remove_node_links(self, node_id: int):
        """Remove all link tracking state for a kicked node.
        Called when KICKED line received from serial — prevents stale
        link_broken events for intentionally removed nodes."""
        node_hex = f"0x{node_id:02X}"
        to_remove = [lk for lk in self._known_links if node_hex in lk]
        for lk in to_remove:
            self._known_links.discard(lk)
            self._link_fail_count.pop(lk, None)
            self.link_rssi.pop(lk, None)
            self.rssi_history.pop(lk, None)
        # Also clean current_rreq_links so no in-flight cycle references remain
        stale = [lk for lk in self._current_rreq_links if node_hex in lk]
        for lk in stale:
            self._current_rreq_links.discard(lk)

    def _handle_heartbeat(self, src, dest, count):
        """Process HB alerts from mainTerm (count=0 online, count=0xFF offline)"""
        if count == 0xFF:
            self._last_offline = {"node": src, "time": time.time()}
        elif count == 0:
            self._last_online = {"node": src, "time": time.time()}

    def record_bad(self):
        self.bad += 1

    def get_pending_events(self):
        """取出并清空待广播的事件列表（离线检测等）"""
        events = self._pending_events[:]
        self._pending_events = []
        return events

    @property
    def average_rssi(self):
        """所有链路当前 RSSI 的平均值"""
        if not self.link_rssi:
            return None
        vals = list(self.link_rssi.values())
        return round(sum(vals) / len(vals), 1)

    @property
    def success_rate(self):
        """通信成功率：完整走完 4 阶段的次数 / 总建立通信次数"""
        if self.cycle_attempts == 0:
            return None
        return round(self.cycle_successes / self.cycle_attempts * 100, 1)

    def get_rssi_history(self, link_key: str = None):
        """返回 RSSI 历史数据，用于前端折线图"""
        result = {}
        keys = [link_key] if link_key else list(self.rssi_history.keys())
        for k in keys:
            result[k] = [{"t": t, "rssi": r} for t, r in self.rssi_history[k]]
        return result

    def get_node_details(self, node_id: int):
        """返回单节点详细信息"""
        s = self.node_stats[node_id]
        role = NODE_ROLES.get(node_id, {"role":"未知","en":"unknown","color":"#6b7280"})
        # 找出涉及该节点的所有链路
        node_hex = f"0x{node_id:02X}"
        links = []
        for k, v in self.link_rssi.items():
            if node_hex in k:
                links.append({"link": k, "rssi": v})
        return {
            "id": node_id, "hexId": node_hex,
            "role": role["role"], "roleEn": role["en"],
            "sent": s["sent"], "received": s["received"],
            "firstSeen": s["firstSeen"], "lastSeen": s["lastSeen"],
            "links": links,
        }

    def to_dict(self):
        return {
            "totalFrames": self.total_frames,
            "rreqCount":   self.rreq,
            "rrepCount":   self.rrep,
            "dataCount":   self.data,
            "ackCount":    self.ack,
            "badFrames":   self.bad,
            "avgRssi":     self.average_rssi,
            "successRate": self.success_rate,
            "cycleAttempts": self.cycle_attempts,
            "cycleSuccesses": self.cycle_successes,
            "nodesSeen":   len(self.nodes_seen),
            "currentRoute": self.current_route,
            "currentPhase": self.current_phase,
            "hbOnline": self._last_online,
            "hbOffline": self._last_offline,
            "phaseUpdated": self.phase_updated,
            "linkRssi":    self.link_rssi,
            "nodeStats":   {f"0x{k:02X}": dict(v) for k, v in self.node_stats.items()},
            "rssiHistory": self.get_rssi_history(),
        }

stats = StatsTracker()

# ★ 两阶段提交去重：PREPARE/COMMIT 重试会产生大量重复事件
_two_phase_seen_ready = set()   # 本轮已上报 READY 的节点
_two_phase_commit_sent = False  # 本轮是否已发 COMMIT 进度
_two_phase_prepare_sent = False # 本轮是否已发 PREPARE 进度

# ★ 节点状态缓存：各节点最新上报的运行时状态
_node_status_cache = {}  # {nodeId: {uptime, freeRAM, txPkts, rxPkts, freq, sf, updated}}

# ════════════════════ WebSocket 连接管理 ════════════════════
class ConnectionManager:
    def __init__(self):
        self.active: list[WebSocket] = []
    async def connect(self, ws: WebSocket):
        await ws.accept(); self.active.append(ws)
    def disconnect(self, ws: WebSocket):
        if ws in self.active: self.active.remove(ws)
    async def broadcast(self, msg: str):
        for ws in self.active:
            try: await ws.send_text(msg)
            except Exception: pass
    async def broadcast_stats(self):
        await self.broadcast(json.dumps({"type":"stats","data":stats.to_dict()}))

manager = ConnectionManager()

# ════════════════════ FastAPI ════════════════════
app = FastAPI(title="通信系统综合设计与实践")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True,
                   allow_methods=["*"], allow_headers=["*"])
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

@app.get("/")
async def serve_index():
    return FileResponse(os.path.join(BASE_DIR, "index.html"))

@app.get("/api/ports")
def get_ports():
    return {"ports": [p.device for p in serial.tools.list_ports.comports()]}

@app.get("/api/stats")
def get_stats():
    return stats.to_dict()

@app.get("/api/reset_stats")
def reset_stats():
    stats.reset(); return {"status":"ok","msg":"统计已重置"}

@app.get("/api/nodes")
def get_nodes():
    return {"nodes": {f"0x{k:02X}": stats.get_node_details(k) for k in stats.node_stats}}

@app.get("/api/node/{node_id}")
def get_node(node_id: int):
    if node_id not in stats.node_stats:
        return {"error": "节点不存在"}
    return stats.get_node_details(node_id)

@app.get("/api/links")
def get_links():
    return {"links": stats.link_rssi, "history": stats.get_rssi_history()}

# ════════════════════ 串口读取线程 ════════════════════
serial_port_obj = None
serial_is_running = False

# 串口文本日志关键字过滤 — 只保留有用信息，丢弃无用噪音
_TEXT_LOG_KEYWORDS = [
    "TOPO", "STATS", "ROUTE", "RSSI", "JOIN", "ERR", "WARN",
    "FAIL", "ERROR", "NET", "CFG", "NODE", "LINK", "PATH",
    "SEND", "RECV", "ACK", "DATA", "RREQ", "RREP",
    "MISS", "RETRY",  # sender 缺失中继重试日志
    "WL", "WHITELIST",  # 白名单管理
    "KICKED", "UNKICKED",  # 踢出/恢复通知
]


def _is_useful_log(line: str) -> bool:
    """判断串口文本行是否包含有用信息"""
    upper = line.upper()
    for kw in _TEXT_LOG_KEYWORDS:
        if kw in upper:
            return True
    return False

def read_serial_thread(port_name: str, baudrate: int, loop):
    global serial_port_obj, serial_is_running
    try:
        serial_port_obj = serial.Serial(port_name, baudrate, timeout=1)
        asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
            "type":"sys","msg":f"✅ 串口 {port_name} 已连接 (波特率 {baudrate})"})), loop)
        buffer = bytearray()
        text_buf = bytearray()
        while serial_is_running:
            if serial_port_obj.in_waiting > 0:
                raw = serial_port_obj.read(serial_port_obj.in_waiting)
                buffer.extend(raw)
                # 同时收集文本行（用于 mainTerm 的 [TOPO] [STATS] 等应答）
                for b in raw:
                    if b == 0x0A:  # \n
                        try:
                            line = text_buf.decode('utf-8', errors='replace').strip()
                            # ★ WL:N: 条目计数 → 记录预期数量，wl_end 时传递给前端校验
                            if line.startswith("WL:N:"):
                                try:
                                    read_serial_thread._wl_expected = int(line[5:])
                                except: pass
                            # ★ WL: 白名单数据 → 结构化事件，绕过关键字过滤
                            elif line.startswith("WL:") and line != "WL:END":
                                parts = line[3:].split(":")
                                if len(parts) >= 2:
                                    nid = int(parts[0], 16)
                                    kicked = int(parts[2]) == 1 if len(parts) >= 3 else False
                                    asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                        "type":"wl_entry","nodeId":nid,
                                        "nodeHex":f"0x{nid:02X}","role":int(parts[1]),
                                        "kicked": kicked
                                    })), loop)
                            elif line == "WL:END":
                                asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                    "type":"wl_end",
                                    "expectedCount": getattr(read_serial_thread, '_wl_expected', 0)
                                })), loop)
                                read_serial_thread._wl_expected = 0  # 重置
                            # ★ KICKED 输出: "KICKED 0x21"
                            elif line.startswith("KICKED 0x") or line.startswith("KICKED 0X"):
                                try:
                                    kid = int(line.split("0x")[1].strip(), 16)
                                    asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                        "type":"kicked","nodeId":kid,
                                        "nodeHex":f"0x{kid:02X}"
                                    })), loop)
                                    stats.remove_node_links(kid)  # 清理链路追踪，防止后续 link_broken
                                except: pass
                            elif line.startswith("UNKICKED 0x") or line.startswith("UNKICKED 0X"):
                                try:
                                    uid = int(line.split("0x")[1].strip(), 16)
                                    asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                        "type":"unkicked","nodeId":uid,
                                        "nodeHex":f"0x{uid:02X}"
                                    })), loop)
                                except: pass
                            # ★ mainTerm 自身状态上报: NODE_STATUS:uptime:freeRAM:txPkts:rxPkts:freq:sf
                            elif line.startswith("NODE_STATUS:"):
                                try:
                                    parts = line[12:].split(":")
                                    if len(parts) >= 6:
                                        nid = 0x10
                                        uptime = int(parts[0])
                                        free_ram = int(parts[1])
                                        tx_pkts = int(parts[2])
                                        rx_pkts = int(parts[3])
                                        freq = int(parts[4])
                                        sf = int(parts[5])
                                        _node_status_cache[nid] = {
                                            "uptime": uptime, "freeRAM": free_ram,
                                            "txPkts": tx_pkts, "rxPkts": rx_pkts,
                                            "freq": freq, "sf": sf, "updated": time.time()
                                        }
                                        asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                            "type": "node_status",
                                            "nodeId": nid,
                                            "nodeHex": f"0x{nid:02X}",
                                            "uptime": uptime,
                                            "freeRAM": free_ram,
                                            "txPkts": tx_pkts,
                                            "rxPkts": rx_pkts,
                                            "freq": freq,
                                            "sf": sf,
                                        })), loop)
                                except: pass
                            elif line and _is_useful_log(line):
                                asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                    "type":"text_log","text":line,"time":time.time()
                                })), loop)
                        except: pass
                        text_buf = bytearray()
                    elif b >= 0x20 or b == 0x0D:  # printable or \r
                        text_buf.append(b)

                while len(buffer) >= FRAME_SIZE:
                    if buffer[0] == 0x4C and buffer[1] == 0x6F:
                        frame_data = buffer[:FRAME_SIZE]
                        try:
                            head, src, dest, msg_type, path_id, count, data_bytes, checksum = \
                                struct.unpack(FRAME_FORMAT, frame_data)
                            stamps, relays = [], []
                            last_hop = None
                            # ★ 处理两阶段提交进度事件（即使 dest 为广播地址也需推送）
                            skip_normal = (dest == BROADCAST_ID or src == BROADCAST_ID)
                            if skip_normal:
                                if msg_type == MSG_CMD_PREPARE:
                                    if not _two_phase_prepare_sent:
                                        _two_phase_prepare_sent = True
                                        _two_phase_seen_ready.clear()
                                        _two_phase_commit_sent = False
                                        freq = (data_bytes[0] << 8) | data_bytes[1]
                                        sf = data_bytes[2]
                                        asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                            "type": "two_phase_progress",
                                            "phase": "prepare",
                                            "freq": freq,
                                            "sf": sf,
                                            "msg": f"主节点发起全网信道切换: {freq}MHz SF{sf}"
                                        })), loop)
                                elif msg_type == MSG_CMD_COMMIT and not _two_phase_commit_sent:
                                    _two_phase_commit_sent = True
                                    freq = (data_bytes[0] << 8) | data_bytes[1]
                                    sf = data_bytes[2]
                                    asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                        "type": "two_phase_progress",
                                        "phase": "commit",
                                        "freq": freq,
                                        "sf": sf,
                                        "msg": "正在执行全网信道切换..."
                                    })), loop)
                                del buffer[:FRAME_SIZE]
                                continue
                            if msg_type == MSG_RREQ:
                                for i in range(min(count, 4)):
                                    rid, r = data_bytes[i*2], to_int8(data_bytes[i*2+1])
                                    if rid or r: stamps.append({"id":f"0x{rid:02X}","rssi":r})
                                if count < 4:
                                    last_hop = to_int8(data_bytes[7])
                            elif msg_type in (MSG_RREP, MSG_DATA, MSG_ACK, MSG_ACK_CONFIRM):
                                for i in range(min(count, 8)):
                                    if data_bytes[i]: relays.append(data_bytes[i])
                                # RREP/ACK/ACK_CONFIRM: data[7] 无真实 RSSI；只有 DATA 有
                                if msg_type == MSG_DATA and count < 4:
                                    last_hop = to_int8(data_bytes[7])
                            stats.record_frame(src, dest, msg_type, path_id, count, stamps, relays)
                            # ★ 广播由 record_frame 产生的待处理事件（如离线检测）
                            for evt in stats.get_pending_events():
                                asyncio.run_coroutine_threadsafe(
                                    manager.broadcast(json.dumps(evt)), loop)
                            frame_msg = {
                                "type":"frame","src":src,"dest":dest,"msgType":msg_type,
                                "msgTypeName":MSG_NAMES.get(msg_type,"UNKN"),
                                "pathId":path_id,"count":count,
                                "rssiStamps":stamps,"relays":relays,
                                "dataHex":data_bytes.hex().upper(),"checksum":checksum,
                                "stats":stats.to_dict(),
                            }
                            if last_hop is not None:
                                frame_msg["lastHopRssi"] = last_hop
                            # ★ 切换结果帧不广播原始帧（已有 channel_switch_result 专用事件）
                            is_commit_result = (msg_type == MSG_CMD_COMMIT and data_bytes[0] == 0x01)
                            if not is_commit_result:
                                asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps(frame_msg)), loop)
                            # === Two-phase commit and channel switch handling ===
                            if msg_type == MSG_CMD_READY:
                                # READY: data[0]=status  ★ PREPARE 重试导致重复，去重
                                if src not in _two_phase_seen_ready:
                                    _two_phase_seen_ready.add(src)
                                    status = data_bytes[0]
                                    asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                        "type": "two_phase_progress",
                                        "phase": "ready",
                                        "nodeId": src,
                                        "nodeHex": f"0x{src:02X}",
                                        "status": status,
                                        "msg": f"节点 0x{src:02X} {'已就绪' if status==0 else '参数不支持'}"
                                    })), loop)
                            elif msg_type == MSG_CMD_PREPARE:
                                if not _two_phase_prepare_sent:
                                    _two_phase_prepare_sent = True
                                    _two_phase_seen_ready.clear()
                                    _two_phase_commit_sent = False
                                    freq = (data_bytes[0] << 8) | data_bytes[1]
                                    sf = data_bytes[2]
                                    asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                        "type": "two_phase_progress",
                                        "phase": "prepare",
                                        "freq": freq,
                                        "sf": sf,
                                        "msg": f"主节点发起全网信道切换: {freq}MHz SF{sf}"
                                    })), loop)
                            elif msg_type == MSG_CMD_COMMIT:
                                if data_bytes[0] == 0x01:
                                    # 切换结果帧
                                    readyMask = data_bytes[1]
                                    onlineMask = data_bytes[2]
                                    total = 3
                                    onlineCount = bin(onlineMask).count("1")
                                    # ★ 逐节点在线状态
                                    nodes_status = {
                                        "0x21": {"ready": bool(readyMask & 0x01), "online": bool(onlineMask & 0x01)},
                                        "0x22": {"ready": bool(readyMask & 0x02), "online": bool(onlineMask & 0x02)},
                                        "0x30": {"ready": bool(readyMask & 0x04), "online": bool(onlineMask & 0x04)},
                                    }
                                    asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                        "type": "channel_switch_result",
                                        "success": onlineCount == total,
                                        "onlineCount": onlineCount,
                                        "totalNodes": total,
                                        "readyMask": readyMask,
                                        "onlineMask": onlineMask,
                                        "nodes": nodes_status,
                                        "msg": f"切换完成: {onlineCount}/{total} 节点在线"
                                    })), loop)
                                    _two_phase_prepare_sent = False  # ★ 本轮结束，允许下一轮 PREPARE
                                elif not _two_phase_commit_sent:
                                    _two_phase_commit_sent = True
                                    freq = (data_bytes[0] << 8) | data_bytes[1]
                                    sf = data_bytes[2]
                                    asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                        "type": "two_phase_progress",
                                        "phase": "commit",
                                        "freq": freq,
                                        "sf": sf,
                                        "msg": "正在执行全网信道切换..."
                                    })), loop)
                            # ★ 节点状态上报帧（relayTerm/sender → LoRa → mainTerm → Serial）
                            if msg_type == MSG_NODE_STATUS:
                                uptime = (data_bytes[0] << 8) | data_bytes[1]
                                free_ram = (data_bytes[2] << 8) | data_bytes[3]
                                tx_pkts = (data_bytes[4] << 8) | data_bytes[5]
                                rx_pkts = (data_bytes[6] << 8) | data_bytes[7]
                                _node_status_cache[src] = {
                                    "uptime": uptime, "freeRAM": free_ram,
                                    "txPkts": tx_pkts, "rxPkts": rx_pkts,
                                    "updated": time.time()
                                }
                                asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                                    "type": "node_status",
                                    "nodeId": src,
                                    "nodeHex": f"0x{src:02X}",
                                    "uptime": uptime,
                                    "freeRAM": free_ram,
                                    "txPkts": tx_pkts,
                                    "rxPkts": rx_pkts,
                                })), loop)
                            del buffer[:FRAME_SIZE]
                        except Exception:
                            del buffer[0:1]
                    else:
                        del buffer[0:1]
    except Exception as e:
        asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
            "type":"sys_error","msg":f"串口错误: {e}"})), loop)
    finally:
        if serial_port_obj and serial_port_obj.is_open:
            serial_port_obj.close()
        asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
            "type":"sys","msg":f"🔌 串口 {port_name} 已断开"})), loop)

# ════════════════════ WebSocket ════════════════════
@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    global serial_is_running
    await manager.connect(websocket)
    loop = asyncio.get_running_loop()
    await websocket.send_text(json.dumps({"type":"stats","data":stats.to_dict()}))
    try:
        while True:
            cmd = json.loads(await websocket.receive_text())
            if cmd.get("action") == "open_serial":
                port, baud = cmd.get("port",""), int(cmd.get("baudrate",9600))
                if serial_is_running:
                    serial_is_running = False; await asyncio.sleep(0.3)
                serial_is_running = True
                threading.Thread(target=read_serial_thread, args=(port,baud,loop), daemon=True).start()
            elif cmd.get("action") == "close_serial":
                serial_is_running = False
            elif cmd.get("action") == "reset_stats":
                stats.reset(); await manager.broadcast_stats()
            elif cmd.get("action") == "send_command":
                if serial_port_obj and serial_port_obj.is_open:
                    serial_port_obj.write((cmd.get("command","")+"\n").encode())
                    await websocket.send_text(json.dumps({
                        "type":"sys","msg":f"📤 命令已发送: {cmd.get('command','')}"}))
                else:
                    await websocket.send_text(json.dumps({
                        "type":"sys_error","msg":"❌ 串口未打开，无法发送命令。请先点击「🔌 打开串口」"}))
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# ════════════════════ 定时推送 ════════════════════
ACK_TIMEOUT_SEC = 4.0  # ACK 阶段超过此时间未收到 ACK_CONFIRM 视为失败
# ★ 各阶段超时阈值（秒）— 卡在任何阶段过久均视作传输失败
PHASE_TIMEOUTS = {
    "rreq": 6.0,   # RREQ: sender 最多 3 次重试 × 2s + 退避 ≈ 7s，6s 起检
    "rrep": 5.0,   # RREP: 正常 <1s，超过 5s 异常
    "data": 5.0,   # DATA: 正常 <1s，超过 5s 异常
    "ack":  4.0,   # ACK: 3 次重传 × 800ms ≈ 2.4s，4s 起检
}
async def periodic_stats():
    while True:
        await asyncio.sleep(3)
        try: await manager.broadcast_stats()
        except Exception: pass
        # ★ 广播待处理事件（如离线检测，兜底保障）
        for evt in stats.get_pending_events():
            try: await manager.broadcast(json.dumps(evt))
            except Exception: pass
        # ★ 各阶段超时检测：卡在任何阶段过久均视作传输失败
        phase = stats.current_phase
        if phase in PHASE_TIMEOUTS:
            elapsed = time.time() - stats.phase_updated
            if elapsed > PHASE_TIMEOUTS[phase]:
                route_info = stats.current_route or {}
                phase_names = {"rreq":"路由发现","rrep":"路由应答","data":"数据传输","ack":"确认应答"}
                cn_name = phase_names.get(phase, phase)
                await manager.broadcast(json.dumps({
                    "type": "phase_timeout",
                    "phase": phase,
                    "pathId": route_info.get("pathId", 0),
                    "relays": route_info.get("relays", []),
                    "msg": f"{cn_name}({phase}) 阶段超时 — 传输失败"
                }))
                stats.mark_cycle_failed(phase)  # 计入失败 + 保持告警状态
                stats.phase_updated = time.time()

@app.on_event("startup")
async def on_startup():
    asyncio.create_task(periodic_stats())

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)
