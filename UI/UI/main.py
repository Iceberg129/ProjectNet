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
MSG_JOIN_REQ = 0x20; MSG_JOIN_ACK = 0x21; MSG_JOIN_REJ = 0x22
MSG_NAMES = {MSG_RREQ:"RREQ",MSG_RREP:"RREP",MSG_DATA:"DATA",MSG_ACK:"ACK",
             MSG_HEARTBEAT:"HB",MSG_CMD:"CMD",MSG_CMD_ACK:"CMD_ACK",
             MSG_JOIN_REQ:"JOIN",MSG_JOIN_ACK:"JOIN_OK",MSG_JOIN_REJ:"JOIN_NO",
             MSG_ACK_CONFIRM:"ACK_CFM"}
MSG_ZH    = {MSG_RREQ:"路由发现",MSG_RREP:"路由应答",MSG_DATA:"数据传输",MSG_ACK:"确认应答",
             MSG_HEARTBEAT:"心跳",MSG_CMD:"下行命令",MSG_CMD_ACK:"命令应答",
             MSG_JOIN_REQ:"入网请求",MSG_JOIN_ACK:"入网允许",MSG_JOIN_REJ:"入网拒绝",
             MSG_ACK_CONFIRM:"ACK确认回执"}

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
        self.phase_updated = time.time()

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
        elif msg_type == MSG_JOIN_REQ:  pass  # 入网请求不计数主类型
        elif msg_type == MSG_HEARTBEAT: pass
        else:                           self.bad  += 1

        # 链路 RSSI（从印章逐跳解析）
        if stamps:
            prev = src
            for s in stamps:
                rid = int(s["id"], 16)
                key = self._link_key(prev, rid)
                self.link_rssi[key] = s["rssi"]
                self.rssi_history[key].append((now, s["rssi"]))
                # 更新中继节点统计
                self.nodes_seen.add(rid)
                ns = self.node_stats[rid]
                ns["lastSeen"] = now
                if ns["firstSeen"] == 0:
                    ns["firstSeen"] = now
                prev = rid
            # 最终跳 → dest
            if prev != dest:
                key = self._link_key(prev, dest)
                self.rssi_history[key].append((now, None))

        # 更新路由信息
        if msg_type == MSG_RREP:
            self.current_route = {"pathId": path_id, "relays": relays, "updated": now}

        # 更新通信阶段
        if   msg_type == MSG_RREQ: self._set_phase("rreq", now)
        elif msg_type == MSG_RREP: self._set_phase("rrep", now)
        elif msg_type == MSG_DATA: self._set_phase("data", now)
        elif msg_type == MSG_ACK:  self._set_phase("ack", now)
        elif msg_type == MSG_ACK_CONFIRM: self._set_phase("ack_done", now)

    def _set_phase(self, phase, now):
        self.current_phase = phase
        self.phase_updated = now

    def record_bad(self):
        self.bad += 1

    @property
    def average_rssi(self):
        """所有链路当前 RSSI 的平均值"""
        if not self.link_rssi:
            return None
        vals = list(self.link_rssi.values())
        return round(sum(vals) / len(vals), 1)

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
            "nodesSeen":   len(self.nodes_seen),
            "currentRoute": self.current_route,
            "currentPhase": self.current_phase,
            "phaseUpdated": self.phase_updated,
            "linkRssi":    self.link_rssi,
            "nodeStats":   {f"0x{k:02X}": dict(v) for k, v in self.node_stats.items()},
            "rssiHistory": self.get_rssi_history(),
        }

stats = StatsTracker()

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
                            if line and _is_useful_log(line):
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
                            asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps(frame_msg)), loop)
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
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# ════════════════════ 模拟测试（增强版：RSSI 随时间变化） ════════════════════
simulate_stop_event = asyncio.Event()     # 取消信号
simulate_lock = asyncio.Lock()            # 防止并发模拟

async def _check_cancel():
    """检查是否被取消，若取消则抛出异常中断模拟"""
    if simulate_stop_event.is_set():
        raise asyncio.CancelledError("模拟已取消")

@app.get("/api/stop_simulate")
async def stop_simulate():
    """停止正在运行的模拟"""
    simulate_stop_event.set()
    return {"status":"ok","msg":"已发送停止信号"}

@app.get("/api/simulate")
async def simulate_traffic():
    # 防止并发模拟
    if simulate_lock.locked():
        return {"status":"busy","msg":"模拟正在运行中，请先停止"}
    async with simulate_lock:
        simulate_stop_event.clear()  # 重置取消信号
        loop = asyncio.get_running_loop()

        def bcast(obj):
            obj.setdefault("stats", stats.to_dict())
            asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps(obj)), loop)

        def rec(src, dest, mtype, pid, cnt, stamps, relays):
            stats.record_frame(src, dest, mtype, pid, cnt, stamps, relays)

        def stamps_hex(stamps):
            h = ""
            for s in stamps:
                h += f"{int(s['id'],16):02X}{u8h(s['rssi'])}"
            return h.ljust(16, '0')

        try:
            # ── 第一阶段：节点入网 ──
            for node_id, role, role_name in [(0x21,1,"relay21"),(0x22,1,"relay22"),(0x30,2,"sender")]:
                await _check_cancel()
                rec(node_id, 0x10, MSG_JOIN_REQ, 0, 0, [], [])
                bcast({"type":"frame","src":node_id,"dest":0x10,"msgType":MSG_JOIN_REQ,"msgTypeName":"JOIN",
                       "pathId":0,"count":0,"rssiStamps":[],"relays":[],
                       "dataHex":f"0{role}00000000000000","checksum":0})
                await asyncio.sleep(0.08)
                await _check_cancel()
                rec(0x10, node_id, MSG_JOIN_ACK, 0, 0, [], [])
                bcast({"type":"frame","src":0x10,"dest":node_id,"msgType":MSG_JOIN_ACK,"msgTypeName":"JOIN_OK",
                       "pathId":0,"count":0,"rssiStamps":[],"relays":[],
                       "dataHex":"AC00000000000000","checksum":0})
                await asyncio.sleep(0.08)
                await _check_cancel()
                # 心跳
                rec(node_id, 0x10, MSG_HEARTBEAT, 0, 0, [], [])
                bcast({"type":"frame","src":node_id,"dest":0x10,"msgType":MSG_HEARTBEAT,"msgTypeName":"HB",
                       "pathId":0,"count":0,"rssiStamps":[],"relays":[],
                       "dataHex":f"0{role}00000000000000","checksum":0})
                await asyncio.sleep(0.05)
            await asyncio.sleep(0.3)

            # 模拟 RSSI 随时间缓慢变化
            base_rssi = {"0x21": -48, "0x22": -55}
            rssi_drift = {"0x21": 0, "0x22": 0}

            def vary(relay: str) -> int:
                """让 RSSI 在 ±8 dB 内缓慢漂移"""
                import random
                rssi_drift[relay] += random.choice([-2,-1,0,1,2])
                rssi_drift[relay] = max(-8, min(8, rssi_drift[relay]))
                return base_rssi[relay] + rssi_drift[relay]

            choices = [
                ([0x21],    "00AB"),
                ([],        "01AB"),
                ([0x22],    "02AB"),
                ([0x21,0x22],"03AB"),
                ([0x21],    "04AB"),
                ([0x22],    "05AB"),
                ([],        "06AB"),
                ([0x21],    "07AB"),
            ]

            for pid, (chosen, payload) in enumerate(choices, start=1):
                await _check_cancel()
                r21_rssi = vary("0x21")
                r22_rssi = vary("0x22")
                r21_stamps = [{"id":"0x21","rssi":r21_rssi}]
                r22_stamps = [{"id":"0x22","rssi":r22_rssi}]
                double_stamps = [{"id":"0x21","rssi":r21_rssi},{"id":"0x22","rssi":r22_rssi+3}]

                # RREQ 直达
                rec(0x30, 0x10, MSG_RREQ, pid, 0, [], [])
                bcast({"type":"frame","src":0x30,"dest":0x10,"msgType":MSG_RREQ,"msgTypeName":"RREQ",
                       "pathId":pid,"count":0,"rssiStamps":[],"relays":[],
                       "dataHex":"0000000000000000","checksum":0})
                await asyncio.sleep(0.10)
                await _check_cancel()

                # RREQ 经 0x21
                rec(0x30, 0x10, MSG_RREQ, pid, 1, r21_stamps, [])
                bcast({"type":"frame","src":0x30,"dest":0x10,"msgType":MSG_RREQ,"msgTypeName":"RREQ",
                       "pathId":pid,"count":1,"rssiStamps":r21_stamps,"relays":[],
                       "dataHex":stamps_hex(r21_stamps),"checksum":0})
                await asyncio.sleep(0.10)
                await _check_cancel()

                # RREQ 经 0x22
                rec(0x30, 0x10, MSG_RREQ, pid, 1, r22_stamps, [])
                bcast({"type":"frame","src":0x30,"dest":0x10,"msgType":MSG_RREQ,"msgTypeName":"RREQ",
                       "pathId":pid,"count":1,"rssiStamps":r22_stamps,"relays":[],
                       "dataHex":stamps_hex(r22_stamps),"checksum":0})
                await asyncio.sleep(0.10)
                await _check_cancel()

                # RREQ 双跳
                rec(0x30, 0x10, MSG_RREQ, pid, 2, double_stamps, [])
                bcast({"type":"frame","src":0x30,"dest":0x10,"msgType":MSG_RREQ,"msgTypeName":"RREQ",
                       "pathId":pid,"count":2,"rssiStamps":double_stamps,"relays":[],
                       "dataHex":stamps_hex(double_stamps),"checksum":0})
                await asyncio.sleep(0.10)
                await _check_cancel()

                # 裁决窗口
                await asyncio.sleep(0.20)
                await _check_cancel()

                # RREP
                dhex = "".join(f"{c:02X}" for c in chosen).ljust(16, '0')
                rec(0x10, 0x30, MSG_RREP, pid, len(chosen), [], chosen)
                bcast({"type":"frame","src":0x10,"dest":0x30,"msgType":MSG_RREP,"msgTypeName":"RREP",
                       "pathId":pid,"count":len(chosen),"rssiStamps":[],"relays":chosen,
                       "dataHex":dhex,"checksum":0})
                await asyncio.sleep(0.12)
                for _ in chosen:
                    bcast({"type":"frame","src":0x10,"dest":0x30,"msgType":MSG_RREP,"msgTypeName":"RREP",
                           "pathId":pid,"count":len(chosen),"rssiStamps":[],"relays":chosen,
                           "dataHex":dhex,"checksum":0})
                    await asyncio.sleep(0.06)

                # DATA
                dhex = ("".join(f"{c:02X}" for c in chosen) + payload).ljust(16, '0')
                rec(0x30, 0x10, MSG_DATA, pid, len(chosen), [], chosen)
                bcast({"type":"frame","src":0x30,"dest":0x10,"msgType":MSG_DATA,"msgTypeName":"DATA",
                       "pathId":pid,"count":len(chosen),"rssiStamps":[],"relays":chosen,
                       "dataHex":dhex,"checksum":0})
                await asyncio.sleep(0.12)
                await _check_cancel()

                # ACK 超时重传（最多 3 次）
                import random
                ack_sent = 0
                ack_success = False
                while ack_sent < 3 and not ack_success:
                    await _check_cancel()
                    ack_sent += 1
                    dhex = "".join(f"{c:02X}" for c in chosen).ljust(16, '0')
                    rec(0x10, 0x30, MSG_ACK, pid, len(chosen), [], chosen)
                    ack_frame = {"type":"frame","src":0x10,"dest":0x30,"msgType":MSG_ACK,"msgTypeName":"ACK",
                           "pathId":pid,"count":len(chosen),"rssiStamps":[],"relays":chosen,
                           "dataHex":dhex,"checksum":0}
                    if ack_sent > 1:
                        ack_frame["_retry"] = ack_sent  # 前端可展示重传次数
                    bcast(ack_frame)
                    # 模拟超时等待（150~250ms）
                    await asyncio.sleep(0.18)
                    await _check_cancel()
                    # 模拟确认：第1次 70% 成功率，第2次 90%，第3次必然成功
                    success_rate = 0.7 if ack_sent == 1 else 0.9 if ack_sent == 2 else 1.0
                    ack_success = random.random() < success_rate

                # ACK 成功 → sender 回 ACK_CONFIRM
                if ack_success:
                    await asyncio.sleep(0.05)
                    await _check_cancel()
                    cfm_hex = "".join(f"{c:02X}" for c in chosen).ljust(16, '0')
                    bcast({"type":"frame","src":0x30,"dest":0x10,"msgType":MSG_ACK_CONFIRM,
                           "msgTypeName":"ACK_CFM","pathId":pid,"count":len(chosen),
                           "rssiStamps":[],"relays":chosen,"dataHex":cfm_hex,"checksum":0})
                else:
                    # ACK 重传全部耗尽 → 路由失效
                    stats.current_route = None
                    stats.current_phase = "ack_fail"
                    stats.phase_updated = time.time()
                    bcast({"type":"route_fail","pathId":pid,"relays":chosen,"msg":"ACK 重传耗尽，路由失效"})

            return {"status":"ok","msg":f"✅ 模拟 {len(choices)} 轮 AODV-Lite 通信完成（含 RSSI 漂移）","rounds":len(choices)}
        except asyncio.CancelledError:
            # 广播取消通知
            asyncio.run_coroutine_threadsafe(manager.broadcast(json.dumps({
                "type":"sys","msg":"⏹ 模拟已手动停止"
            })), loop)
            return {"status":"cancelled","msg":"模拟已停止","rounds":0}

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
                stats.current_phase = f"{phase}_fail"  # 保持告警状态，直到下轮 RREQ 重置
                stats.phase_updated = time.time()

@app.on_event("startup")
async def on_startup():
    asyncio.create_task(periodic_stats())

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)
