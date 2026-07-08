# LoRa AODV-Lite 多跳通信系统

> **第 21 队** · 黄鹏峻 · 吴汉鹏 · 林金鑫 · 贾笛  
> GitHub: [050415/ProjectNet](https://github.com/050415/ProjectNet)

基于 **Arduino Uno + LoRa SX1278** 的轻量按需距离矢量路由（AODV-Lite）系统，配套 **FastAPI + WebSocket + ECharts** 实时监控界面。适用于低功耗、多跳、窄带宽的物联网通信场景。

---

## 目录

- [架构](#架构)
- [硬件连接](#硬件连接)
- [通信流程](#通信流程)
- [帧格式](#帧格式)
- [消息类型与 data 编码](#消息类型与-data-编码)
- [路由算法](#路由算法)
- [两阶段信道切换](#两阶段信道切换)
- [节点入网与白名单](#节点入网与白名单)
- [心跳与节点状态](#心跳与节点状态)
- [串口协议](#串口协议)
- [Web 监控界面](#web-监控界面)
- [快速开始](#快速开始)
- [文件结构](#文件结构)
- [关键设计点](#关键设计点)

---

## 架构

```
sender(0x30) ── LoRa ── relay(0x21) ── LoRa ── relay(0x22) ── LoRa ── mainTerm(0x10) ── USB ── PC(Python+Web)
```

| 角色 | ID | 固件 | 硬件 | 职责 |
|------|----|------|------|------|
| **sender** (终端) | `0x30` | `sender/sender.ino` | Uno + SX1278 + DHT22 + 电位器 + 红外 | 每 10s 发起通信，采集传感器数据 |
| **relay** (中继) | `0x21` / `0x22` | `relayTerm/relayTerm.ino` | Uno + SX1278 | RSSI 盖章 + 非阻塞退避转发 |
| **mainTerm** (主控) | `0x10` | `mainTerm/mainTerm.ino` | Uno + SX1278 + USB | 瓶颈 RSSI 裁决 + 串口桥接 PC |

- **LoRa 参数**：530MHz / SF7 / BW 125kHz (默认) / 17dBm
- **拓扑**：线型 4 节点，最多支持 2 跳中继（`MAX_RELAYS = 2`）
- **广播地址**：`0xFF`，用于 RREQ / PREPARE / COMMIT 全网广播

### 节点状态机

```
sender:  JOINING → IDLE → WAITING_RREP → WAITING_ACK → IDLE (循环)
              ↓                                              ↑
           KICKED ──────────── UNKICK ──────────────────────→
```

- **KICKED**：被踢出后静默，不收发业务帧，仅监听 UNKICK 恢复命令
- **pauseAODV**：两阶段提交期间暂停正常通信，避免并发切换

---

## 硬件连接

### LoRa SX1278 → Arduino Uno

| SX1278 | Uno |
|--------|-----|
| VCC | 3.3V |
| GND | GND |
| NSS | D10 |
| MOSI | D11 |
| MISO | D12 |
| SCK | D13 |
| RST | D9 |
| DIO0 | D2 |

### sender 传感器

| 传感器 | Uno 引脚 | 说明 |
|--------|---------|------|
| DHT22 | D3 | 温湿度，手动时序读取，约 25ms |
| 电位器 | A0 | 模拟输入 0-1023，顺时针增大 |
| 红外避障 | D2 | LOW=触发 |

> **注意**：DHT22 读取期间全程关中断 (`noInterrupts`)，防止 LoRa 中断干扰时序。读取间隔 ≥ 2.5s。

---

## 通信流程

```
sender                              relay(s)                          mainTerm
  │                                    │                                  │
  │── RREQ (广播, count=0) ───────────→│                                  │
  │                                    │── RREQ (盖章 {id,rssi}) ───────→│
  │                                    │       …多路径到达…               │
  │                                    │                      收集 500ms 窗口
  │                                    │                      瓶颈 RSSI 裁决
  │←─ RREP (中继列表, 单播) ───────────│←─────────────────────────────────│
  │                                    │                                  │
  │── DATA (传感器+中继列表) ─────────→│── DATA (转发) ─────────────────→│
  │                                    │                      去重 + 记录
  │←─ ACK (确认, 发 3 份) ────────────│←─────────────────────────────────│
  │                                    │                                  │
  │── ACK_CONFIRM (确认回执) ─────────→│── ACK_CONFIRM ─────────────────→│
  │                                    │                      路由高亮确认
```

### 各阶段详解

| 阶段 | 发起方 | 关键参数 | 说明 |
|------|--------|----------|------|
| **RREQ** | sender | `pathId++`, `count=0` | 广播探路。中继逐跳盖章 `{relayId, rssi}`，最多 4 对 (8B) |
| **收集窗口** | mainTerm | 500ms | 等待多路径 RREQ 到达。`MAX_CANDIDATES=6` 条候选路径 |
| **RREP** | mainTerm | 单播沿选定路径返回 | 携带中继 ID 列表，sender 存入路由表 |
| **DATA** | sender | 传感器数据 5B | 温度(int8) + 湿度(uint8) + 电位(uint16) + 告警位(uint8) |
| **ACK** | mainTerm | 连发 3 份，间隔 800ms | 确保 sender 至少收到一份 |
| **ACK_CONFIRM** | sender | 收到 ACK 后触发 | 超时等待约 `40+count×40ms`，收到后 UI 高亮路由 |

### 可靠性与重试

| 机制 | 参数 | 说明 |
|------|------|------|
| **RREQ 撞包重试** | 最多 3 次，退避 180/300/420ms | 递增退避避免二次碰撞 |
| **缺失中继重试** | 最多 2 次 | 上轮有中继、本轮缺失时触发，防中继意外故障 |
| **ACK 超时重传** | 3500ms | sender 等待 ACK 超时后重发 RREQ |
| **路由老化** | 10s (`ROUTE_MAX_AGE`) | ACK 后主动清除路由，下一轮重新探测 |

---

## 帧格式

### 统一 16 字节帧 (`LoRaFrame`)

```
Byte:   0      1      2      3      4       5       6       7~14      15
     +------+------+------+------+-------+-------+-------+----------+---------+
     |head[0]|head[1]|srcId |destId|msgType|pathId | count | data[8]  |checksum |
     +------+------+------+------+-------+-------+-------+----------+---------+
       0x4C   0x6F
```

```c
#pragma pack(push, 1)
typedef struct {
  uint8_t head[2];    // 帧头同步字: 0x4C 0x6F ("Lo")
  uint8_t srcId;      // 源节点地址 (0x01~0xEF)
  uint8_t destId;     // 目的节点地址, 0xFF=广播
  uint8_t msgType;    // 消息类型码
  uint8_t pathId;     // 路由标识符 (sender 每轮递增)
  uint8_t count;      // 语义随 msgType 变化
  uint8_t data[8];    // 变长负载
  uint8_t checksum;   // XOR 校验和
} LoRaFrame;
#pragma pack(pop)
```

**Python 解包**：`FRAME_FORMAT = '<2sBBBBB8sB'`（小端序，共 16 字节）

### 校验和算法

```c
void calcChecksum(LoRaFrame* f) {
  f->checksum = 0;
  uint8_t c = 0;
  c ^= f->srcId;  c ^= f->destId;
  c ^= f->msgType; c ^= f->pathId;  c ^= f->count;
  for (uint8_t i = 0; i < 8; i++) c ^= f->data[i];
  f->checksum = c;
}
```

对 `srcId ⊕ destId ⊕ msgType ⊕ pathId ⊕ count ⊕ data[0..7]` 逐字节异或。帧头 (head[0..1]) 不参与校验。

---

## 消息类型与 data 编码

### 消息类型一览

| 码值 | 常量 | 中文含义 | 方向 |
|------|------|----------|------|
| `0x01` | `MSG_DATA` | 数据传输 | sender → mainTerm |
| `0x02` | `MSG_ACK` | 数据确认 (发 3 份) | mainTerm → sender |
| `0x03` | `MSG_HB` | 心跳保活 | 所有节点 → mainTerm |
| `0x04` | `MSG_CMD` | 下行命令 (单节点) | PC → 节点 |
| `0x05` | `MSG_CMD_ACK` | 命令应答 | 节点 → PC |
| `0x06` | `MSG_ACK_CONFIRM` | ACK 确认回执 | sender → mainTerm |
| `0x07` | `MSG_NODE_STATUS` | 节点状态上报 (已合并入 HB) | 节点 → mainTerm |
| `0x08` | `MSG_KICK` | 踢出命令 | mainTerm → 节点 |
| `0x09` | `MSG_UNKICK` | 恢复命令 | mainTerm → 节点 |
| `0x0A` | `MSG_CMD_PREPARE` | 信道准备 (2PC Phase 1) | mainTerm → 广播 |
| `0x0B` | `MSG_CMD_READY` | 信道就绪 (2PC Phase 1 ACK) | 节点 → mainTerm |
| `0x0C` | `MSG_CMD_COMMIT` | 信道切换 (2PC Phase 2) | mainTerm → 广播 |
| `0x10` | `MSG_RREQ` | 路由发现请求 | sender → 广播 |
| `0x11` | `MSG_RREP` | 路由发现应答 | mainTerm → sender |
| `0x20` | `MSG_JOIN_REQ` | 入网请求 | 新节点 → mainTerm |
| `0x21` | `MSG_JOIN_ACK` | 入网允许 | mainTerm → 节点 |
| `0x22` | `MSG_JOIN_REJ` | 入网拒绝 | mainTerm → 节点 |

### 各消息类型 data[8] 编码

#### RREQ (0x10) — 路由发现

| 字段 | 说明 |
|------|------|
| `count` | 已盖章数 (每个中继转发时 +1，最多 4) |
| `data[]` | `{relayId (1B), rssi (1B, int8)}` 印章对，最多 4 对 |

#### RREP (0x11) / ACK (0x02) / ACK_CONFIRM (0x06) — 路由应答 & 确认

| 字段 | 说明 |
|------|------|
| `count` | 中继节点数 (0=直达) |
| `data[0..count-1]` | 中继 ID 列表 (沿路径顺序) |

#### DATA (0x01) — 传感器数据

| 字段 | 说明 |
|------|------|
| `count` | 中继节点数 |
| `data[0..count-1]` | 中继 ID 列表 |
| `data[count+0]` | 温度 int8 °C |
| `data[count+1]` | 湿度 uint8 % |
| `data[count+2..count+3]` | 电位 uint16 (0-1023) |
| `data[count+4]` | 告警位：bit4=DHT有效, bit3=红外, bit2=电位低, bit1=高湿, bit0=高温 |

> **传感器编码示例**：`count=2` 时，`data[0..1]`=中继列表，`data[2]`=温度，`data[3]`=湿度，`data[4..5]`=电位，`data[6]`=告警位。

#### HB (0x03) — 富心跳

| data 偏移 | 字段 | 编码 | 说明 |
|-----------|------|------|------|
| `0` | uptime | `>>2` | 运行时间 (最大 ~1020s) |
| `1` | freeRAM | `>>3` | 空闲内存 (最大 ~2040B) |
| `2~3` | txPacketCount | uint16 BE | 累计发送包数 |
| `4~5` | rxPacketCount | uint16 BE | 累计接收包数 |
| `6` | lastHeardId | raw | 最近监听邻居 ID |
| `7` | lastHeardRssi | int8 | 该邻居 RSSI (dBm) |

#### CMD (0x04) — 下行命令 (单节点)

| data 偏移 | 字段 |
|-----------|------|
| `3~4` | 频率 uint16 (MHz) |
| `5` | SF (7~12) |
| `6` | 功率 (dBm) |

> `freq==0 && sf==0` 时仅调整功率，不改变频率/SF。

#### CMD_PREPARE (0x0A) / CMD_COMMIT (0x0C) — 信道切换

| data 偏移 | 字段 |
|-----------|------|
| `0~1` | 新频率 uint16 |
| `2` | 新 SF |
| `3` | TTL (初始值=2，中继每次 -1) |

#### CMD_READY (0x0B) — 节点就绪

| data 偏移 | 字段 |
|-----------|------|
| `0` | 状态：0=OK, 1=频率不合法, 2=SF 不合法 |

#### JOIN_REQ (0x20) — 入网请求

| data 偏移 | 字段 |
|-----------|------|
| `0` | 角色：1=中继, 2=终端 |

#### JOIN_ACK (0x21) / JOIN_REJ (0x22) — 入网响应

| data 偏移 | 字段 |
|-----------|------|
| `0` | ACK=0xAC, REJ=原因码 (0x01 角色不匹配, 0x02 不在白名单, 0x03 已踢出) |

#### KICK (0x08) / UNKICK (0x09)

| data 偏移 | 字段 |
|-----------|------|
| `0` | KICK=0x4B ('K'), UNKICK=0x55 ('U') |

---

## 路由算法

### 瓶颈 RSSI 选路

采用**木桶原理**：路径可靠度取决于最弱一跳。

```
路径评估:
  Candidate.bottleneckRssi = min(所有中继印章的 RSSI, 末跳 RSSI)

选路规则:
  1. 优先选 bottleneckRssi 最大的路径
  2. bottleneck 相等时，跳数少 (count 小) 的优先
  3. 跳数也相等时，pathId 大的优先（最新路径）
```

### 退避防碰撞 (CSMA/CA)

中继节点在转发前计算退避时间，避免多个中继同时发包碰撞：

```
退避时间 = myPos × 60 + 10 ms

其中 myPos = 当前节点在印章列表中的位置 (0-based)
```

- **RREQ**：`(MY_NODE_ID & 0x0F) × 60 + 5 ms`，多跳时用较小退避 `(MY_NODE_ID & 0x0F) × 10 + 5 ms`
- **PREPARE/COMMIT**：`(MY_NODE_ID & 0x0F) × 40 + 20 ms`

### 非阻塞转发队列

中继节点使用 pending 队列实现非阻塞转发（最多 4 个待发帧），发送期间持续收包。

```
struct PendingFwd {
  LoRaFrame frame;
  uint32_t  nextSend;    // 计划发送时间
  int8_t    retries;     // 剩余重试次数
  bool      active;
};
```

### 防重复转发

通过 `(srcId, destId, pathId, msgType)` 四元组去重，防止同一帧被同一中继重复转发（如 RREQ 广播回环）。

---

## 两阶段信道切换

全网信道协调切换，确保所有节点同步切换到新频率/SF，避免网络分裂。

### 协议流程

```
mainTerm                           relay/sender
   │                                     │
   │── PREPARE (广播, TTL=2) ────────────→│
   │   data=[freq, sf, ttl]              │ 各节点暂存参数
   │                                     │ 计算退避
   │←── READY (单播) ────────────────────│
   │   data[0]=0(OK) / 1(ERR) / 2(ERR)  │ 上报是否支持新参数
   │                                     │
   │  …收集所有节点 READY… 最多 3s+2次重试│
   │                                     │
   │── COMMIT (广播, TTL=2) ─────────────→│
   │   data=[freq, sf, ttl]              │ 执行切换并回复 ACK
   │                                     │
   │  …进入 VERIFYING 状态, 15s 超时…     │
   │  监听心跳确认所有节点在线             │
```

| 状态 | 值 | 说明 |
|------|-----|------|
| IDLE | 0 | 空闲 |
| WAITING_READY | 1 | 等待所有节点回复 READY (含 PREPARE 重试) |
| SENDING_COMMIT | 2 | 发送 COMMIT，等待节点 ACK |
| VERIFYING | 3 | 等待心跳确认切换成功 |

### 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `PREPARE_TIMEOUT` | 3s | 等待 READY 超时 |
| `PREPARE_RETRY_MS` | 1s | PREPARE 重试间隔 (最多 2 次重试 = 共发 3 次) |
| `VERIFY_TIMEOUT` | 15s | 验证阶段超时 (足够一轮心跳) |
| TTL | 2 | 跳数限制，每跳 -1，到 0 停止转发 |

---

## 节点入网与白名单

### 入网流程

```
sender                           mainTerm
  │                                  │
  │── JOIN_REQ (data[0]=角色) ──────→│
  │                                  │ 检查白名单 + 角色匹配 + 踢出状态
  │←── JOIN_ACK (data[0]=0xAC) ─────│  允许入网
  │  或                              │
  │←── JOIN_REJ (data[0]=原因码) ────│  拒绝入网
```

### 白名单管理 (mainTerm EEPROM)

白名单存储在 Arduino EEPROM 中，首字节为魔数 `0xA0`。

| 串口命令 | 说明 |
|----------|------|
| `WL?` | 查询白名单 |
| `WL+XX:Y` | 添加节点 `0xXX`，角色 Y (1=中继, 2=终端) |
| `WL-XX` | 删除节点 `0xXX` |
| `KICK:XX` | 踢出节点 `0xXX` |
| `UNKICK:XX` | 恢复节点 `0xXX` |

EEPROM 布局：
```
[0] = WL_MAGIC (0xA0)
[1] = 白名单条目数
[2..] = 每条 3 字节: {id (1B), role (1B), kicked_flag (1B)}
```

### 踢出七层隔离

被踢出节点在以下 7 个层面全部被拦截：
1. **JOIN** — JOIN_REQ 直接拒绝 (0x03)
2. **帧级** — 非 JOIN_REQ 帧 `wlFind()` 不通过则静默丢弃
3. **白名单** — 从内存列表中移除
4. **RREQ** — 印章中的踢出节点 ID 被跳过
5. **心跳** — 在线告警跳过踢出节点
6. **中继 ID** — 转发时跳过踢出的中继
7. **链路追踪** — RSSI 链路不包含踢出节点

---

## 心跳与节点状态

### 心跳机制

所有节点每 **10s ± 2s jitter** 向 mainTerm 发送富心跳帧 (MSG_HB)，内容包含运行时间、空闲内存、收发包计数、邻居 RSSI。

mainTerm 侧：
- 每次收到 HB 更新 `nodeLastSeen[id]`
- `HB_TIMEOUT = 24s`（连续 2 次丢失）判定离线
- 在线/离线变化时通过串口 `TOPO:HB_ONLINE` / `TOPO:HB_OFFLINE` 上报 PC

### 邻居 RSSI 嗅探

中继/终端节点在接收任何合法帧时，记录最新邻居的 ID 和 RSSI：

```c
lastHeardId   = rxFrame.srcId;
lastHeardRssi = rssi;
```

该信息在下一次心跳帧的 `data[6..7]` 中上报，供 UI 绘制链路质量。

> **HB RSSI 不覆盖原则**：心跳偷听到的弱信号不覆盖 RREQ 阶段已刷新的链路 RSSI，防止低质量信号污染路由表。

---

## 串口协议

mainTerm 通过 USB 串口 (9600bps) 与 PC 通信，使用**二进制帧透传 + 文本行命令**混合模式：

### PC → mainTerm (文本命令)

| 命令 | 说明 |
|------|------|
| `j` | 切换干扰模式 (调试用) |
| `s` | 打印统计信息 |
| `WL?` | 查询白名单 |
| `WL+XX:Y` | 添加节点 |
| `WL-XX` | 删除节点 |
| `KICK:XX` 或 `K:XX` | 踢出节点 |
| `UNKICK:XX` 或 `UK:XX` | 恢复节点 |
| `CFG:XX:P17` | 调整节点 XX 功率 |
| `CFG_ALL:F530:S8` | 全网信道切换 (两阶段提交) |

### mainTerm → PC (二进制帧透传)

所有 LoRaFrame (16B 原始字节) 通过 `Serial.write()` 透传给 Python 后端解析。

### mainTerm → PC (文本行)

| 文本行 | 说明 |
|--------|------|
| `NODE_STATUS:uptime:ram:tx:rx:freq:sf` | mainTerm 自身状态，15s 间隔 |
| `TOPO:HB_ONLINE:0xXX` | 节点上线通知 |
| `TOPO:HB_OFFLINE:0xXX` | 节点离线通知 |
| `JAMMER ON/OFF` | 干扰模式状态变更 |
| `OK` / `ERR` 系列 | 命令执行结果 |

### sender/relay → PC (通过 mainTerm 透传)

sender 和 relay 的串口输出（含传感器读数、JOIN 状态等）由 mainTerm 以文本行形式透传给 PC。

---

## Web 监控界面

**启动**：`cd UI/UI && py -3.12 main.py` → `http://127.0.0.1:8000`

### 界面功能

| 功能模块 | 说明 |
|----------|------|
| **拓扑图** | ECharts 力导向布局，实时节点/链路渲染。节点颜色标识角色，链路粗细+RSSI dBm 标注。ACK_CONFIRM 到达时路由高亮（蓝紫色脉冲），非 ACK 消息不触发路由高亮防止误导 |
| **RSSI 趋势图** | 自定义 SVG 绘制，120 秒滑动窗口，逐段渐变色（绿→黄→红）。离线期间自动断线留空，重连后继续。支持鼠标悬停查看精确值 |
| **消息流网格** | 40 格动画填充网格，每种消息类型不同颜色，悬停弹窗显示详情 + 点击跳转日志对应行 |
| **信道配置** | 单节点功率独立调节 + 全网两阶段提交信道切换（频率+SF），含超时和错误提示 |
| **白名单管理** | 在线添加/踢出/恢复节点，踢出节点在网络拓扑中变灰 |
| **会话日志** | `log/session-*.txt` 自动记录逐帧详情 + 事件 + 30s 周期统计快照 |
| **诊断报告** | 节点详情面板：运行时间、内存、收发统计、邻居 RSSI、在线状态 |

### API 端点

| 端点 | 方法 | 说明 |
|------|------|------|
| `/` | GET | 返回 Web 前端页面 |
| `/api/ports` | GET | 获取可用串口列表 |
| `/api/stats` | GET | 获取全局统计 (总帧数、各类型计数、RSSI 历史) |
| `/api/reset_stats` | GET | 重置统计数据 |
| `/api/nodes` | GET | 获取所有节点详情 |
| `/api/node/{id}` | GET | 获取单个节点详情 |
| `/api/links` | GET | 获取链路 RSSI 数据及历史 |
| `/ws` | WebSocket | 实时数据推送 |

### WebSocket 事件类型

| type | 说明 |
|------|------|
| `frame` | 每一帧的实时推送，含 src/dest/msgType/pathId/relays/rssi |
| `event` | 路由建立完成、节点上下线、入网结果等事件 |
| `stats` | 周期性统计快照 (1s 间隔) |
| `topo` | 拓扑结构变更通知 |
| `alert` | 传感器告警 (高温/高湿/电位低/红外触发) |

---

## 快速开始

### 环境要求

- **硬件**：4 块 Arduino Uno + 4 块 LoRa SX1278 + DHT22 + 电位器 + 红外避障模块
- **IDE**：Arduino IDE 1.8+ 或 VS Code + PlatformIO
- **Python**：3.10+ (推荐 3.12)
- **浏览器**：Chrome / Edge (支持 ECharts)

### 步骤

```bash
# 1. 连接硬件
#    按照「硬件连接」章节接线，确保所有 SX1278 天线已安装

# 2. 烧录固件
#    - relayTerm: 修改 MY_NODE_ID 为 0x21 或 0x22
#    - sender:    MY_NODE_ID=0x30, DEST_ID=0x10
#    - mainTerm:  MY_NODE_ID=0x10
#    按角色分别烧录到对应 Arduino

# 3. 安装 Python 依赖
pip install fastapi uvicorn pyserial

# 4. 启动后端
cd UI/UI && py -3.12 main.py

# 5. 打开浏览器访问 http://127.0.0.1:8000
#    在界面中选择 mainTerm 对应的串口，点击"连接"
```

### 预期行为

1. sender 上电后先发送 JOIN_REQ 入网
2. 入网成功后每 10s 发起一次通信 (RREQ → RREP → DATA → ACK → ACK_CONFIRM)
3. 所有节点每 10s 发送一次心跳
4. Web 界面实时显示拓扑、RSSI、消息流

### 硬件测试

`Project/hardware_test/hardware_test.ino` 提供基础的 LoRa 收发测试，用于验证单个节点的硬件是否正常工作。

---

## 文件结构

```
Project_v3/
├── README.md                          ← 本文件
├── Project/
│   ├── sender/sender.ino              ← 终端节点固件 (DHT22 + 传感器采集)
│   ├── relayTerm/relayTerm.ino        ← 中继节点固件 (RSSI 盖章 + 非阻塞转发)
│   ├── mainTerm/mainTerm.ino          ← 主控节点固件 (路由裁决 + 串口桥)
│   └── hardware_test/hardware_test.ino ← 硬件基础测试
├── UI/UI/
│   ├── main.py                        ← FastAPI 后端 (帧解析 + WebSocket + API)
│   ├── index.html                     ← Web 前端 (ECharts 拓扑 + RSSI 趋势 + 消息流)
│   ├── echarts.min.js                 ← ECharts 5.5.0 本地化
│   └── tailwind.min.js                ← Tailwind CSS (Play CDN 本地化)
├── Docs/
│   ├── design_proposal.tex            ← 详细设计文档 (LaTeX)
│   ├── diagrams/                      ← 架构/流程/帧结构 Mermaid 图
│   └── *.jpg                          ← 设计文档插图
├── log/                               ← 会话日志 (session-YYYYMMDD-HHMMSS.txt)
└── .claude/                           ← Claude Code 配置
```

---

## 关键设计点

### 协议设计

| 设计点 | 说明 |
|--------|------|
| **瓶颈 RSSI 选路** | 木桶原理——路径质量取决于最弱一跳，选最弱中最强者。瓶颈相等时跳数少优先 |
| **ACK_CONFIRM 高亮** | 完整通信确认 (s→m→s→m) 后才高亮路由。RREP/DATA/ACK 阶段不触发高亮，避免误导用户 |
| **RSSI 缓存保护** | 非 RREQ 阶段优先使用已缓存的 RSSI，防止 DATA/ACK 阶段的瞬时 RSSI 波动覆盖路由表 |
| **HB RSSI 不覆盖** | 心跳偷听到的弱信号不污染 RREQ 已刷新的链路质量数据 |
| **RREQ 收集窗口** | mainTerm 等待 500ms 收集多路径 RREQ，容忍中继转发延迟 |
| **逐链路离线追踪** | 容忍偶发丢包，连续失败达阈值才判定不可达——避免单个包丢失导致路由震荡 |
| **DATA 去重** | mainTerm 对同一 (srcId, pathId, temp, hum) 2 秒内不重复处理 |

### 工程实现

| 设计点 | 说明 |
|--------|------|
| **非阻塞中继** | Pending 队列 (4 槽位) + 逐包退避，发送期间持续收包——避免阻塞导致丢包 |
| **DHT 手动时序** | 零依赖读取 DHT22，读取期间关中断防 LoRa 干扰。Arduino Uno 仅 2KB RAM，省掉 DHT 库 |
| **XOR 校验和** | 极简校验，覆盖全部可变字段 (13B)。帧头不参与——固定的 0x4C6F 无校验价值 |
| **紧凑帧设计** | 固定 16 字节，`#pragma pack(1)` 消除对齐 padding。`data[8]` 复用机制使同一结构支持 16 种消息类型 |
| **踢出七层隔离** | JOIN / 帧级检测 / 白名单 / RREQ 印章 / 心跳 / 中继 ID / 链路追踪——全面拦截被踢节点 |
| **两阶段信道切换** | PREPARE→READY→COMMIT，三重试 + 15s 心跳验证——确保全网原子切换 |
| **前端本地化** | ECharts + Tailwind 本地部署，不依赖 CDN 外网——断网环境可用 |

### 可靠性

| 设计点 | 说明 |
|--------|------|
| **ACK 三连发** | mainTerm 间隔 800ms 连发 3 份 ACK——LoRa 丢包率 ~5-10% 下单份可能丢失 |
| **RREQ 递增退避** | 180/300/420ms 三档递增，两次碰撞后大概率错开 |
| **缺失中继重试** | sender 记住上轮通信的中继列表，若本轮缺失则额外重试 2 次 |
| **路由快速老化** | 10s 老化——每轮通信后主动清除，避免陈旧路由导致盲发 |
| **串口断连恢复** | Python 后端自动检测串口断连，前端显示断开状态，重连后恢复 |
