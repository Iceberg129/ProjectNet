# LoRa AODV-Lite 多跳通信系统

基于 Arduino Uno + LoRa SX1278 的轻量按需距离矢量路由（AODV-Lite）多跳通信实验系统，配套 FastAPI + WebSocket + ECharts 实时可视化监控界面。

---

## 系统架构

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌──────────────┐
│  sender     │     │  relayTerm  │     │  mainTerm   │     │  PC (Python  │
│  0x30       │     │  0x21/0x22  │     │  0x10       │     │  + Browser)  │
│  Uno+LoRa   │     │  Uno+LoRa   │     │  Uno+LoRa   │     │  Web UI      │
└──────┬──────┘     └──────┬──────┘     └──────┬──────┘     └──────┬───────┘
       │                   │                   │                   │
       │    LoRa 530MHz    │    LoRa 530MHz    │    USB Serial      │
       │◄────────────────►│◄────────────────►│◄───────────────────►│
       │     SF7 17dBm     │     SF7 17dBm     │     9600 baud      │
       │                   │                   │   Binary frames    │
       │                   │                   │   + Text logs      │
```

## 硬件要求

| 组件 | 数量 | 说明 |
|---|---|---|
| Arduino Uno | ≥3 | sender + relay + mainTerm |
| LoRa SX1278 模块 | ≥3 | SPI 接口, 频段需支持 530 MHz |
| USB 数据线 | ≥1 | mainTerm ↔ PC 串口连接 |
| PC | 1 | 运行 Python Web 服务器 + 浏览器 |

**通信参数：**
- 频率：530 MHz
- 扩频因子：SF7
- 发射功率：17 dBm（最大）
- 空中速率：~21.9 kbps（SF7 下最快）

## 可能的网络拓扑

```
sender(0x30) ──────────────────────────→ mainTerm(0x10)    直达
sender(0x30) ──→ relay(0x21) ──────────→ mainTerm(0x10)    单跳中继
sender(0x30) ──→ relay(0x22) ──────────→ mainTerm(0x10)    单跳中继
sender(0x30) ──→ relay(0x21) ──→ relay(0x22) ──→ mainTerm  双跳中继
sender(0x30) ──→ relay(0x22) ──→ relay(0x21) ──→ mainTerm  双跳中继
```

| 角色 | 节点 ID | 固件文件 | 说明 |
|---|---|---|---|
| sender | `0x30` | `sender/sender.ino` | 发送终端，每 10s 发起一轮 AODV-Lite 传输 |
| relay | `0x21`, `0x22`, … | `relayTerm/relayTerm.ino` | 中继节点，RSSI 盖章 + 非阻塞转发 |
| mainTerm | `0x10` | `mainTerm/mainTerm.ino` | 目的终端，RREQ 收集 + 瓶颈 RSSI 裁决 + 串口桥接 PC |

> **烧录 relay 时**：修改 `MY_NODE_ID` 以区分不同中继节点（如 `0x21`、`0x22`）。

---

## 统一帧格式（16 字节）

```
Offset:  0       2        3        4        5        6        7        8              15
       ┌────────┬────────┬────────┬────────┬────────┬────────┬──────────────────┬────────┐
       │ head   │ srcId  │ destId │msgType │ pathId │ count  │ data[8]          │chksum  │
       │ 2B     │ 1B     │ 1B     │ 1B     │ 1B     │ 1B     │ 8B               │ 1B     │
       └────────┴────────┴────────┴────────┴────────┴────────┴──────────────────┴────────┘
```

| 字段 | 偏移 | 长度 | 说明 |
|---|---|---|---|
| `head` | 0 | 2 | 魔术字 `0x4C 0x6F`（"Lo"） |
| `srcId` | 2 | 1 | 原始发送者 ID |
| `destId` | 3 | 1 | 最终目标 ID |
| `msgType` | 4 | 1 | 消息类型（见下表） |
| `pathId` | 5 | 1 | 路由发现序号（每轮递增） |
| `count` | 6 | 1 | RREQ=印章对数, RREP/DATA/ACK=中继 ID 数 |
| `data` | 7 | 8 | RREQ=`{id,rssi}` 对（≤4 对）; RREP/DATA/ACK=中继 ID 列表 + 载荷 |
| `checksum` | 15 | 1 | 异或校验和（覆盖 srcId~data 共 13 字节） |

**`data[7]` 特殊用途**：当 `count < 4` 时，mainTerm 在转发给 PC 的二进制帧中，用 `data[7]` 嵌入末跳 RSSI 值（int8），供 Python UI 标注链路信号强度。

### 消息类型

| 常量 | 值 | 方向 | 说明 |
|---|---|---|---|
| `MSG_RREQ` | `0x10` | sender → 广播 | 路由发现请求 |
| `MSG_RREP` | `0x11` | mainTerm → sender | 路由发现应答（沿选中路径回传） |
| `MSG_DATA` | `0x01` | sender → mainTerm | 用户数据 |
| `MSG_ACK` | `0x02` | mainTerm → sender | 数据确认（最多 3 次重传） |
| `MSG_ACK_CONFIRM` | `0x06` | sender → mainTerm | ACK 确认回执 |

---

## 完整通信流程

```
sender(0x30)          relay(0x21)           relay(0x22)           mainTerm(0x10)       PC Web UI
    |                     |                     |                     |                    |
    |──RREQ(count=0)─────>|                     |                     |                    |
    |   广播                |──RREQ(stamp:21)───>|                     | [收集 500ms]       |
    |                     |   (退避90ms)         |──RREQ(stamp:21,22)>| [瓶颈 RSSI 裁决]    |
    |                     |                     |   (退避180ms)       | [选路: best vs dir] |
    |                     |                     |                     |                    |
    |                     |<──RREP(path=21,22)──|                     |                    |
    |                     |   (退避10ms转发)      |<──RREP(path=21,22)──|                    |
    |<──RREP(path=21,22)──|                     |   (退避70ms转发)      |                    |
    |                     |                     |                     |                    |
    | [等待 80~140ms 确保所有中继完成 RREP 转发]  |                     |                    |
    |                     |                     |                     |                    |
    |──DATA(path=21,22)──>|                     |                     |                    |
    |                     |──DATA(path=21,22)──>|                     |                    |
    |                     |   (退避10ms)          |──DATA(path=21,22)─>| [延迟 110~140ms]   |
    |                     |                     |   (退避70ms)         | [等中继转发完成]    |
    |                     |                     |                     |<──ACK──────────────|
    |                     |<──ACK───────────────|                     |   (最多3次重传)     |
    |                     |   (退避10ms)          |<──ACK──────────────|                    |
    |<──ACK───────────────|                     |   (退避70ms)         |                    |
    |                     |                     |                     |                    |
    | [等待 80~120ms 确保所有中继完成 ACK 转发]    |                     |                    |
    |                     |                     |                     |                    |
    |──ACK_CONFIRM───────>|                     |                     |                    |
    |                     |──ACK_CONFIRM───────>|                     |                    |
    |                     |   (退避10ms)          |──ACK_CONFIRM──────>| ✅                 |
    |                     |                     |   (退避70ms)         |                    |
    |                     |                     |                     |                    |
    | [ACK 成功 → 清空路由，10s 后重新探测]      |                     |                    |
    |                     |                     |                     |                    |
    |  ── RREQ 撞包重试机制 ──                    |                     |                    |
    |  RREP 超时(2s) ──→ backoff 180ms ──→ 重发 RREQ (第1次)           |                    |
    |  RREP 超时(2s) ──→ backoff 300ms ──→ 重发 RREQ (第2次)           |                    |
    |  RREP 超时(2s) ──→ backoff 420ms ──→ 重发 RREQ (第3次)           |                    |
    |  连续3次失败 → 路由不可达，等下一周期         |                    |
```

### 串口桥接数据流（mainTerm → PC → Web UI）

```
mainTerm.ino                     Python main.py                   Browser index.html
    │                                │                                │
    │ Serial.write(fwd, 16)          │                                │
    │ (data[7]=末跳RSSI)             │                                │
    │ ──────────────────────────────>│                                │
    │                                │ struct.unpack → frame_msg      │
    │                                │ extract stamps/relays/lastHop  │
    │                                │ WebSocket JSON ───────────────>│
    │                                │                                │ updateTopology()
    │                                │                                │ setLink() → ECharts
    │                                │                                │ RSSI labels on edges
```

---

## 节点详解

### sender（发送终端 `0x30`）

**文件**：`sender/sender.ino`

**状态机**：`IDLE → WAITING_RREP → WAITING_ACK → IDLE`

| 状态 | 说明 |
|---|---|
| `IDLE` | 空闲，10s 间隔后检查路由有效性 |
| `WAITING_RREP` | 已发送 RREQ，等待 RREP（超时 2s）— **支持最多 3 次递增退避重试** |
| `WAITING_ACK` | 已发送 DATA，等待 ACK 确认（超时 3.5s） |

**行为**：
- 每 10 秒发起一轮传输
- 路由失效/过期（>10s）或首次启动时发送 RREQ 广播
- **★ RREQ 撞包重试**：RREP 超时后不是直接放弃，而是递增退避重发 RREQ（最多 3 次）：
  - 第 1 次重试：退避 180ms 后重发（避开撞包窗口）
  - 第 2 次重试：退避 300ms 后重发
  - 第 3 次重试：退避 420ms 后重发
  - 连续 3 次同一路径 RSSI 均未到达 → 视作不可达，路由失效，等下一周期
  - 任意一次成功收到 RREP → 立即重置重试计数
- **★ 缺失中继重试**：收到 RREP 后发现上一轮已知的中继不在路径中 → 在当前周期内重发 RREQ（最多 2 次，50ms/80ms 快速退避）：
  - 第 1 次重试后仍缺 → 再次重发
  - 第 2 次重试后仍缺 → 接受当前路径，发送 DATA
  - 通信成功后更新已知中继列表（`knownRelays`）
  - 同一个周期内最多 3 次 RREQ 尝试（初始 + 2 次重试）
- 收到 RREP 后**动态等待** `20 + relayCount × 60 ms`（1跳→80ms, 2跳→140ms），确保所有中继完成 RREP 转发再发 DATA
- DATA 载荷 = sampleCounter + 0xAB
- 收到 ACK 后**动态等待** `40 + relayCount × 40 ms`（1跳→80ms, 2跳→120ms），确保所有中继完成 ACK 转发再发 ACK_CONFIRM
- 等待期间使用非阻塞 `while + checkForPacket()` 持续收包，防止丢失重传帧
- 收到 ACK 后**强制清空路由**，下一轮重新探测（确保路径实时准确）
- RREP 超时（2s）→ 递增退避重试（最多 3 次），耗尽后回 IDLE 并标记路由无效；ACK 超时（3.5s）→ 回 IDLE 并标记路由无效
- **★ 防死循环**：已决定发 DATA（`dataPending=false`）后，等待期间的延迟 RREP 副本不再触发重试

### relayTerm（中继节点 `0x21` / `0x22`）

**文件**：`relayTerm/relayTerm.ino`

**核心特性**：非阻塞转发 + 退避防碰撞 + 转发去重

#### RREQ 处理流程

1. **防环检查**：已盖章（本节点 ID 在 data 中）或印章满（≥4）→ 丢弃
2. **RSSI 盖章**：在 `data[]` 尾部追加 `{MY_ID, int8_rssi}`，`count++`
3. **计算退避**：
   - 单跳（count≤1）：`backoff = (ID末4位) × 80 + 10 ms`
   - 多跳（count>1）：`backoff = (ID末4位) × 40 + 100 ms`
4. **入队**：帧放入 pending 队列，`servicePending()` 异步定时发送
5. **发送**：每帧只发 1 份（多副本会导致盲区过长，阻断多跳链）

#### RREP / DATA / ACK / ACK_CONFIRM 处理

- 检查本节点是否在 `data[]` 路径列表中，记录位置 `myPos`
- **在路径中** → 位置退避 `myPos * 60 + 10 ms` 后非阻塞入队转发
  - `myPos=0`（靠近 sender）→ 10ms 退避
  - `myPos=1`（靠近 mainTerm）→ 70ms 退避
  - 不同位置的中继错开发送，避免空中碰撞
- **不在路径中** → 忽略
- **去重保护**：4 条环形缓存记录已转发帧的 `(srcId, destId, pathId, msgType)`，400ms 过期

#### Pending 队列

```
MAX_PENDING = 4    ← 最多同时 4 个待转发帧（RREQ + RREP + DATA + ACK）
```

队列满时丢弃新的 RREQ，避免内存溢出。

### mainTerm（目的终端 `0x10`）

**文件**：`mainTerm/mainTerm.ino`

**双重角色**：LoRa 目的节点 + USB 串口桥接 PC UI

#### RREQ 收集与裁决

1. **窗口收集**：每个新 `pathId` 启动 500ms 收集窗口
2. **去重合并**：相同 `(srcId, pathId, stampCount, relay IDs)` 保留瓶颈 RSSI 最好的版本；不同中继 ID 视为独立候选
3. **路径裁决**（`evaluateAndReply`）：
   - 分离直达（count=0）和中继（count>0）候选
   - 各选瓶颈 RSSI 最大的
   - **瓶颈 RSSI** = min(所有跳的 RSSI)，木桶原理
   - `bestRelayedRssi >= bestDirectRssi` → 选中继路径，否则选直达
4. **发送 RREP**：沿选中路径回传

#### ACK 确认与 DATA 去重

- **ACK 延迟发送**：收到 DATA 后若路径含中继，延迟 `80 + count × 30 ms` 再发 ACK，确保中继完成 DATA 转发并回到 RX 模式
- **ACK 超时重传**：最多 3 次，每次等待 800ms 收 ACK_CONFIRM
- **DATA 去重**：同一 `(srcId, pathId, payload)` 在 2s 内不重复处理，防止直连+转发双收浪费 ACK 周期

#### PC UI 串口桥接

mainTerm 通过 `Serial.write()` 将以下帧以二进制（16 字节）转发给 Python：

| 帧类型 | 触发位置 | data[7] 末跳 RSSI | 用途 |
|---|---|---|---|
| 收到的 RREQ | `handlePacket()` | ✅ 塞入 LoRa RSSI | 拓扑图 + RSSI 印章 |
| 收到的 DATA | `handlePacket()` | ✅ 塞入 LoRa RSSI | 链路信号显示 |
| 发出的 RREP | `sendRREP()` | ❌ 0x00 | 通信阶段指示器 |
| 发出的 ACK | `sendACK()` | ❌ 0x00 | 通信阶段指示器 |

#### 串口命令

| 命令 | 说明 |
|---|---|
| `j` | 切换干扰模式（连续发包霸占信道，测试抗干扰能力） |
| `s` | 打印收包统计（总数 / 长度错误 / 魔术字错误 / 校验错误 / 合法 RREQ / 合法 DATA） |

#### 统计字段

```cpp
stat_totalHeard   // 总收到包数
stat_badSize      // 长度不匹配
stat_badMagic     // 魔术字错误
stat_badDest      // 非本节点地址
stat_badChecksum  // 校验和错误
stat_goodRREQ     // 合法 RREQ
stat_goodDATA     // 合法 DATA + Beacon
```

---

## Python Web 后端

**文件**：`UI/UI/main.py`

**启动**：`py -3.12 main.py`（监听 `127.0.0.1:8000`）

### 核心组件

| 组件 | 说明 |
|---|---|
| `serial_reader_thread` | 独立线程读取串口，分离二进制帧和文本行 |
| `StatsTracker` | 全局统计追踪器（帧计数、节点统计、**逐链路 RSSI 失败追踪**、通信阶段、**成功率**） |
| `ConnectionManager` | WebSocket 连接池管理 + 广播 |
| `periodic_stats` | 每 3 秒推送全量统计 + **全阶段超时检测**（RREQ 6s / RREP 5s / DATA 5s / ACK 4s）+ 待处理事件广播 |

### 二进制帧解析

```python
FRAME_FORMAT = '<2sBBBBB8sB'   # 16 字节
#  head:2B, srcId:B, destId:B, msgType:B, pathId:B, count:B, data:8B, checksum:B
```

**RSSI 提取逻辑**：

| 帧类型 | stamps 提取 | relays 提取 | lastHopRssi 提取 |
|---|---|---|---|
| RREQ | 从 data 中逐对解析 `{id, rssi}` | — | `data[7]` 当 `count < 4` |
| RREP | — | 从 data 提取中继 ID 列表 | ❌（mainTerm 生成，data[7]=0） |
| DATA | — | 从 data 提取中继 ID 列表 | `data[7]` 当 `count < 4` |
| ACK | — | 从 data 提取中继 ID 列表 | ❌（mainTerm 生成，data[7]=0） |

### API 端点

| 端点 | 方法 | 说明 |
|---|---|---|
| `/` | GET | 返回 `index.html` |
| `/ws` | WebSocket | 实时数据通道 |
| `/api/ports` | GET | 列出可用串口 |
| `/api/stats` | GET | 获取全量统计 JSON |
| `/api/reset_stats` | GET | 重置统计 |
| `/api/nodes` | GET | 获取所有节点详情 |
| `/api/node/{id}` | GET | 获取指定节点详情 |
| `/api/links` | GET | 获取链路 RSSI 数据及历史 |
| `/api/simulate` | GET | 运行 8 轮模拟通信（含 RSSI 漂移） |

### WebSocket 消息类型

| type | 方向 | 说明 |
|---|---|---|
| `stats` | server→client | 全量统计数据（每 3s 推送） |
| `frame` | server→client | 解析后的 LoRa 帧（含 stamps/relays/lastHopRssi） |
| `phase_timeout` | server→client | **阶段超时告警**（RREQ/RREP/DATA/ACK 任一阶段卡住过久） |
| `route_fail` | server→client | 路由失效通知（ACK 重传耗尽或阶段超时） |
| `link_retry` | server→client | **链路 RSSI 未刷新预警**（RREP 时发现缺失，sender 正在重试） |
| `link_broken` | server→client | **链路不可达判定**（ACK_CONFIRM 时 sender 多次重试仍未发现） |
| `link_restored` | server→client | **链路 RSSI 恢复**（之前 broken 的链路重获 RSSI 刷新） |
| `node_offline` | server→client | **节点离线判定**（所有已知链路均达失败阈值） |
| `sys` | server→client | 系统消息（串口连接/断开） |
| `sys_error` | server→client | 系统错误 |
| `text_log` | server→client | 串口文本行（调试日志，含 sender 重试信息） |
| `open_serial` | client→server | 打开指定串口 |
| `close_serial` | client→server | 关闭串口 |
| `reset_stats` | client→server | 重置统计 |
| `send_command` | client→server | 向 mainTerm 发送命令 |

---

## Web 前端界面

**文件**：`UI/UI/index.html`

**技术栈**：Tailwind CSS + ECharts 5.5 + 原生 JavaScript

### 页面布局

```
┌──────────────────────────────────────────────────────────┐
│  HEADER: 标题 + 串口选择 + 波特率 + 模拟/演示/信道按钮     │
├──────────────────────────────────────────────────────────┤
│  STATS BAR: 总帧数 | RREQ | RREP | DATA | ACK | 成功率  │
├────────────────────────────────┬─────────────────────────┤
│                                │  Tab: 📜日志 🔍节点 📶链路│
│   ECharts 拓扑图               │  ┌─────────────────────┐ │
│   (力导向布局)                 │  │ 日志流 / 节点详情    │ │
│                                │  │ 链路统计            │ │
│   ┌─ 图例 (左上)               │  └─────────────────────┘ │
│   ├─ 通信阶段指示器 (右上)      │                          │
│   └─ RSSI 标注 (边标签)        │                          │
├────────────────────────────────┴─────────────────────────┤
│  RSSI 趋势图 (折线图, 可收起)                             │
└──────────────────────────────────────────────────────────┘
```

### 核心功能

#### 拓扑图

- **力导向布局**：节点可拖拽、缩放、平移，固定坐标消除抖动
- **节点分类**：主节点（橙）、中继（青）、终端（紫）、PC 网关（蓝）
- **链路样式**：闲置链路为灰色虚线，USB 有线连接为蓝色实线，broken 链路为红色点线低透明度
- **路由节点高亮**：仅当前路由上的节点发光（shadowBlur），未使用的节点不发光，PC 网关始终不受影响
- **RSSI 颜色编码**：🟢 > -40 dBm | 🟡 -40~-50 | 🟠 -50~-60 | 🔴 ≤ -70
- **RSSI 边标签**：直接在拓扑图链路上显示 dBm 数值（broken 链路隐藏）
- **路由高亮**：**仅在 ACK_CONFIRM 确认后（完整通信流程跑通）**高亮成功路径，链路发光加粗并随 RSSI 着色；RREP/DATA/ACK 阶段不提前高亮；新一轮 RREQ 发起时清除旧高亮
- **tooltip 悬停**：鼠标移至节点显示设备详情，移至链路显示 RSSI + 最后更新时间 + broken 状态
- **节点点击**：自动跳转节点详情面板
- **链路点击**：在拓扑图中点击链路 → 底部弹出链路详情条（RSSI + 更新时间），同步更新 RSSI 趋势图选择

#### RSSI 数据流

```
LoRa RSSI  → mainTerm data[7] → Python lastHopRssi → WebSocket → setLink()
                                                                    │
                                         RREQ stamps → linkRssiCache │
                                                                    ▼
                                              ECharts edge label: "-45 dBm"
```

**关键设计**：
- RSSI 值从不过期覆盖：`setLink()` 仅在收到非 null 值时更新，保护已有正确值
- **缓存优先策略**：RREP/DATA/ACK 阶段所有跳（含末尾跳）均优先使用 `linkRssiCache`，避免 DATA 帧的瞬时 RSSI（如 -63）覆盖组网阶段的发现 RSSI（如 -33）
- 缓存键归一化：`lo-hex < hi-hex` 确保 `0x21-0x30` 和 `0x30-0x21` 命中同一缓存
- RREP/ACK 不提取 `lastHopRssi`（mainTerm 发出的帧 `data[7]=0x00`），仅 RREQ/DATA 提取

#### 通信阶段指示器

```
 ① RREQ  ──→  ② RREP  ──→  ③ DATA  ──→  ④ ACK
(橙点亮)     (青点亮)     (绿点亮)     (黄点亮)
```

- 当前阶段高亮 + 脉冲动画
- 已完成阶段显示 ✓ + 对应颜色
- RREP 和 ACK 由 mainTerm 主动 `Serial.write` 触发（解决"永远卡在最后一步"问题）
- **★ 阶段超时告警**：后端 + 前端双重超时检测
  - 后端：每 3s 检查，RREQ>6s / RREP>5s / DATA>5s / ACK>4s → 广播 `phase_timeout`
  - 前端：每 2s 独立检查，RREQ>8s / RREP/DATA/ACK>6s → 自动触发失败 UI
  - 失败阶段显示 ⚠ 红色闪烁，之前阶段保持 ✓，之后阶段保持灰色
  - 支持 `rreq_fail` / `rrep_fail` / `data_fail` / `ack_fail` 全部失败状态

#### 日志流

- 按消息类型过滤：全部 / RREQ / RREP / DATA / ACK / 系统
- 显示帧详情：源→目标、类型、pathId、RSSI 印章、路径、data 十六进制
- 最多缓存 300 条，渲染最近 120 条

#### 节点详情

- 点击拓扑图中的节点 → 自动切换到节点 Tab
- 显示：HEX ID、DEC ID、角色、最后活跃时间
- 列出该节点所有相关链路及 RSSI

#### 链路统计

- 列出所有活跃链路（不含 PC-0x10）
- 显示：RSSI 值、线型（虚线/实线）、线宽
- 当前路由链路标记 ★

#### RSSI 趋势图

- 底部折线图，展示每条链路的 RSSI 历史（最近 120 秒）
- 可按需收起/展开
- 数据来源：Python StatsTracker 的 `rssi_history`（deque maxlen=120）

#### 附加功能

- **自动演示**：每 6s 触发一次模拟通信（8 轮含 RSSI 漂移）
- **信道配置面板**：下发频率/SF/功率到目标节点（演示模式）
- **节点垃圾回收**：25s 无活动 → 彻底删除（安全兜底）；半透明/离线仅由后端 `node_offline` 事件触发（所有链路均 broken 后）
- **链路垃圾回收**：30s 未更新且非 broken 状态 → 前端安全兜底删除；broken 链路由后端逐链路失败计数管理（连续 5 次失败后从 `_known_links` 移除）
- **跨中继链路保护**：`0x21-0x22` 等跨中继 RSSI 仅出现在双跳 RREQ 中，后端容忍 5 次缺失才清理，防止偶发丢包误删
- **WebSocket 自动重连**：断线 3s 后重试

---

## RSSI 数据完整传递链

```
relayTerm               mainTerm                  Python                    前端
   │                        │                        │                        │
   │──LoRa RREQ───────────>│                        │                        │
   │  data: [{0x21,-44}]   │                        │                        │
   │                        │──fwd[14]←末跳RSSI      │                        │
   │                        │  Serial.write(16B) ──>│                        │
   │                        │                        │──stamps: [{0x21,-44}]  │
   │                        │                        │  lastHopRssi: -55     │
   │                        │                        │  WS JSON ────────────>│
   │                        │                        │                        │──RREQ: setLink()
   │                        │                        │                        │  0x21-0x30: -44 ✓
   │                        │                        │                        │  0x10-0x21: -55 ✓
   │                        │                        │                        │
   │<──LoRa RREP───────────│                        │                        │
   │                        │──Serial.write(16B) ──>│                        │
   │                        │  (data[7]=0x00)       │                        │
   │                        │                        │──relays: [0x21]        │
   │                        │                        │  lastHopRssi: None    │
   │                        │                        │  WS JSON ────────────>│
   │                        │                        │                        │──RREP: setLink()
   │                        │                        │                        │  cache命中 -55 ✓
   │                        │                        │                        │
   │──LoRa DATA───────────>│                        │                        │
   │                        │──fwd[14]←末跳RSSI      │                        │
   │                        │  Serial.write(16B) ──>│                        │
   │                        │                        │──relays: [0x21]        │
   │                        │                        │  lastHopRssi: -52     │
   │                        │                        │  WS JSON ────────────>│
   │                        │                        │                        │──DATA: setLink()
   │                        │                        │                        │  ★linkRssiCache 命中
   │                        │                        │                        │  0x10-0x21: -55 ✓ (缓存保护)
```

---

## 关键设计决策

| 设计点 | 说明 |
|---|---|
| **瓶颈 RSSI 选路** | 选择路径中最弱一跳信号最强的路径（木桶原理），而非平均 RSSI |
| **RREQ 撞包重试** | sender RREP 超时后递增退避重发 RREQ（最多 3 次，180/300/420ms），连续失败才视作不可达 |
| **全阶段超时检测** | 后端 + 前端双重超时，RREQ/RREP/DATA/ACK 任一阶段卡住均触发失败告警，不等下一轮更新 |
| **ACK 成功才高亮** | 前端路由高亮仅在 ACK_CONFIRM 回执确认后（整个通信流程跑通）触发，不提前误导 |
| **位置退避防碰撞** | 所有帧转发按路径位置 `myPos * 60 + 10 ms` 退避，避免多中继同时发送空中碰撞 |
| **非阻塞中继** | relay 使用 pending 队列 + 退避，所有帧转发均为非阻塞，发送期间持续收包 |
| **转发去重** | 4 条环形缓存记录已转发帧的 `(src, dest, pathId, msgType)`，阻断乒乓循环 |
| **ACK 分级冗余** | mainTerm ACK 最多 3 次重传（每次 800ms 超时），sender 收到 ACK 后回 ACK_CONFIRM |
| **动态等待窗口** | sender DATA/ACK_CONFIRM 发送前等待时间按中继数动态计算（1跳 80ms, 2跳 120-140ms） |
| **DATA 去重** | mainTerm 对相同 `(srcId, pathId, payload)` 在 2s 内不重复 ACK，避免直连+转发双收 |
| **每轮重新发现** | sender 收到 ACK 后强制清空路由，确保路径表达当前电磁环境 |
| **二进制帧桥接** | mainTerm 通过 `Serial.write` 将 16 字节原始帧转发 PC，Python 解析为结构化 JSON |
| **末跳 RSSI 嵌入** | 当 `count < 4` 时利用 `data[7]` 传递末跳 RSSI（int8），不破坏原有帧结构 |
| **RSSI 缓存保护** | 所有非 RREQ 阶段优先使用 `linkRssiCache`，防止 DATA 帧瞬时 RSSI 覆盖组网发现值 |
| **缓存键归一化** | `lo-hex < hi-hex` 统一链路键格式，确保不同方向查同一缓存 |
| **逐链路失败追踪** | 后端以 `_known_links`（全量历史）为基准逐链路追踪 RSSI 刷新，容忍偶发丢失，连续失败达阈值才判定不可达 |
| **Sender 周期内重试** | sender 发现已知中继缺失后在当前周期内快速重试 RREQ（最多 2 次），3 次总尝试仍找不到才接受降级路径 |
| **ACK_CONFIRM 最终判定** | 链路缺失的最终判定在 ACK_CONFIRM 时（周期末），此时 sender 已耗尽重试、数据收集完整；RREP 时仅发早期预警 |
| **重试 RREQ 不重置收集器** | sender 重试 RREQ 时后端不重置 `_current_rreq_links`，累积所有重试中的 RREQ 印章，确保完整评估 |
| **链路缺失即重搜** | 若某链路 RSSI 未捕获，sender 主动重试 RREQ；同一周期内多次尝试 + 跨周期累计，确保"多次都没找到才不可达" |

---

## 串口输出示例

### relay 收包日志

```
[ HEAR] type=0x10 src=0x30 dst=0x10 cnt=0 RSSI=-45
  └─ RREQ queued  backoff=90ms rssi=-45dBm stamps=[]
  └─ RREQ SENT  stamps=[0x21:-45]

[ HEAR] type=0x11 src=0x10 dst=0x30 cnt=1 RSSI=-52
  └─ RREP FORWARDED  path=[0x21 ]
```

### mainTerm 路线图

```
── pathId=3 [2] ──
  0x30 -----[-55]-----> [C]                       b=-55
▶ 0x30 -----[-50]-----> 0x21 ----[-42]----> [C]   b=-50 ★  -50≥-55

RREP sent → 0x30  pathId=3  relays=1 (RELAYED)

<<< DATA from 0x30 via 0x21  RSSI=-48 dBm  payload=02AB
  ACK sent → 0x30 via 0x21
```

---

## 快速开始

### 1. 烧录 Arduino

| 节点 | 文件 | 修改项 |
|---|---|---|
| sender | `sender/sender.ino` | 无需修改 |
| relay #1 | `relayTerm/relayTerm.ino` | `MY_NODE_ID = 0x21` |
| relay #2 | `relayTerm/relayTerm.ino` | `MY_NODE_ID = 0x22` |
| mainTerm | `mainTerm/mainTerm.ino` | 无需修改 |

### 2. 启动 Python 服务

```bash
cd UI/UI
py -3.12 main.py
```

服务监听 `http://127.0.0.1:8000`

### 3. 打开浏览器

访问 `http://127.0.0.1:8000`，选择 mainTerm 的串口（如 COM6），波特率 9600，点击"打开串口"。

### 4. 上电节点

全部上电后，sender 自动每 10s 发起一轮通信。拓扑图、统计、RSSI 趋势图实时更新。

### 5. 无硬件测试

点击"🧪 模拟"按钮，或开启"▶ 自动演示"模式，使用内置模拟数据观察完整通信流程。

---

## 故障排查

| 现象 | 可能原因 | 解决方案 |
|---|---|---|
| 网页不更新 | Arduino 未烧录最新固件 | 重新烧录 mainTerm（含 Serial.write 改动） |
| 串口输出乱码 | 二进制帧被串口监视器当文本解析 | 关闭 Arduino IDE 串口监视器 |
| COM 口 Code 31 | Windows COM Name Arbiter 冲突 | 管理员运行 `fix_com6.ps1`（清理注册表） |
| Port 8000 被占用 | 残留 Python 进程 | `Get-NetTCPConnection -LocalPort 8000 \| Stop-Process` |
| RSSI 悬停显示"未知" | ECharts link data 缺少 rssi 字段 | 已修复：`makeTopoOption()` 中 link data 补充 `rssi` 属性 |
| RSSI 组网后骤变 | DATA 帧 lastHopRssi 覆盖了组网 RSSI | 已修复：非 RREQ 阶段末尾跳优先使用 `linkRssiCache` |
| RSSI 显示"0 dBm" | RREP/ACK 的 data[7]=0 覆盖了正确值 | 已修复：Python 仅对 RREQ/DATA 提取 lastHopRssi |
| 通信阶段卡在最后一步 | mainTerm 未转发 RREP/ACK 到串口 | 已修复：sendRREP/sendACK 中加了 Serial.write |
| 阶段指示器卡住不动 | 某阶段超时未检测到 | 已修复：后端 + 前端双重阶段超时检测，超时后显示 ⚠ 告警 |
| 撞包导致反复组网失败 | 多中继同时转发 RREQ 碰撞 | 已修复：sender 递增退避重试（最多 3 次），避开撞包窗口 |
| 拓扑图不显示中继节点 | 中继未转发或超出老化时间 | 检查 relay 电源和天线，观察串口输出 |
| 节点变半透明离线 | 后端检测到所有链路均不可达（`node_offline`） | 节点恢复数据后自动转为在线 + "重新上线"日志 |
| 两节点间无连线 | 跨中继链路 RSSI 仅出现在双跳 RREQ 中，偶发丢包可能导致该链路未被捕获 | sender 会自动重试 RREQ；后端容忍 5 次缺失才清理 |
| 链路显示红色点线 | 后端判定链路不可达（`link_broken`） | sender 重试后若链路 RSSI 恢复，自动 `link_restored` |
| sender 卡在 RREQ↔RREP 循环 | 已决定发 DATA 后等待期间收到延迟 RREP 副本触发新重试 | 已修复：`dataPending` 为 false 时不再触发缺失中继重试 |
| 路由高亮与实际不一致 | RREP 阶段就提前高亮 | 已修复：仅 ACK_CONFIRM 确认后才高亮成功路径 |

---

## 文件清单

```
Project_v3/
├── README.md                    ← 本文件
├── Project/
│   ├── sender/
│   │   └── sender.ino           ← 发送终端固件（0x30）
│   ├── relayTerm/
│   │   └── relayTerm.ino        ← 中继节点固件（0x21 / 0x22）
│   └── mainTerm/
│       └── mainTerm.ino         ← 目的终端固件（0x10）+ PC 桥接
└── UI/
    └── UI/
        ├── main.py              ← FastAPI 后端（串口解析 + WebSocket）
        └── index.html           ← Web 前端（ECharts 拓扑 + RSSI 趋势）
```

---

## 更新日志

### 2026-06-30 — 智能组网重试与逐链路离线检测

**sender.ino**
- 新增已知中继记忆机制：通信成功后记录 `knownRelays`，下轮若缺失则周期内快速重试 RREQ（最多 2 次，50ms/80ms 退避）
- 防死循环：已决定发 DATA（`dataPending=false`）后等待期间的延迟 RREP 不再触发重试

**main.py（后端）**
- 新增逐链路 RSSI 失败追踪：`_link_fail_count` + `_known_links`（全量历史基准）+ `_current_rreq_links`（周期收集）
- 链路检测两阶段：RREP 时预警（`link_retry`，不递增计数器）→ ACK_CONFIRM 时最终判定（`link_broken` + `node_offline`）
- sender 重试 RREQ 不重置收集器，累积所有重试数据
- 新增事件类型：`link_retry`、`link_broken`、`link_restored`、`node_offline`
- 最终跳链路写入 `_known_links`，RREP relay 路径链路写入 `_current_rreq_links`
- 容忍跨中继链路偶发丢包：连续 5 次失败才从 `_known_links` 清理

**index.html（前端）**
- 拓扑图路由节点高亮：仅当前路由上的节点发光，未用到的中继不发光，PC 始终正常
- Broken 链路显示：红色点线低透明度，tooltip 显示不可达警告 + 最后更新时间
- 链路详情条：点击拓扑图中的边或链路卡片 → 底部弹出详情
- 节点半透明仅由后端 `node_offline` 触发，移除前端 12s 自动老化
- 链路 GC：broken 链路由后端管理，前端不主动删除
- 成功率统计卡片 + 卡片逐级延迟动画
- RSSI 趋势图数据时效显示
