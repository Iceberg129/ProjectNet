# LoRa AODV-Lite 多跳通信系统

> **第 21 队** · 黄鹏峻 · 吴汉鹏 · 林金鑫 · 贾笛  
> GitHub: [050415/ProjectNet](https://github.com/050415/ProjectNet)

基于 Arduino Uno + LoRa SX1278 的轻量按需距离矢量路由（AODV-Lite）系统，配套 FastAPI + WebSocket + ECharts 实时监控界面。

---

## 架构

```
sender(0x30) ── LoRa ── relay(0x21) ── LoRa ── relay(0x22) ── LoRa ── mainTerm(0x10) ── USB ── PC(Python+Web)
```

| 角色 | ID | 固件 | 说明 |
|------|----|------|------|
| sender | `0x30` | `sender/sender.ino` | 每 10s 发起通信，含 DHT22 温湿度传感器 |
| relay | `0x21/0x22` | `relayTerm/relayTerm.ino` | RSSI 盖章 + 非阻塞退避转发 |
| mainTerm | `0x10` | `mainTerm/mainTerm.ino` | 瓶颈 RSSI 裁决 + 串口桥接 PC |

**LoRa 参数：** 530MHz / SF7 / 17dBm

---

## 通信流程

```
sender → RREQ(广播) → relay(盖章) → mainTerm(收集500ms+瓶颈裁决) → RREP → DATA → ACK → ACK_CONFIRM
```

- **RREQ 撞包重试**：sender 最多 3 次递增退避（180/300/420ms）
- **瓶颈选路**：选 min(各跳 RSSI) 最大的路径；瓶颈相等时跳数少优先
- **退避防碰撞**：中继按路径位置 `myPos×60+10ms` 错开发送

---

## 帧格式（16 字节）

| 偏移 | 0 | 2 | 3 | 4 | 5 | 6 | 7 | 15 |
|------|---|---|---|---|---|---|---|---|
| 字段 | head `4C 6F` | srcId | destId | msgType | pathId | count | data[8] | checksum |

- RREQ: `data` = `{relayId, rssi}` × 4 对
- RREP/DATA/ACK: `data` = 中继 ID 列表
- `data[7]`: 当 count<4 时嵌入末跳 RSSI

---

## Web 监控界面

**启动：** `cd UI/UI && py -3.12 main.py` → `http://127.0.0.1:8000`

| 功能 | 说明 |
|------|------|
| 拓扑图 | ECharts 力导向，实时节点/链路/RSSI 标注，路由高亮 |
| RSSI 趋势图 | 自定义 SVG，120 秒窗口，逐段渐变色，离线自动断线 |
| 消息流网格 | 40 格动画填充，悬停弹窗 + 点击跳转日志 |
| 信道配置 | 单节点功率调整 + 全网两阶段提交信道切换 |
| 白名单管理 | 串口 WL? 协议，在线添加/踢出/恢复 |
| 会话日志 | `log/session-*.txt` 自动记录逐帧+事件+周期快照 |

---

## 快速开始

```bash
# 1. 烧录固件（修改 relay 的 MY_NODE_ID）
# 2. 安装依赖
pip install fastapi uvicorn pyserial
# 3. 启动后端
cd UI/UI && py -3.12 main.py
# 4. 浏览器打开 http://127.0.0.1:8000，选择串口连接
```

---

## 文件结构

```
Project_v3/
├── README.md
├── Project/
│   ├── sender/sender.ino
│   ├── relayTerm/relayTerm.ino
│   └── mainTerm/mainTerm.ino
├── UI/UI/
│   ├── main.py          ← FastAPI 后端
│   ├── index.html       ← Web 前端
│   ├── echarts.min.js   ← ECharts 本地化
│   └── tailwind.min.js  ← Tailwind 本地化
├── log/                 ← 会话日志
└── Docs/                ← 设计文档
```

---

## 关键设计点

- **瓶颈 RSSI 选路**：木桶原理，最弱一跳最强者胜
- **ACK_CONFIRM 高亮**：完整通信确认后才高亮路由，不提前误导
- **RSSI 缓存保护**：非 RREQ 阶段优先用缓存值，防瞬时波动覆盖
- **非阻塞中继**：pending 队列 + 退避，发送期间持续收包
- **逐链路离线追踪**：容忍偶发丢失，连续失败达阈值才判定不可达
- **踢出七层隔离**：JOIN/帧/白名单/RREQ/心跳/中继ID/链路追踪全面拦截
- **两阶段信道切换**：PREPARE→READY→COMMIT，非阻塞退避防碰撞
- **前端本地化**：CDN 资源本地部署，不依赖外网
- **HB RSSI 不覆盖**：心跳偷听弱信号不污染 RREQ 已刷新链路
