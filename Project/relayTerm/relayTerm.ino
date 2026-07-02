// =============================================
// relayTerm.ino – 中继节点（AODV-Lite RSSI 盖章 非阻塞版）
// 适用于 Arduino Uno + LoRa SX1278 
// 烧录时修改 MY_NODE_ID：B=0x21, D=0x22 ...
// =============================================
#include <SPI.h>
#include <LoRa.h>

// ---------- 节点身份（烧录时修改） ----------
const uint8_t MY_NODE_ID = 0x22;       // 中继节点 ID

// ---------- 协议常量 ----------
const uint8_t FRAME_HEADER_0 = 0x4C;
const uint8_t FRAME_HEADER_1 = 0x6F;
const uint8_t MSG_RREQ       = 0x10;   // 探路请求
const uint8_t MSG_RREP       = 0x11;   // 探路应答
const uint8_t MSG_DATA       = 0x01;   // 用户数据
const uint8_t MSG_ACK        = 0x02;   // 数据确认
const uint8_t MSG_ACK_CONFIRM = 0x06;   // ACK 确认回执
const uint8_t MSG_JOIN_REQ = 0x20;
const uint8_t MSG_JOIN_ACK = 0x21;
const uint8_t MSG_JOIN_REJ = 0x22;
const uint8_t MSG_CMD = 0x04;
const uint8_t MSG_CMD_ACK = 0x05;
const uint8_t MSG_CMD_PREPARE = 0x0A;
const uint8_t MSG_CMD_READY   = 0x0B;
const uint8_t MSG_CMD_COMMIT  = 0x0C;
const uint8_t BROADCAST_ID    = 0xFF;
const uint8_t MSG_HB       = 0x03;   // heartbeat
const uint8_t MSG_KICK     = 0x08;   // 踢出命令
const uint8_t MSG_UNKICK   = 0x09;   // 恢复命令
const uint8_t MAX_RELAYS     = 2;
const uint8_t MAX_STAMPS     = 4;      // 最多 4 对 {id, rssi} = 8 字节
const uint8_t FRAME_SIZE     = 16;

// ---------- 统一帧结构 ----------
#pragma pack(push, 1)
typedef struct {
  uint8_t head[2];
  uint8_t srcId;
  uint8_t destId;
  uint8_t msgType;
  uint8_t pathId;
  uint8_t count;            // RREQ=印章数, RREP/DATA=中继数
  uint8_t data[8];          // RREQ: {id,rssi}对; RREP/DATA: relay列表+payload
  uint8_t checksum;
} LoRaFrame;
#pragma pack(pop)

// ---------- 非阻塞转发队列 ----------
#define MAX_PENDING 4
struct PendingFwd {
  LoRaFrame frame;
  uint32_t  nextSend;
  int8_t    retries;
  bool      active;
};
PendingFwd pending[MAX_PENDING];
uint8_t pendingCount = 0;
uint32_t lastHbTime    = 0;
uint16_t hbInterval    = 10000;

// ---------- 全局接收缓冲 ----------
LoRaFrame rxFrame;

// ---------- 转发去重（防乒乓循环，2秒过期允许重传） ----------
#define FWD_HISTORY 4
#define FWD_EXPIRE_MS 400    // 去重 400ms 过期：拦截乒乓循环(~10ms)，放行重传(~530ms)
struct FwdRecord {
  uint8_t srcId, destId, pathId, msgType;
  uint32_t timestamp;
};
FwdRecord fwdHist[FWD_HISTORY];
uint8_t  fwdHistIdx = 0;
// Two-phase commit -- pending channel switch parameters
bool     pendingReady = false;
uint16_t pendingFreq  = 530;
uint8_t  pendingSf    = 7;
// ★ 非阻塞 READY 退避
bool     readyPending = false;
uint32_t readySendTime = 0;
uint8_t  readyStatusVal = 0;

bool wasForwarded(uint8_t srcId, uint8_t destId, uint8_t pathId, uint8_t msgType) {
  for (uint8_t i = 0; i < FWD_HISTORY; i++) {
    if (fwdHist[i].srcId == srcId &&
        fwdHist[i].destId == destId &&
        fwdHist[i].pathId == pathId &&
        fwdHist[i].msgType == msgType) {
      // 未过期 → 确实已转发，拦截
      if (millis() - fwdHist[i].timestamp < FWD_EXPIRE_MS) return true;
      // 已过期 → 继续检查其他槽位（可能有更新的记录）
    }
  }
  return false;
}

void markForwarded(uint8_t srcId, uint8_t destId, uint8_t pathId, uint8_t msgType) {
  fwdHist[fwdHistIdx].srcId     = srcId;
  fwdHist[fwdHistIdx].destId    = destId;
  fwdHist[fwdHistIdx].pathId    = pathId;
  fwdHist[fwdHistIdx].msgType   = msgType;
  fwdHist[fwdHistIdx].timestamp = millis();
  fwdHistIdx = (fwdHistIdx + 1) % FWD_HISTORY;
}

bool gKicked = false;  // ★ 踢出标志

// ★ 节点状态 + 邻居 RSSI（合并进 HB）
uint16_t txPacketCount = 0;
uint16_t rxPacketCount = 0;
uint8_t  lastHeardId   = 0;     // 最近监听到的邻居节点 ID
int8_t   lastHeardRssi = 0;     // 该邻居的 RSSI

int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// (sendNodeStatus removed — merged into sendHeartbeat)
// =============================================
void setup() {

  LoRa.setPins(A3);
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(530E6)) {
    Serial.println(F("LoRa init fail!"));
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setTxPower(17);

  memset(pending, 0, sizeof(pending));

  Serial.print(F("Relay 0x"));
  Serial.print(MY_NODE_ID, HEX);
  Serial.println(F(" OK  530MHz SF7"));
}

// =============================================
// 主循环
// =============================================
void loop() {
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck >= 5) {
    lastCheck = millis();
    handlePacket();  // ★ 即使 KICKED 也继续监听，等待 UNKICK
  }
  // ★ 非阻塞 READY 发送：定时器到期后发送，期间正常收包
  if (readyPending && millis() >= readySendTime) {
    readyPending = false;
    LoRaFrame ack;
    memset(&ack, 0, FRAME_SIZE);
    ack.head[0] = FRAME_HEADER_0;
    ack.head[1] = FRAME_HEADER_1;
    ack.srcId   = MY_NODE_ID;
    ack.destId  = 0x10;
    ack.msgType = MSG_CMD_READY;
    ack.data[0] = readyStatusVal;
    calcChecksum(&ack);
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&ack, FRAME_SIZE);
    LoRa.endPacket(); txPacketCount++;
    Serial.print(F("READY tx status="));
    Serial.println(readyStatusVal, DEC);
  }
  if (gKicked) { return; }  // 静默：不转发不发送（但仍监听 UNKICK，已在上方 handlePacket 处理）
  // ★ 节点状态已合并入 sendHeartbeat，不再单独发送 MSG_NODE_STATUS
  // Heartbeat: 10s +/- 2s jitter (carries uptime + freeRAM + txPkts + neighbor RSSI)
  if (millis() - lastHbTime >= hbInterval) {
    sendHeartbeat();
    lastHbTime = millis();
    hbInterval = 8000 + random(4000);
  }

  // ★ 非阻塞发送：检查所有 pending 转发
  servicePending();
}

// =============================================
// 非阻塞发送服务
// =============================================
void servicePending() {
  for (uint8_t p = 0; p < MAX_PENDING; p++) {
    if (!pending[p].active) continue;
    if (millis() < pending[p].nextSend) continue;  // 还没到时间

    // 发送一份
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&pending[p].frame, FRAME_SIZE);
    LoRa.endPacket(); txPacketCount++;

    pending[p].retries--;
    if (pending[p].retries > 0) {
      pending[p].nextSend = millis() + 30;  // ★ 拷贝间隔从10→30ms，给其他 relay 更多 RX 窗口
    } else {
      // 全部发完
      pending[p].active = false;
      pendingCount--;

      bool isRREQ = (pending[p].frame.msgType == MSG_RREQ);
      Serial.print(F("  fwd tx ["));
      for (uint8_t i = 0; i < pending[p].frame.count; i++) {
        Serial.print(F("0x"));
        Serial.print(pending[p].frame.data[isRREQ ? (i * 2) : i], HEX);
        if (isRREQ) {
          Serial.write(':');
          Serial.print((int8_t)pending[p].frame.data[i * 2 + 1], DEC);
        }
        if (i < pending[p].frame.count-1) Serial.write(',');
      }
      Serial.println(F("]"));
    }
  }
}

// =============================================
// 添加一个待转发帧到队列
// =============================================
bool enqueueForward(LoRaFrame* f, uint32_t backoff, int8_t rssi, const __FlashStringHelper* type) {
  // RREQ: data[]={id,rssi}对; RREP/ACK/DATA: data[]=中继ID列表
  bool isRREQ = (f->msgType == MSG_RREQ);

  // 检查是否已有相同的待转发（去重 pending 队列）
  for (uint8_t p = 0; p < MAX_PENDING; p++) {
    if (pending[p].active &&
        pending[p].frame.srcId  == f->srcId &&
        pending[p].frame.pathId == f->pathId &&
        pending[p].frame.count  == f->count) {
      // 检查中继 ID 链是否相同
      bool same = true;
      for (uint8_t j = 0; j < f->count; j++) {
        uint8_t idx = isRREQ ? (j * 2) : j;
        if (pending[p].frame.data[idx] != f->data[idx]) { same = false; break; }
      }
      if (same) {
        // 已在队列中，不重复添加
        Serial.print(F("  ")); Serial.print(type); Serial.println(F(" dup"));
        return false;
      }
    }
  }

  // 找空槽
  for (uint8_t p = 0; p < MAX_PENDING; p++) {
    if (!pending[p].active) {
      memcpy(&pending[p].frame, f, FRAME_SIZE);
      pending[p].nextSend = millis() + backoff;
      pending[p].retries  = 1;       // ★ 只发1份（多副本导致盲区过长，阻断多跳链）
      pending[p].active   = true;
      pendingCount++;

      Serial.print(F("  ")); Serial.print(type); Serial.print(F(" q bo="));
      Serial.print(backoff, DEC);
      if (isRREQ) {
        Serial.print(F(" r="));
        Serial.print(rssi, DEC);
      }
      Serial.print(F(" ["));
      for (uint8_t i = 0; i < f->count; i++) {
        Serial.print(F("0x"));
        Serial.print(f->data[isRREQ ? (i * 2) : i], HEX);
        if (isRREQ) {
          Serial.write(':');
          Serial.print((int8_t)f->data[i * 2 + 1], DEC);
        }
        if (i < f->count-1) Serial.write(',');
      }
      Serial.println(F("]"));
      return true;
    }
  }

  // 队列满
  Serial.print(F("  ")); Serial.print(type); Serial.println(F(" drop full"));
  return false;
}

// =============================================
// 收包 → 校验 → 分发
// =============================================
// =============================================
// Heartbeat: periodic liveness signal to mainTerm
// =============================================
void sendHeartbeat() {
  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);
  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = 0x10;
  frame.msgType = MSG_HB;
  // ★ 富心跳 v2：uptime 1B + freeRAM 1B + txPkts 2B + rxPkts 2B + neighbor RSSI
  uint32_t uptime = millis() / 1000;
  uint16_t freeRam = (uint16_t)freeMemory();
  frame.data[0] = (uint8_t)(uptime >> 2);             // uptime/4, 0-1020s
  frame.data[1] = (uint8_t)(freeRam >> 3);            // freeRAM/8, 0-2040B
  frame.data[2] = (txPacketCount >> 8) & 0xFF;
  frame.data[3] = txPacketCount & 0xFF;
  frame.data[4] = (rxPacketCount >> 8) & 0xFF;
  frame.data[5] = rxPacketCount & 0xFF;
  frame.data[6] = lastHeardId;
  frame.data[7] = lastHeardRssi;
  calcChecksum(&frame);
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&frame, FRAME_SIZE);
  LoRa.endPacket(); txPacketCount++;
  Serial.print(F("HB tx nbr=0x"));
  Serial.print(lastHeardId, HEX);
  Serial.print(F(" r="));
  Serial.println(lastHeardRssi, DEC);
}

void handlePacket() {
  int pktSize = LoRa.parsePacket();
  if (pktSize != FRAME_SIZE) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  LoRa.readBytes((uint8_t*)&rxFrame, FRAME_SIZE);
  rxPacketCount++;  // ★ 统计接收包数

  int rssi = LoRa.packetRssi();

  // ★ MSG_KICK / MSG_UNKICK 即使 destId=自己也要处理
  if (rxFrame.msgType == MSG_KICK && rxFrame.destId == MY_NODE_ID) {
    gKicked = true;
    Serial.print(F("KICKED by mainTerm r="));
    Serial.println(rssi, DEC);
    return;
  }
  if (rxFrame.msgType == MSG_UNKICK && rxFrame.destId == MY_NODE_ID) {
    if (gKicked) {
      gKicked = false;
      Serial.print(F("UNKICKED — resumed r="));
      Serial.println(rssi, DEC);
    }
    return;
  }

  bool ignore = false;
  const char* reason = "";

  if (rxFrame.head[0] != FRAME_HEADER_0 || rxFrame.head[1] != FRAME_HEADER_1) {
    ignore = true; reason = "bad magic";
  } else if (rxFrame.destId == MY_NODE_ID && rxFrame.msgType != MSG_CMD_PREPARE && rxFrame.msgType != MSG_CMD_COMMIT) {
    ignore = true; reason = "dest is me";
  } else if (!verifyChecksum(&rxFrame)) {
    ignore = true; reason = "checksum fail";
  }

  // ★ 记录邻居 RSSI（用于富心跳上报，覆盖全部合法帧类型）
  if (!ignore && rxFrame.srcId != MY_NODE_ID && rxFrame.srcId != BROADCAST_ID) {
    lastHeardId   = rxFrame.srcId;
    lastHeardRssi = rssi;
  }

  // ★ 先转发（不阻塞），再打印日志（避免串口拖慢转发）
  if (!ignore) {
    switch (rxFrame.msgType) {
      case MSG_RREQ:  handleRREQ(rssi);  break;
      case MSG_RREP:  handleRREP();      break;
      case MSG_DATA:  handleDATA();      break;
      case MSG_JOIN_REJ:  handleJOIN();            break;
      case MSG_JOIN_ACK:
      case MSG_JOIN_REQ:
      case MSG_CMD_ACK:   handleCMD();            break;
      case MSG_CMD_PREPARE:
      case MSG_CMD_COMMIT: handleTwoPhase(); break;
      case MSG_CMD_READY: handleREADY(rssi); break;  // ★ 转发其他节点的 READY
      case MSG_CMD:
      case MSG_ACK:   handleACK();       break;
      case MSG_ACK_CONFIRM: handleACK(); break;
      default: break;
    }
  }

  Serial.print(ignore ? F("!") : F("H "));
  Serial.print(F("t=0x"));
  Serial.print(rxFrame.msgType, HEX);
  Serial.print(F(" s=0x"));
  Serial.print(rxFrame.srcId, HEX);
  Serial.print(F(" d=0x"));
  Serial.print(rxFrame.destId, HEX);
  Serial.print(F(" n="));
  Serial.print(rxFrame.count, DEC);
  Serial.print(F(" r="));
  Serial.print(rssi, DEC);
  if (ignore) { Serial.write(' '); Serial.print(reason); }
  Serial.println();
}

// =============================================
// RREQ 处理：RSSI 盖章 + 非阻塞入队
// =============================================
void handleRREQ(int8_t rssi) {
  // 防环
  if (rxFrame.count >= MAX_STAMPS) {
    Serial.print(F("  skip stamps="));
    Serial.println(rxFrame.count, DEC);
    return;
  }
  for (uint8_t i = 0; i < rxFrame.count; i++) {
    if (rxFrame.data[i * 2] == MY_NODE_ID) {
      Serial.println(F("  skip stamped"));
      return;
    }
  }

  // RSSI 盖章
  uint8_t idx = rxFrame.count * 2;
  rxFrame.data[idx]     = MY_NODE_ID;
  rxFrame.data[idx + 1] = rssi;
  rxFrame.count++;

  // 重算校验和
  calcChecksum(&rxFrame);

  // ★ 非阻塞：计算退避，入队等待发送
  // 退避 = (节点ID末4位) × 60 + 5
  // 但如果是多跳（count>1），说明是收到另一中继的转发，用小退避
  uint16_t backoff;
  if (rxFrame.count <= 1) {
    backoff = (MY_NODE_ID & 0x0F) * 80 + 10;   // 1跳：0x21→90ms, 0x22→170ms（拉开90ms+60ms空口间隔）
  } else {
    backoff = (MY_NODE_ID & 0x0F) * 40 + 100;  // 多跳：0x21→140ms, 0x22→180ms（等1跳全部清空）
  }

  enqueueForward(&rxFrame, backoff, rssi, F("RREQ"));
}

// =============================================
// RREP 处理：仅当自己在选中路径中时才转发
// =============================================
void handleRREP() {
  bool inPath = false;
  uint8_t myPos = 0;
  for (uint8_t i = 0; i < rxFrame.count && i < MAX_RELAYS; i++) {
    if (rxFrame.data[i] == MY_NODE_ID) { inPath = true; myPos = i; break; }
  }
  if (!inPath) {
    Serial.print(F("  RREP !path I=0x"));
    Serial.print(MY_NODE_ID, HEX);
    Serial.print(F(" path=["));
    for (uint8_t i = 0; i < rxFrame.count && i < MAX_RELAYS; i++) {
      Serial.print(F("0x"));
      Serial.print(rxFrame.data[i], HEX);
      Serial.write(' ');
    }
    Serial.println(F("]"));
    return;
  }

  if (wasForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType)) {
    Serial.println(F("  RREP dedup"));
    return;
  }
  markForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType);

  // ★ 位置退避：多头中继避免同时转发碰撞
  // myPos=0(靠终端)→10ms, myPos=1(靠主节点)→70ms
  uint32_t backoff = myPos * 60 + 10;
  enqueueForward(&rxFrame, backoff, 0, F("RREP"));
}

// =============================================
// ACK 处理：与 RREP 相同，在路径中就转发
// =============================================
void handleACK() {
  bool inPath = false;
  uint8_t myPos = 0;
  for (uint8_t i = 0; i < rxFrame.count && i < MAX_RELAYS; i++) {
    if (rxFrame.data[i] == MY_NODE_ID) { inPath = true; myPos = i; break; }
  }
  if (!inPath) {
    Serial.print(F("  ACK !path I=0x"));
    Serial.print(MY_NODE_ID, HEX);
    Serial.print(F(" path=["));
    for (uint8_t i = 0; i < rxFrame.count && i < MAX_RELAYS; i++) {
      Serial.print(F("0x"));
      Serial.print(rxFrame.data[i], HEX);
      Serial.write(' ');
    }
    Serial.println(F("]"));
    return;
  }

  if (wasForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType)) {
    Serial.println(F("  ACK dedup"));
    return;
  }
  markForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType);

  // ★ 位置退避：多头中继避免同时转发碰撞
  uint32_t backoff = myPos * 60 + 10;
  enqueueForward(&rxFrame, backoff, 0, F("ACK"));
}

// =============================================
// JOIN frame: dedup + transparent forward (no RSSI stamp)
// =============================================
void handleJOIN() {
  if (wasForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType)) {
    Serial.println(F("  JOIN dup"));
    return;
  }
  markForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType);
  uint16_t backoff = (MY_NODE_ID & 0x0F) * 40 + 20;
  enqueueForward(&rxFrame, backoff, 0, F("JOIN"));
}

// =============================================
// CMD frame: check if in relay path, then forward
// =============================================
void handleCMD() {
  uint8_t relayCount = rxFrame.data[0];
  bool inPath = (relayCount == 0);  // relayCount==0 = broadcast/direct, forward
  for (uint8_t i = 0; i < relayCount && i < MAX_RELAYS; i++) {
    if (rxFrame.data[1 + i] == MY_NODE_ID) { inPath = true; break; }
  }
  if (!inPath) { Serial.println(F("  CMD not in path")); return; }
  if (wasForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType)) {
    Serial.println(F("  CMD dup"));
    return;
  }
  markForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType);
  uint16_t backoff = (MY_NODE_ID & 0x0F) * 40 + 20;
  enqueueForward(&rxFrame, backoff, 0, F("CMD"));
}

// =============================================
// Two-phase commit: PREPARE (save+reply READY) / COMMIT (apply switch)
// =============================================
void handleTwoPhase() {
  uint8_t msgType = rxFrame.msgType;
  uint16_t freq = ((uint16_t)rxFrame.data[0] << 8) | rxFrame.data[1];
  uint8_t  sf   = rxFrame.data[2];
  uint8_t  ttl  = rxFrame.data[3];
  int rssi = LoRa.packetRssi();

  if (msgType == MSG_CMD_PREPARE) {
    // === Phase 1: PREPARE ===
    Serial.print(F("PREPARE F=")); Serial.print(freq, DEC);
    Serial.print(F(" SF")); Serial.print(sf, DEC);
    Serial.print(F(" TTL=")); Serial.print(ttl, DEC);
    Serial.print(F(" r=")); Serial.println(rssi, DEC);

    uint8_t status = 0;
    if (freq < 137 || freq > 1020) status = 1;
    else if (sf < 7 || sf > 12)    status = 2;

    if (status == 0) {
      pendingFreq  = freq;
      pendingSf    = sf;
      pendingReady = true;
    }

    // ★ 非阻塞退避：等 PREPARE 转发活动结束（~220ms）后再发 READY
    readyStatusVal = status;
    readySendTime  = millis() + 300 + (MY_NODE_ID & 0x0F) * 60;
    readyPending   = true;

    // Forward PREPARE (TTL limit)
    if (ttl > 1) {
      rxFrame.data[3] = ttl - 1;
      calcChecksum(&rxFrame);
      if (!wasForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType)) {
        markForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType);
        uint16_t backoff = (MY_NODE_ID & 0x0F) * 40 + 20;
        enqueueForward(&rxFrame, backoff, rssi, F("PREPARE"));
      }
    }
  }
  else if (msgType == MSG_CMD_COMMIT) {
    // === Phase 2: COMMIT ===
    Serial.print(F("COMMIT F=")); Serial.print(freq, DEC);
    Serial.print(F(" SF")); Serial.print(sf, DEC);
    Serial.print(F(" TTL=")); Serial.print(ttl, DEC);
    Serial.print(F(" r=")); Serial.println(rssi, DEC);

    if (pendingReady) {
      LoRa.begin(freq * 1000000UL);
      LoRa.setSpreadingFactor(sf);
      LoRa.setTxPower(17);
      pendingReady = false;
      readyPending = false;  // ★ 已切换，取消待发的 READY
      Serial.println(F("COMMIT applied"));
    } else {
      Serial.println(F("COMMIT ignored (no PREPARE)"));
    }

    // Forward COMMIT (TTL limit)
    if (ttl > 1) {
      rxFrame.data[3] = ttl - 1;
      calcChecksum(&rxFrame);
      if (!wasForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType)) {
        markForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType);
        uint16_t backoff = (MY_NODE_ID & 0x0F) * 40 + 20;
        enqueueForward(&rxFrame, backoff, rssi, F("COMMIT"));
      }
    }
  }
}

// =============================================
// READY 转发：让远端节点的就绪确认能通过中继到达主节点
// =============================================
void handleREADY(int8_t rssi) {
  // 不转发自己发出的 READY（已在 handleTwoPhase 中直接发送）
  if (rxFrame.srcId == MY_NODE_ID) return;
  if (wasForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType)) {
    Serial.println(F("  READY dup"));
    return;
  }
  markForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType);
  uint16_t backoff = (MY_NODE_ID & 0x0F) * 40 + 20;
  enqueueForward(&rxFrame, backoff, rssi, F("READY"));
}

// =============================================
// DATA 处理：仅当自己是路径上的中继时才转发
// =============================================
void handleDATA() {
  bool inPath = false;
  uint8_t myPos = 0;
  for (uint8_t i = 0; i < rxFrame.count && i < MAX_RELAYS; i++) {
    if (rxFrame.data[i] == MY_NODE_ID) { inPath = true; myPos = i; break; }
  }
  if (!inPath) {
    Serial.print(F("  DATA !path I=0x"));
    Serial.print(MY_NODE_ID, HEX);
    Serial.print(F(" path=["));
    for (uint8_t i = 0; i < rxFrame.count && i < MAX_RELAYS; i++) {
      Serial.print(F("0x"));
      Serial.print(rxFrame.data[i], HEX);
      Serial.write(' ');
    }
    Serial.println(F("]"));
    return;
  }

  if (wasForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType)) {
    Serial.println(F("  DATA dedup"));
    return;
  }
  markForwarded(rxFrame.srcId, rxFrame.destId, rxFrame.pathId, rxFrame.msgType);

  // ★ 位置退避：多头中继避免同时转发碰撞
  uint32_t backoff = myPos * 60 + 10;
  enqueueForward(&rxFrame, backoff, 0, F("DATA"));
}

// =============================================
// 校验和工具
// =============================================
void calcChecksum(LoRaFrame* f) {
  f->checksum = 0;
  uint8_t c = 0;
  c ^= f->srcId;  c ^= f->destId;
  c ^= f->msgType; c ^= f->pathId;  c ^= f->count;
  for (uint8_t i = 0; i < 8; i++) c ^= f->data[i];
  f->checksum = c;
}

bool verifyChecksum(LoRaFrame* f) {
  uint8_t c = 0;
  c ^= f->srcId;  c ^= f->destId;
  c ^= f->msgType; c ^= f->pathId;  c ^= f->count;
  for (uint8_t i = 0; i < 8; i++) c ^= f->data[i];
  return c == f->checksum;
}