// =============================================
// mainTerm.ino – 主节点 C（AODV-Lite 终点裁决 修复版）
// 适用于 Arduino Uno + LoRa SX1278 
// =============================================
#include <SPI.h>
#include <LoRa.h>

const uint8_t MY_NODE_ID = 0x10;

const uint8_t FRAME_HEADER_0 = 0x4C;
const uint8_t FRAME_HEADER_1 = 0x6F;
const uint8_t MSG_RREQ       = 0x10;
const uint8_t MSG_RREP       = 0x11;
const uint8_t MSG_DATA       = 0x01;
const uint8_t MSG_ACK        = 0x02;
const uint8_t MSG_ACK_CONFIRM = 0x06;  // ACK 确认回执（超时重传用）
const uint8_t MAX_RELAYS     = 2;
const uint8_t MSG_CMD_ACK   = 0x05;
const uint8_t MSG_HB       = 0x03;   // heartbeat
const uint8_t MSG_CMD       = 0x04;
const uint8_t MSG_JOIN_REJ   = 0x22;
const uint8_t MSG_JOIN_ACK   = 0x21;
const uint8_t MSG_JOIN_REQ   = 0x20;
const uint8_t MAX_STAMPS     = 4;
const uint8_t FRAME_SIZE     = 16;

#pragma pack(push, 1)
typedef struct {
  uint8_t head[2];
  uint8_t srcId;
  uint8_t destId;
  uint8_t msgType;
  uint8_t pathId;
  uint8_t count;
  uint8_t data[8];
  uint8_t checksum;
} LoRaFrame;
#define WL_ENTRY_SIZE 3
#define EEPROM_ADDR_DATA 2
#define EEPROM_ADDR_COUNT 1
#define EEPROM_ADDR_MAGIC 0
#define WL_MAGIC 0xA0

#include <EEPROM.h>
#pragma pack(pop)

#define MAX_CANDIDATES  6
struct Candidate {
  uint8_t  srcId;
  uint8_t  pathId;
  uint8_t  stampCount;
  uint8_t  stampsData[8];
  int8_t   lastHopRssi;
  int8_t   bottleneckRssi;
  bool     valid;
};
Candidate candidates[MAX_CANDIDATES];

// Default whitelist (first-boot EEPROM init)
const uint8_t DEFAULT_WL[][2] = {
  {0x21, 1},
  {0x22, 1},
  {0x30, 2},
};
const uint8_t DEFAULT_WL_COUNT = sizeof(DEFAULT_WL) / sizeof(DEFAULT_WL[0]);

uint8_t  candidateCount = 0;
uint8_t  currentPathId  = 0xFF;
uint32_t windowStart    = 0;
const uint32_t COLLECT_WINDOW = 500;   // ★ 150->500ms（串口打印会阻塞收包）

bool     jammingMode  = false;
bool     waitingCmdAck = false;
uint32_t cmdSendTime   = 0;
uint32_t CMD_ACK_TIMEOUT = 2000;
uint8_t  cmdDestNode   = 0;
// Heartbeat tracking: lastSeen per node (index by node ID)
uint32_t nodeLastSeen[256];   // last heartbeat timestamp per node ID
bool     nodeOnline[256];     // online status per node ID
const uint32_t HB_TIMEOUT = 24000;  // offline after 24s (2 missed HBs)
uint32_t lastJamSend  = 0;

// ★ DATA 去重：同一 (srcId, pathId, data[off], data[off+1]) 2秒内不重复处理
#define DATA_DEDUP_MS 2000
struct DataRecord {
  uint8_t  srcId, pathId;
  uint8_t  payload[2];
  uint32_t timestamp;
};
DataRecord lastData = { 0, 0, {0,0}, 0 };

// ★ 全局统计（帮助诊断中继是否到达）
uint16_t stat_totalHeard   = 0;
uint16_t stat_badSize      = 0;
uint16_t stat_badMagic     = 0;
uint16_t stat_badDest      = 0;
uint16_t stat_badChecksum  = 0;
uint16_t stat_goodRREQ     = 0;
uint16_t stat_goodDATA     = 0;

// =============================================
void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(530E6)) {
    Serial.println(F("LoRa init fail!"));
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setTxPower(17);   // 最大发射功率

  memset(candidates, 0, sizeof(candidates));
  memset(nodeLastSeen, 0, sizeof(nodeLastSeen));
  memset(nodeOnline, 0, sizeof(nodeOnline));

  // EEPROM first-boot init
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != WL_MAGIC) {
    EEPROM.write(EEPROM_ADDR_MAGIC, WL_MAGIC);
    EEPROM.write(EEPROM_ADDR_COUNT, DEFAULT_WL_COUNT);
    for (uint8_t j = 0; j < DEFAULT_WL_COUNT; j++) {
      EEPROM.write(EEPROM_ADDR_DATA + j * WL_ENTRY_SIZE,     DEFAULT_WL[j][0]);
      EEPROM.write(EEPROM_ADDR_DATA + j * WL_ENTRY_SIZE + 1, DEFAULT_WL[j][1]);
      EEPROM.write(EEPROM_ADDR_DATA + j * WL_ENTRY_SIZE + 2, 0);
    }
    Serial.println(F("EEPROM WL init"));
  }

  Serial.print(F("Main 0x"));
  Serial.print(MY_NODE_ID, HEX);
  Serial.println(F(" OK  530MHz SF7"));
}

// =============================================
void loop() {
  if (Serial.available()) {
    Serial.setTimeout(50);
    String cmdLine = Serial.readStringUntil("
");
    cmdLine.trim();
    if (cmdLine.length() == 0) { /* skip */ }
    else if (cmdLine == "j") {
      jammingMode = !jammingMode;
      Serial.println(jammingMode ? F("JAMMER ON") : F("JAMMER OFF"));
    }
    else if (cmdLine == "s") {
      Serial.print(F("
--- Stats ---"));
      Serial.print(F("  total: ")); Serial.print(stat_totalHeard, DEC);
      Serial.print(F("  bad len: ")); Serial.print(stat_badSize, DEC);
      Serial.print(F("  bad magic: ")); Serial.print(stat_badMagic, DEC);
      Serial.print(F("  bad chk: ")); Serial.print(stat_badChecksum, DEC);
      Serial.print(F("  good RREQ: ")); Serial.print(stat_goodRREQ, DEC);
      Serial.print(F("  good DATA: ")); Serial.print(stat_goodDATA, DEC);
      Serial.println();
    }
    else if (cmdLine == "WL?") { wlPrint(); }
    else if (cmdLine.startsWith("WL-")) {
      uint8_t id = (uint8_t)strtol(cmdLine.substring(3).c_str(), NULL, 16);
      if (wlRemove(id)) { Serial.print(F("WL removed 0x")); Serial.println(id, HEX); }
      else { Serial.print(F("WL not found 0x")); Serial.println(id, HEX); }
    }
    else if (cmdLine.startsWith("WL+")) {
      int sep = cmdLine.indexOf(":");
      uint8_t id = (uint8_t)strtol(cmdLine.substring(3, sep).c_str(), NULL, 16);
      uint8_t role = (uint8_t)cmdLine.substring(sep+1).toInt();
      if (wlAdd(id, role)) { Serial.print(F("WL added 0x")); Serial.print(id, HEX); Serial.print(F(" role=")); Serial.println(role, DEC); }
      else { Serial.println(F("WL add failed")); }
    }
    else if (cmdLine.startsWith("CFG:")) {
      int c1 = cmdLine.indexOf(":");
      int c2 = cmdLine.indexOf(":", c1+1);
      uint8_t dest = (uint8_t)strtol(cmdLine.substring(c1+1, c2).c_str(), NULL, 16);
      int fIdx = cmdLine.indexOf("F");
      int sIdx = cmdLine.indexOf("S");
      int pIdx = cmdLine.indexOf("P");
      uint16_t freq = (uint16_t)cmdLine.substring(fIdx+1, sIdx).toInt();
      uint8_t sf    = (uint8_t)cmdLine.substring(sIdx+1, pIdx).toInt();
      uint8_t power = (uint8_t)cmdLine.substring(pIdx+1).toInt();
      sendCMD(dest, freq, sf, power);
    }
  }

  if (jammingMode) {
    if (millis() - lastJamSend >= 1) {
      lastJamSend = millis();
      LoRa.beginPacket();
      LoRa.print(F("JAMMER_ACTIVE_JAMMER_ACTIVE"));
      LoRa.endPacket();
    }
    return;
  }

  static uint32_t lastCheck = 0;
  if (millis() - lastCheck >= 5) {
    lastCheck = millis();
    handlePacket();
  }


  // Heartbeat offline scan (every ~5s via loop timing)
  static uint32_t lastHbScan = 0;
  if (millis() - lastHbScan >= 5000) {
    lastHbScan = millis();
    for (uint8_t id = 0x21; id <= 0x30; id++) {
      if (id == MY_NODE_ID) continue;
      if (nodeOnline[id] && (millis() - nodeLastSeen[id] > HB_TIMEOUT)) {
        nodeOnline[id] = false;
        Serial.print(F("OFFLINE 0x"));
        Serial.println(id, HEX);
        // Send alert frame to PC
        LoRaFrame alert;
        memset(&alert, 0, FRAME_SIZE);
        alert.head[0] = FRAME_HEADER_0;
        alert.head[1] = FRAME_HEADER_1;
        alert.srcId   = id;
        alert.destId  = MY_NODE_ID;
        alert.msgType = MSG_HB;
        alert.count   = 0xFF;  // offline alert flag
        Serial.write((uint8_t*)&alert, FRAME_SIZE);
      }
    }
  }
  // CMD_ACK timeout check
  if (waitingCmdAck && (millis() - cmdSendTime > CMD_ACK_TIMEOUT)) {
    waitingCmdAck = false;
    Serial.print(F("CMD_ACK timeout for 0x"));
    Serial.println(cmdDestNode, HEX);
  }
  // 窗口到期 -> 裁决
  if (candidateCount > 0 && (millis() - windowStart >= COLLECT_WINDOW)) {
    evaluateAndReply();
  }
}

void handlePacket() {
  int pktSize = LoRa.parsePacket();
  if (pktSize <= 0) return;

  int rssi = LoRa.packetRssi();
  stat_totalHeard++;

  // 读取原始帧（即使是错误长度也读出来看看）
  uint8_t raw[FRAME_SIZE];
  uint8_t readLen = (pktSize < FRAME_SIZE) ? pktSize : FRAME_SIZE;
  LoRa.readBytes(raw, readLen);
  while (LoRa.available()) LoRa.read();  // 清空残留

  // 1. 长度校验
  if (pktSize != FRAME_SIZE) {
    stat_badSize++;
    Serial.print(F("!LEN "));
    Serial.print(pktSize, DEC);
    Serial.print(F(" r="));
    Serial.println(rssi, DEC);
    return;
  }

  LoRaFrame* rx = (LoRaFrame*)raw;

  // 2. 魔术字校验
  if (rx->head[0] != FRAME_HEADER_0 || rx->head[1] != FRAME_HEADER_1) {
    stat_badMagic++;
    Serial.print(F("!MAGIC r="));
    Serial.println(rssi, DEC);
    return;
  }

  // 3. 地址过滤
  if (rx->destId != MY_NODE_ID) {
    stat_badDest++;
    // 只对非广播消息打印（减少噪声）
    // Serial.print(F("[NOT FOR ME] dst=0x"));
    // Serial.print(rx->destId, HEX);
    // Serial.print(F(" RSSI="));
    // Serial.println(rssi, DEC);
    return;
  }

  // 4. 校验和
  if (!verifyChecksum(rx)) {
    stat_badChecksum++;
    Serial.print(F("!CHK t=0x"));
    Serial.print(rx->msgType, HEX);
    Serial.print(F(" s=0x"));
    Serial.print(rx->srcId, HEX);
    Serial.print(F(" r="));
    Serial.println(rssi, DEC);
    return;
  }

  // ★ 精简摘要
  if (rx->msgType == MSG_RREQ) stat_goodRREQ++;
  if (rx->msgType == MSG_DATA) stat_goodDATA++;

  Serial.print(rx->msgType == MSG_RREQ ? F("RREQ ") : F("DATA "));
  Serial.print(F("s=0x")); Serial.print(rx->srcId, HEX);
  Serial.print(F(" c="));   Serial.print(rx->count, DEC);
  Serial.print(F(" r="));   Serial.print(rssi, DEC);
  if (rx->msgType == MSG_RREQ && rx->count > 0) {
    Serial.print(F(" ["));
    for (uint8_t i = 0; i < rx->count; i++) {
      Serial.print(F("0x")); Serial.print(rx->data[i*2], HEX);
      Serial.write(':');
      Serial.print((int8_t)rx->data[i*2+1], DEC);
      if (i < rx->count-1) Serial.write(',');
    }
    Serial.write(']');
  }
  Serial.println();

  // ★ 将帧转发给 PC UI（ACK_CONFIRM 由 sendACK 转发，此处跳过避免重复）
  if (rx->msgType != MSG_ACK_CONFIRM) {
    uint8_t fwd[FRAME_SIZE];
    memcpy(fwd, raw, FRAME_SIZE);
    if (rx->count < 4) fwd[14] = (uint8_t)(int8_t)rssi;
    Serial.write(fwd, FRAME_SIZE);
  }

  switch (rx->msgType) {
    case MSG_HB:        handleHeartbeat(rx, rssi);  break;  // heartbeat
    case MSG_JOIN_REQ:  handleJOIN(rx, rssi);   break;
    case MSG_CMD_ACK:   handleCMD_ACK(rx, rssi);  break;
    case MSG_CMD:       handleCMD(rx, rssi);     break;
    case MSG_RREQ:  handleRREQ(rx);  break;
    case MSG_DATA:  handleDATA(rx);  break;
    default: break;
  }
}

// =============================================

// =============================================
// =============================================
void handleHeartbeat(LoRaFrame* rx, int8_t rssi) {
  uint8_t srcId = rx->srcId;
  nodeLastSeen[srcId] = millis();
  if (!nodeOnline[srcId]) {
    nodeOnline[srcId] = true;
    Serial.print(F("ONLINE 0x"));
    Serial.print(srcId, HEX);
    Serial.print(F(" r="));
    Serial.println(rssi, DEC);
    // Send online alert to PC
    LoRaFrame alert;
    memset(&alert, 0, FRAME_SIZE);
    alert.head[0] = FRAME_HEADER_0;
    alert.head[1] = FRAME_HEADER_1;
    alert.srcId   = srcId;
    alert.destId  = MY_NODE_ID;
    alert.msgType = MSG_HB;
    alert.count   = 0;  // online flag
    Serial.write((uint8_t*)&alert, FRAME_SIZE);
  }
}

void handleCMD(LoRaFrame* rx, int8_t rssi) {
  uint16_t freq  = ((uint16_t)rx->data[3] << 8) | rx->data[4];
  uint8_t  sf    = rx->data[5];
  uint8_t  power = rx->data[6];
  Serial.print(F("CMD self F=")); Serial.print(freq, DEC);
  Serial.print(F(" SF")); Serial.print(sf, DEC);
  Serial.print(F(" P")); Serial.print(power, DEC);
  Serial.print(F(" r=")); Serial.print(rssi, DEC);
  if (freq < 137 || freq > 1020) { Serial.println(F(" bad freq")); sendCmdAck(rx->srcId, 1); return; }
  if (sf < 7 || sf > 12)    { Serial.println(F(" bad SF"));   sendCmdAck(rx->srcId, 2); return; }
  if (power < 2 || power > 20) { Serial.println(F(" bad power")); sendCmdAck(rx->srcId, 3); return; }
  if (!LoRa.begin(freq * 1000000UL)) { Serial.println(F(" LoRa fail")); sendCmdAck(rx->srcId, 4); return; }
  LoRa.setSpreadingFactor(sf);
  LoRa.setTxPower(power);
  Serial.println(F(" OK"));
  sendCmdAck(rx->srcId, 0);
}

void sendCmdAck(uint8_t destId, uint8_t status) {
  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);
  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = destId;
  frame.msgType = MSG_CMD_ACK;
  frame.data[0] = status;
  calcChecksum(&frame);
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&frame, FRAME_SIZE);
  LoRa.endPacket();
  Serial.write((uint8_t*)&frame, FRAME_SIZE);
}

// =============================================
void handleCMD_ACK(LoRaFrame* rx, int8_t rssi) {
  uint8_t status = rx->data[0];
  Serial.print(F("CMD_ACK from 0x"));
  Serial.print(rx->srcId, HEX);
  Serial.print(F(" status="));
  Serial.println(status, DEC);
  if (rx->srcId == cmdDestNode) { waitingCmdAck = false; }
  Serial.write((uint8_t*)rx, FRAME_SIZE);
}
void handleRREQ(LoRaFrame* rx) {
  int rssi = LoRa.packetRssi();

  // 新 pathId -> 重置窗口
  if (rx->pathId != currentPathId) {
    candidateCount = 0;
    currentPathId  = rx->pathId;
    windowStart    = millis();
    memset(candidates, 0, sizeof(candidates));
    Serial.print(F("\npid="));
    Serial.print(rx->pathId, DEC);
    Serial.println();
  }

  // ★ 去重：同一 (pathId, srcId, stampCount, relayIds) 只保留 bottleneck 最好的
  //   不同中继 ID 的路径视为不同候选（如 0x21 和 0x22 分开保存）
  for (uint8_t i = 0; i < candidateCount; i++) {
    if (candidates[i].valid &&
        candidates[i].srcId      == rx->srcId &&
        candidates[i].pathId     == rx->pathId &&
        candidates[i].stampCount == rx->count) {
      // 检查中继 ID 是否相同
      bool sameRelays = (rx->count == 0);  // direct 不需要比 ID
      if (rx->count > 0) {
        sameRelays = true;
        for (uint8_t j = 0; j < rx->count; j++) {
          if (candidates[i].stampsData[j * 2] != rx->data[j * 2]) {
            sameRelays = false; break;  // 不同中继 -> 不同路径
          }
        }
      }
      if (!sameRelays) continue;  // 不合并，作为新候选

      // 相同路径，保留瓶颈更好的版本
      int8_t newBottleneck = (int8_t)rssi;
      for (uint8_t j = 0; j < rx->count; j++) {
        int8_t h = (int8_t)rx->data[j * 2 + 1];
        if (h < newBottleneck) newBottleneck = h;
      }
      if (newBottleneck > candidates[i].bottleneckRssi) {
        memcpy(candidates[i].stampsData, rx->data, 8);
        candidates[i].lastHopRssi    = (int8_t)rssi;
        candidates[i].bottleneckRssi = newBottleneck;
        // bottleneck updated (silent)
      }
      return;
    }
  }

  if (candidateCount >= MAX_CANDIDATES) return;

  // 计算木桶瓶颈
  int8_t bottleneck = (int8_t)rssi;
  for (uint8_t i = 0; i < rx->count; i++) {
    int8_t hopRssi = (int8_t)rx->data[i * 2 + 1];
    if (hopRssi < bottleneck) bottleneck = hopRssi;
  }

  // 保存候选
  Candidate* c = &candidates[candidateCount];
  c->srcId          = rx->srcId;
  c->pathId         = rx->pathId;
  c->stampCount     = rx->count;
  memcpy(c->stampsData, rx->data, 8);
  c->lastHopRssi    = (int8_t)rssi;
  c->bottleneckRssi = bottleneck;
  c->valid          = true;
  candidateCount++;

  // ★ 精简日志
  Serial.print(F("  #"));
  Serial.print(candidateCount, DEC);
  Serial.print(rx->count == 0 ? F(" dir "):F(" rly "));
  Serial.print(F("b="));
  Serial.print(bottleneck, DEC);
  if (rx->count > 0) {
    Serial.print(F(" ["));
    for (uint8_t i = 0; i < rx->count; i++) {
      Serial.print(F("0x")); Serial.print(rx->data[i*2], HEX);
      Serial.write(':'); Serial.print((int8_t)rx->data[i*2+1], DEC);
      if (i < rx->count-1) Serial.write(',');
    }
    Serial.write(']');
  }
  Serial.println();
}

// =============================================
void evaluateAndReply() {
  if (candidateCount == 0) return;

  // ★ 分离 direct 和 relayed 候选
  int8_t  bestDirectRssi   = -128;
  int8_t  bestRelayedRssi  = -128;
  int     bestRelayedIdx   = -1;
  int     bestDirectIdx    = -1;

  for (uint8_t i = 0; i < candidateCount; i++) {
    if (!candidates[i].valid) continue;
    if (candidates[i].stampCount == 0) {
      if (candidates[i].bottleneckRssi > bestDirectRssi) {
        bestDirectRssi = candidates[i].bottleneckRssi;
        bestDirectIdx = i;
      }
    } else {
      if (candidates[i].bottleneckRssi > bestRelayedRssi) {
        bestRelayedRssi = candidates[i].bottleneckRssi;
        bestRelayedIdx = i;
      }
    }
  }

  Candidate* best;
  uint8_t relays[MAX_RELAYS] = {0, 0};
  uint8_t relayCount;
  bool useRelay = false;

  if (bestRelayedIdx >= 0 && bestDirectIdx >= 0) {
    if (bestRelayedRssi >= bestDirectRssi) useRelay = true;
  } else if (bestRelayedIdx >= 0) {
    useRelay = true;
  }

  if (useRelay) {
    best = &candidates[bestRelayedIdx];
    relayCount = best->stampCount;
    if (relayCount > MAX_RELAYS) relayCount = MAX_RELAYS;
    for (uint8_t i = 0; i < relayCount; i++) {
      relays[i] = best->stampsData[i * 2];
    }
  } else {
    best = &candidates[bestDirectIdx];
    relayCount = 0;
  }

  // ★ 精简：只打印获胜路径
  Serial.print(F("  pathId="));
  Serial.print(currentPathId, DEC);
  Serial.print(F(" win="));
  if (relayCount == 0) {
    Serial.print(F("direct b="));
    Serial.print(-best->bottleneckRssi, DEC);
  } else {
    for (uint8_t i = 0; i < relayCount; i++) {
      Serial.print(F("0x")); Serial.print(relays[i], HEX);
      if (i < relayCount - 1) Serial.write(',');
    }
    Serial.print(F(" b="));
    Serial.print(-best->bottleneckRssi, DEC);
  }
  if (useRelay && bestDirectIdx >= 0) {
    Serial.print(bestRelayedRssi >= bestDirectRssi ? F(" >=dir"):F(" <dir"));
  }
  Serial.println();

  sendRREP(best->srcId, best->pathId, relayCount, relays);

  candidateCount = 0;
  currentPathId  = 0xFF;
}

// =============================================
// =============================================
void sendCMD(uint8_t destId, uint16_t freq, uint8_t sf, uint8_t power) {
  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);
  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = destId;
  frame.msgType = MSG_CMD;
  frame.data[3] = (freq >> 8) & 0xFF;
  frame.data[4] = freq & 0xFF;
  frame.data[5] = sf;
  frame.data[6] = power;
  calcChecksum(&frame);
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&frame, FRAME_SIZE);
  LoRa.endPacket();
  Serial.write((uint8_t*)&frame, FRAME_SIZE);
  waitingCmdAck = true;
  cmdSendTime   = millis();
  cmdDestNode   = destId;
  Serial.print(F("CMD tx -> 0x"));
  Serial.print(destId, HEX);
  Serial.print(F(" F=")); Serial.print(freq, DEC);
  Serial.print(F(" SF")); Serial.print(sf, DEC);
  Serial.print(F(" P")); Serial.print(power, DEC);
  Serial.println(F(" dBm"));
}

// =============================================
void handleJOIN(LoRaFrame* rx, int8_t rssi) {
  uint8_t srcId = rx->srcId;
  uint8_t role  = rx->data[0];
  Serial.print(F("JOIN from 0x"));
  Serial.print(srcId, HEX);
  Serial.print(F(" role=")); Serial.print(role, DEC);
  Serial.print(F(" r=")); Serial.print(rssi, DEC);
  int8_t idx = wlFind(srcId);
  if (idx >= 0) {
    uint8_t wlRole = EEPROM.read(EEPROM_ADDR_DATA + idx * WL_ENTRY_SIZE + 1);
    if (wlRole == role || wlRole == 0) {
      Serial.println(F(" -> OK"));
      sendJoinResponse(srcId, MSG_JOIN_ACK, 0xAC);
    } else {
      Serial.println(F(" -> role mismatch"));
      sendJoinResponse(srcId, MSG_JOIN_REJ, 0x01);
    }
  } else {
    Serial.println(F(" -> REJECT"));
    sendJoinResponse(srcId, MSG_JOIN_REJ, 0x02);
  }
}

void sendJoinResponse(uint8_t destId, uint8_t msgType, uint8_t statusCode) {
  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);
  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = destId;
  frame.msgType = msgType;
  frame.data[0] = statusCode;
  calcChecksum(&frame);
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&frame, FRAME_SIZE);
  LoRa.endPacket();
  Serial.write((uint8_t*)&frame, FRAME_SIZE);
  Serial.print(msgType == MSG_JOIN_ACK ? F("JOIN_OK -> 0x") : F("JOIN_NO -> 0x"));
  Serial.println(destId, HEX);
}

void sendRREP(uint8_t destId, uint8_t pathId,
              uint8_t relayCount, uint8_t* relays) {
  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);

  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = destId;
  frame.msgType = MSG_RREP;
  frame.pathId  = pathId;
  frame.count   = relayCount;
  memcpy(frame.data, relays, relayCount);

  calcChecksum(&frame);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&frame, FRAME_SIZE);
  LoRa.endPacket();

  // 转发给 PC UI（相位指示器需要看到 RREP 帧）
  Serial.write((uint8_t*)&frame, FRAME_SIZE);

  Serial.print(F("RREP ->0x"));
  Serial.print(destId, HEX);
  Serial.print(F(" pid="));
  Serial.print(pathId, DEC);
  Serial.print(relayCount == 0 ? F(" direct"):F(" via"));
  Serial.println();
}

// =============================================
void handleDATA(LoRaFrame* rx) {
  int rssi = LoRa.packetRssi();

  // ★ 检测 relay 信标包 (data[0]=0xBE, data[1]=0xAC)
  bool isBeacon = (rx->data[0] == 0xBE && rx->data[1] == 0xAC);

  if (isBeacon) {
    Serial.print(F("BEACON 0x"));
    Serial.print(rx->srcId, HEX);
    Serial.print(F(" r="));
    Serial.println(rssi, DEC);
    return;
  }

  // ★ DATA 去重：同一 (srcId, pathId, payload) 在 2s 内的重复帧直接忽略
  {
    uint8_t off = rx->count;
    uint8_t p0 = (off < 8) ? rx->data[off] : 0;
    uint8_t p1 = (off + 1 < 8) ? rx->data[off + 1] : 0;
    if (lastData.srcId   == rx->srcId &&
        lastData.pathId  == rx->pathId &&
        lastData.payload[0] == p0 &&
        lastData.payload[1] == p1 &&
        millis() - lastData.timestamp < DATA_DEDUP_MS) {
      Serial.print(F("DATA dup s=0x")); Serial.print(rx->srcId, HEX);
      Serial.print(F(" pid=")); Serial.print(rx->pathId, DEC);
      Serial.print(F(" r=")); Serial.println(rssi, DEC);
      return;
    }
    lastData.srcId   = rx->srcId;
    lastData.pathId  = rx->pathId;
    lastData.payload[0] = p0;
    lastData.payload[1] = p1;
    lastData.timestamp = millis();
  }

  // ★ 中继路径：延迟 ACK，等中继完成 DATA 转发并回到 RX 模式
  // 中继退避 10~70ms + 空中 50ms → 需等待 60~120ms
  if (rx->count > 0) {
    delay(80 + rx->count * 30);  // 1跳→110ms, 2跳→140ms
  }
  sendACK(rx->srcId, rx->pathId, rx->count, rx->data);

  // 再打印日志
  Serial.print(F("DATA s=0x")); Serial.print(rx->srcId, HEX);
  if (rx->count > 0) {
    Serial.print(F(" via="));
    for (uint8_t i = 0; i < rx->count && i < MAX_RELAYS; i++) {
      Serial.print(F("0x")); Serial.print(rx->data[i], HEX);
      if (i < rx->count-1 && i < MAX_RELAYS-1) Serial.write(',');
    }
  }
  Serial.print(F(" r=")); Serial.print(rssi, DEC);
  Serial.print(F(" pay="));
  for (uint8_t i = rx->count; i < rx->count + 2 && i < 8; i++) {
    if (rx->data[i] < 0x10) Serial.write('0');
    Serial.print(rx->data[i], HEX);
  }
  Serial.println();
}

// =============================================
// ★ ACK 超时重传（最多 3 次，等待 sender 回 ACK_CONFIRM）
// =============================================
void sendACK(uint8_t destId, uint8_t pathId,
             uint8_t relayCount, uint8_t* relays) {
  const uint32_t ACK_TIMEOUT_MS = 800;   // 等待 ACK_CONFIRM 的超时（含 relay 转发+串口开销）
  const uint8_t  ACK_MAX_RETRY  = 3;

  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);
  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = destId;
  frame.msgType = MSG_ACK;
  frame.pathId  = pathId;
  frame.count   = relayCount;
  if (relayCount > 0) memcpy(frame.data, relays, relayCount);
  calcChecksum(&frame);

  for (uint8_t attempt = 0; attempt < ACK_MAX_RETRY; attempt++) {
    // 发送 ACK
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&frame, FRAME_SIZE);
    LoRa.endPacket();

    // 转发原始帧给 PC UI（必需，之后立即开始 polling）
    Serial.write((uint8_t*)&frame, FRAME_SIZE);

    // ★ 立即轮询 ACK_CONFIRM（串口打印移到结果确定后）
    uint32_t t0 = millis();
    bool confirmed = false;
    LoRaFrame rx;
    while (millis() - t0 < ACK_TIMEOUT_MS) {
      int pktSize = LoRa.parsePacket();
      if (pktSize == FRAME_SIZE) {
        LoRa.readBytes((uint8_t*)&rx, FRAME_SIZE);
        if (rx.head[0] == FRAME_HEADER_0 && rx.head[1] == FRAME_HEADER_1 &&
            rx.msgType == MSG_ACK_CONFIRM && rx.srcId == destId &&
            rx.pathId == pathId && verifyChecksum(&rx)) {
          confirmed = true;
          break;
        }
      }
      while (LoRa.available()) LoRa.read();
    }

    // 结果确定后再打印（不阻塞 polling）
    Serial.print(F("ACK #"));
    Serial.print(attempt + 1, DEC);
    Serial.print(F(" ->0x"));
    Serial.print(destId, HEX);

    if (confirmed) {
      Serial.println(F(" ok"));
      Serial.write((uint8_t*)&rx, FRAME_SIZE);
      break;
    }

    if (attempt < ACK_MAX_RETRY - 1) {
      Serial.println(F(" retry"));
    } else {
      Serial.println(F(" fail"));
    }
  }
}

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
// ---- Whitelist lookup ----
int8_t wlFind(uint8_t id) {
  uint8_t count = EEPROM.read(EEPROM_ADDR_COUNT);
  for (uint8_t i = 0; i < count; i++) {
    if (EEPROM.read(EEPROM_ADDR_DATA + i * WL_ENTRY_SIZE) == id) { return i; }
  }
  return -1;
}

// ---- Whitelist add ----
bool wlAdd(uint8_t id, uint8_t role) {
  int8_t idx = wlFind(id);
  if (idx >= 0) { EEPROM.write(EEPROM_ADDR_DATA + idx * WL_ENTRY_SIZE + 1, role); return true; }
  uint8_t count = EEPROM.read(EEPROM_ADDR_COUNT);
  if ((EEPROM_ADDR_DATA + (count + 1) * WL_ENTRY_SIZE) > E2END) return false;
  EEPROM.write(EEPROM_ADDR_DATA + count * WL_ENTRY_SIZE,     id);
  EEPROM.write(EEPROM_ADDR_DATA + count * WL_ENTRY_SIZE + 1, role);
  EEPROM.write(EEPROM_ADDR_DATA + count * WL_ENTRY_SIZE + 2, 0);
  EEPROM.write(EEPROM_ADDR_COUNT, count + 1);
  return true;
}

// ---- Whitelist remove ----
bool wlRemove(uint8_t id) {
  int8_t idx = wlFind(id);
  if (idx < 0) return false;
  uint8_t count = EEPROM.read(EEPROM_ADDR_COUNT);
  if (idx < count - 1) {
    EEPROM.write(EEPROM_ADDR_DATA + idx * WL_ENTRY_SIZE,     EEPROM.read(EEPROM_ADDR_DATA + (count-1) * WL_ENTRY_SIZE));
    EEPROM.write(EEPROM_ADDR_DATA + idx * WL_ENTRY_SIZE + 1, EEPROM.read(EEPROM_ADDR_DATA + (count-1) * WL_ENTRY_SIZE + 1));
    EEPROM.write(EEPROM_ADDR_DATA + idx * WL_ENTRY_SIZE + 2, EEPROM.read(EEPROM_ADDR_DATA + (count-1) * WL_ENTRY_SIZE + 2));
  }
  EEPROM.write(EEPROM_ADDR_COUNT, count - 1);
  return true;
}

// ---- Whitelist print ----
void wlPrint() {
  uint8_t count = EEPROM.read(EEPROM_ADDR_COUNT);
  Serial.print(F("
=== Whitelist ("));
  Serial.print(count, DEC);
  Serial.println(F(") ==="));
  for (uint8_t i = 0; i < count; i++) {
    uint8_t id   = EEPROM.read(EEPROM_ADDR_DATA + i * WL_ENTRY_SIZE);
    uint8_t role = EEPROM.read(EEPROM_ADDR_DATA + i * WL_ENTRY_SIZE + 1);
    Serial.print(F("  0x")); Serial.print(id, HEX);
    Serial.print(F(" role=")); Serial.print(role, DEC);
    Serial.println(role == 1 ? F(" relay") : F(" sender"));
  }
  Serial.println();
}
