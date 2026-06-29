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
const uint8_t MAX_RELAYS     = 2;
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
uint8_t  candidateCount = 0;
uint8_t  currentPathId  = 0xFF;
uint32_t windowStart    = 0;
const uint32_t COLLECT_WINDOW = 500;   // ★ 150→500ms（串口打印会阻塞收包）

bool     jammingMode  = false;
uint32_t lastJamSend  = 0;

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

  Serial.print(F("Dest 0x"));
  Serial.print(MY_NODE_ID, HEX);
  Serial.println(F(" ready. [AODV-Lite v2]"));
  Serial.println(F("'j'=jammer  window=150ms  freq=530MHz SF7 Tx=17dBm"));
}

// =============================================
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'j') {
      jammingMode = !jammingMode;
      Serial.println(jammingMode ? F("JAMMER ON") : F("JAMMER OFF"));
    }
    if (cmd == 's') {
      // ★ 打印统计信息
      Serial.print(F("\n--- Stats ---"));
      Serial.print(F("  total heard: ")); Serial.print(stat_totalHeard, DEC);
      Serial.print(F("  bad len: ")); Serial.print(stat_badSize, DEC);
      Serial.print(F("  bad magic: ")); Serial.print(stat_badMagic, DEC);
      Serial.print(F("  bad chk: ")); Serial.print(stat_badChecksum, DEC);
      Serial.print(F("  good RREQ: ")); Serial.print(stat_goodRREQ, DEC);
      Serial.print(F("  good DATA: ")); Serial.print(stat_goodDATA, DEC);
      Serial.println();
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

  // 窗口到期 → 裁决
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
    Serial.print(F("[BAD LEN] len="));
    Serial.print(pktSize, DEC);
    Serial.print(F(" RSSI="));
    Serial.print(rssi, DEC);
    // 打印前几个字节帮助诊断
    Serial.print(F(" raw="));
    for (uint8_t i = 0; i < readLen && i < 6; i++) {
      if (raw[i] < 0x10) Serial.write('0');
      Serial.print(raw[i], HEX);
    }
    Serial.println();
    return;
  }

  LoRaFrame* rx = (LoRaFrame*)raw;

  // 2. 魔术字校验
  if (rx->head[0] != FRAME_HEADER_0 || rx->head[1] != FRAME_HEADER_1) {
    stat_badMagic++;
    Serial.print(F("[BAD MAGIC] "));
    Serial.print(rx->head[0], HEX); Serial.write(' ');
    Serial.print(rx->head[1], HEX);
    Serial.print(F(" RSSI="));
    Serial.print(rssi, DEC);
    Serial.println();
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
    Serial.print(F("[BAD CHK] type=0x"));
    Serial.print(rx->msgType, HEX);
    Serial.print(F(" src=0x"));
    Serial.print(rx->srcId, HEX);
    Serial.print(F(" cnt="));
    Serial.print(rx->count, DEC);
    Serial.print(F(" RSSI="));
    Serial.print(rssi, DEC);
    // ★ 打印 data 区前几个字节帮助诊断
    Serial.print(F(" data="));
    for (uint8_t i = 0; i < rx->count * 2 && i < 6; i++) {
      if (rx->data[i] < 0x10) Serial.write('0');
      Serial.print(rx->data[i], HEX);
    }
    Serial.println();
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

  // ★ 将原始帧转发给 PC UI（二进制，16字节）
  // 在 data[7] 塞入末跳 RSSI（供 Python UI 显示链路信号强度）
  {
    uint8_t fwd[FRAME_SIZE];
    memcpy(fwd, raw, FRAME_SIZE);
    if (rx->count < 4) fwd[14] = (uint8_t)(int8_t)rssi;
    Serial.write(fwd, FRAME_SIZE);
  }

  switch (rx->msgType) {
    case MSG_RREQ:  handleRREQ(rx);  break;
    case MSG_DATA:  handleDATA(rx);  break;
    default: break;
  }
}

// =============================================
void handleRREQ(LoRaFrame* rx) {
  int rssi = LoRa.packetRssi();

  // 新 pathId → 重置窗口
  if (rx->pathId != currentPathId) {
    candidateCount = 0;
    currentPathId  = rx->pathId;
    windowStart    = millis();
    memset(candidates, 0, sizeof(candidates));
    Serial.print(F("\n── pathId="));
    Serial.print(rx->pathId, DEC);
    Serial.println(F(" ──"));
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
            sameRelays = false; break;  // 不同中继 → 不同路径
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
        Serial.print(F("  update bneck="));
        Serial.println(newBottleneck, DEC);
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

  // ─── 路线图 ───
  Serial.print(F("\n── pathId="));
  Serial.print(currentPathId, DEC);
  Serial.print(F(" ["));
  Serial.print(candidateCount, DEC);
  Serial.println(F("] ──"));

  for (uint8_t i = 0; i < candidateCount; i++) {
    Candidate* c = &candidates[i];
    bool win = (c == best);

    Serial.print(win ? F("▶ "):F("  "));
    Serial.print(F("0x")); Serial.print(c->srcId, HEX);

    if (c->stampCount == 0) {
      // 直达: 0x30 -----[-45]-----> [C]
      uint8_t n = (uint8_t)(-c->lastHopRssi / 10);
      Serial.write(' ');
      for (uint8_t k = 0; k < n; k++) Serial.write('-');
      Serial.print(F("[-")); Serial.print(-c->lastHopRssi, DEC); Serial.print(F("]"));
      for (uint8_t k = 0; k < n; k++) Serial.write('-');
      Serial.print(F("> [C]"));
    } else {
      // 中继: 0x30 ---[-50]---> 0x21 ---[-42]---> [C]
      for (uint8_t j = 0; j < c->stampCount; j++) {
        int8_t hopRssi = (int8_t)c->stampsData[j * 2 + 1];
        uint8_t n = (uint8_t)(-hopRssi / 10);
        Serial.write(' ');
        for (uint8_t k = 0; k < n; k++) Serial.write('-');
        Serial.print(F("[-")); Serial.print(-hopRssi, DEC); Serial.print(F("]"));
        for (uint8_t k = 0; k < n; k++) Serial.write('-');
        Serial.print(F("> 0x")); Serial.print(c->stampsData[j * 2], HEX);
      }
      // 末跳 → mainTerm
      uint8_t n = (uint8_t)(-c->lastHopRssi / 10);
      Serial.write(' ');
      for (uint8_t k = 0; k < n; k++) Serial.write('-');
      Serial.print(F("[-")); Serial.print(-c->lastHopRssi, DEC); Serial.print(F("]"));
      for (uint8_t k = 0; k < n; k++) Serial.write('-');
      Serial.print(F("> [C]"));
    }

    // 瓶颈 + 选中标记
    Serial.print(F("  b=-"));
    Serial.print(-c->bottleneckRssi, DEC);
    if (win) {
      Serial.print(F(" ★"));
      if (useRelay && bestDirectIdx >= 0) {
        Serial.print(F("  -"));
        Serial.print(-bestRelayedRssi, DEC);
        Serial.print(bestRelayedRssi >= bestDirectRssi ? F("≥"):F("<"));
        Serial.print(F("-"));
        Serial.print(-bestDirectRssi, DEC);
      }
    }
    Serial.println();
  }

  sendRREP(best->srcId, best->pathId, relayCount, relays);

  candidateCount = 0;
  currentPathId  = 0xFF;
}

// =============================================
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

  Serial.print(F("RREP sent → 0x"));
  Serial.print(destId, HEX);
  Serial.print(F("  pathId="));
  Serial.print(pathId, DEC);
  Serial.print(F("  relays="));
  Serial.print(relayCount, DEC);
  Serial.print(relayCount == 0 ? F(" (DIRECT)") : F(" (RELAYED)"));
  Serial.println();
}

// =============================================
void handleDATA(LoRaFrame* rx) {
  int rssi = LoRa.packetRssi();

  // ★ 检测 relay 信标包 (data[0]=0xBE, data[1]=0xAC)
  bool isBeacon = (rx->data[0] == 0xBE && rx->data[1] == 0xAC);

  if (isBeacon) {
    Serial.print(F("\n🔔 BEACON from Relay 0x"));
    Serial.print(rx->srcId, HEX);
    Serial.print(F("  RSSI="));
    Serial.print(rssi, DEC);
    Serial.println(F(" dBm  ← relay→mainTerm link is ALIVE!"));
    return;
  }

  Serial.print(F("\n<<< DATA from 0x"));
  Serial.print(rx->srcId, HEX);

  if (rx->count == 0) {
    Serial.print(F(" (direct)"));
  } else {
    Serial.print(F(" via "));
    for (uint8_t i = 0; i < rx->count && i < MAX_RELAYS; i++) {
      if (i > 0) Serial.write(',');
      Serial.print(F("0x"));
      Serial.print(rx->data[i], HEX);
    }
  }

  Serial.print(F("  RSSI="));
  Serial.print(rssi, DEC);
  Serial.print(F(" dBm  payload="));
  for (uint8_t i = rx->count; i < rx->count + 2 && i < 8; i++) {
    if (rx->data[i] < 0x10) Serial.write('0');
    Serial.print(rx->data[i], HEX);
  }
  Serial.println();

  // ★ 发送 ACK 确认
  sendACK(rx->srcId, rx->pathId, rx->count, rx->data);
}

// =============================================
void sendACK(uint8_t destId, uint8_t pathId,
             uint8_t relayCount, uint8_t* relays) {
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

  // 转发给 PC UI（相位指示器需要看到 ACK 帧）
  Serial.write((uint8_t*)&frame, FRAME_SIZE);

  // ★ 发 3 份提高可靠性（ACK 是关键确认，不容丢失）
  for (uint8_t i = 0; i < 3; i++) {
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&frame, FRAME_SIZE);
    LoRa.endPacket();
    if (i < 2) delay(80);  // 80ms 间隔，确保 relay 转发完上一份并切回 RX
  }

  Serial.print(F("  ACK sent → 0x"));
  Serial.print(destId, HEX);
  if (relayCount > 0) {
    Serial.print(F(" via "));
    for (uint8_t i = 0; i < relayCount && i < MAX_RELAYS; i++) {
      Serial.print(F("0x"));
      Serial.print(relays[i], HEX);
      Serial.write(' ');
    }
  }
  Serial.println();
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