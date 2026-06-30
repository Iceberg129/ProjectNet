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
const uint32_t COLLECT_WINDOW = 500;   // ★ 150->500ms（串口打印会阻塞收包）

bool     jammingMode  = false;
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

  Serial.print(F("Main 0x"));
  Serial.print(MY_NODE_ID, HEX);
  Serial.println(F(" OK  530MHz SF7"));
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
    case MSG_RREQ:  handleRREQ(rx);  break;
    case MSG_DATA:  handleDATA(rx);  break;
    default: break;
  }
}

// =============================================
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