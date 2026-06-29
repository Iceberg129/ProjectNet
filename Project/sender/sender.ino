// =============================================
// sender.ino – 终端发送节点 A（AODV-Lite 修复版）
// 适用于 Arduino Uno + LoRa SX1278 
// =============================================
#include <SPI.h>
#include <LoRa.h>

const uint8_t MY_NODE_ID = 0x30;
const uint8_t DEST_ID    = 0x10;

const uint8_t FRAME_HEADER_0 = 0x4C;
const uint8_t FRAME_HEADER_1 = 0x6F;
const uint8_t MSG_RREQ       = 0x10;
const uint8_t MSG_RREP       = 0x11;
const uint8_t MSG_DATA       = 0x01;
const uint8_t MSG_ACK        = 0x02;
const uint8_t MAX_RELAYS     = 2;
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

struct RouteEntry {
  uint8_t  destId;
  uint8_t  relayCount;
  uint8_t  relays[MAX_RELAYS];
  uint8_t  pathId;
  uint32_t timestamp;
  bool     valid;
};
RouteEntry route = { DEST_ID, 0, {0,0}, 0, 0, false };

uint8_t  pathIdCounter = 0;
uint8_t  sampleCounter = 0;
bool     dataPending   = false;

enum DiscState { IDLE, WAITING_RREP, WAITING_ACK };
DiscState discState    = IDLE;
uint32_t  rreqSendTime = 0;
uint32_t  ackSendTime  = 0;
uint32_t  lastAction   = 0;       // 全局，ACK 后重置用
const uint32_t RREP_TIMEOUT   = 2000;
const uint32_t ACK_TIMEOUT    = 2000;
const uint32_t ROUTE_MAX_AGE  = 10000;  // 10 秒路由老化（每轮 ACK 后主动清掉）

// =============================================
void setup() {
  // LoRa.setPins(A3);
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(530E6)) {
    Serial.println(F("LoRa init fail!"));
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setTxPower(17);   // ★ 最大发射功率

  Serial.print(F("Sender 0x"));
  Serial.print(MY_NODE_ID, HEX);
  Serial.print(F(" → 0x"));
  Serial.print(DEST_ID, HEX);
  Serial.println(F("  [AODV-Lite v2]  freq=530MHz SF7 Tx=17dBm"));
}

// =============================================
void loop() {
  checkForPacket();

  uint32_t now = millis();

  if (discState == IDLE) {
    if (now - lastAction >= 10000) {
      lastAction = now;

      if (!route.valid || (now - route.timestamp > ROUTE_MAX_AGE)) {
        Serial.print(F("\n--- Route discovery #"));
        Serial.print(pathIdCounter, DEC);
        Serial.println(F(" ---"));
        Serial.print(F("   route "));
        Serial.println(route.valid ? F("EXPIRED (age>60s)") : F("INVALID (never set)"));
        sendRREQ();
        dataPending = true;
      } else {
        Serial.print(F("\n--- Using cached route ---"));
        Serial.print(F("  path="));
        if (route.relayCount == 0) {
          Serial.print(F("direct"));
        } else {
          for (uint8_t i = 0; i < route.relayCount; i++) {
            Serial.print(F("0x"));
            Serial.print(route.relays[i], HEX);
            Serial.write(' ');
          }
        }
        Serial.print(F("  age="));
        Serial.print((now - route.timestamp) / 1000, DEC);
        Serial.println(F("s"));
        sendData();
        dataPending = false;
      }
    }
  }
  else if (discState == WAITING_RREP) {
    if (now - rreqSendTime > RREP_TIMEOUT) {
      discState = IDLE;
      Serial.println(F("RREP timeout. Will retry."));
    }
  }
  else if (discState == WAITING_ACK) {
    if (now - ackSendTime > ACK_TIMEOUT) {
      discState = IDLE;
      route.valid = false;  // ★ 路由可能失效，重新发现
      Serial.println(F("ACK timeout. Route invalidated. Will retry."));
    }
  }
}

// =============================================
void sendRREQ() {
  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);

  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = DEST_ID;
  frame.msgType = MSG_RREQ;
  frame.pathId  = pathIdCounter++;
  frame.count   = 0;

  calcChecksum(&frame);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&frame, FRAME_SIZE);
  LoRa.endPacket();

  discState    = WAITING_RREP;
  rreqSendTime = millis();

  Serial.print(F("RREQ sent  pathId="));
  Serial.println(frame.pathId, DEC);
}

// =============================================
void checkForPacket() {
  int pktSize = LoRa.parsePacket();
  if (pktSize != FRAME_SIZE) {
    if (pktSize > 0) {
      Serial.print(F("[NOISE] pktSize="));
      Serial.print(pktSize, DEC);
      Serial.print(F(" (expected "));
      Serial.print(FRAME_SIZE, DEC);
      Serial.println(F(")"));
    }
    while (LoRa.available()) LoRa.read();
    return;
  }

  LoRaFrame frame;
  LoRa.readBytes((uint8_t*)&frame, FRAME_SIZE);

  // 过滤
  if (frame.head[0] != FRAME_HEADER_0 || frame.head[1] != FRAME_HEADER_1) {
    return;
  }
  if (frame.destId != MY_NODE_ID) {
    return;
  }
  if (frame.msgType != MSG_RREP && frame.msgType != MSG_ACK) {
    return;  // 只关心 RREP 和 ACK
  }
  if (!verifyChecksum(&frame)) {
    Serial.println(F("[FILTER] checksum FAIL"));
    return;
  }

  if (frame.msgType == MSG_RREP) {
    handleRREPFrame(&frame);
  } else if (frame.msgType == MSG_ACK) {
    handleACKFrame(&frame);
  }
}

// =============================================
void handleRREPFrame(LoRaFrame* frame) {
  if (discState != WAITING_RREP) {
    Serial.print(F("[ACCEPT] RREP arrived late (state="));
    Serial.print(discState, DEC);
    Serial.println(F(") — accepting anyway"));
  }

  route.destId     = frame->srcId;
  route.relayCount = frame->count;
  if (route.relayCount > MAX_RELAYS) route.relayCount = MAX_RELAYS;
  memcpy(route.relays, frame->data, route.relayCount);
  route.pathId     = frame->pathId;
  route.timestamp  = millis();
  route.valid      = true;

  int rssi = LoRa.packetRssi();
  Serial.print(F("\n╔══════════════════════════════════════╗"));
  Serial.print(F("\n║ RREP RECEIVED  pathId="));
  Serial.print(frame->pathId, DEC);
  Serial.print(F("  RSSI="));
  Serial.print(rssi, DEC);
  Serial.println(F(" dBm"));
  Serial.print(F("║   route="));
  if (route.relayCount == 0) {
    Serial.print(F("★ DIRECT (no relays)"));
  } else {
    Serial.print(F("via "));
    for (uint8_t i = 0; i < route.relayCount; i++) {
      Serial.print(F("0x"));
      Serial.print(route.relays[i], HEX);
      if (i < route.relayCount - 1) Serial.print(F(" → "));
    }
  }
  Serial.print(F("\n╚══════════════════════════════════════╝\n"));

  if (dataPending) {
    Serial.print(F("  → Sending DATA via this route...\n"));
    sendData();
    dataPending = false;
  }
  // 如果 dataPending 为 false 但收到了 RREP（重传等），忽略
}

// =============================================
void handleACKFrame(LoRaFrame* frame) {
  if (discState != WAITING_ACK) {
    Serial.print(F("[LATE] ACK arrived (state="));
    Serial.print(discState, DEC);
    Serial.println(F(") — ignoring"));
    return;
  }

  int rssi = LoRa.packetRssi();
  Serial.print(F("\n╔══════════════════════════════════════╗"));
  Serial.print(F("\n║ ✅ ACK RECEIVED  pathId="));
  Serial.print(frame->pathId, DEC);
  Serial.print(F("  RSSI="));
  Serial.print(rssi, DEC);
  Serial.println(F(" dBm"));
  Serial.print(F("║   Delivery confirmed!"));
  Serial.print(F("\n╚══════════════════════════════════════╝\n"));

  discState  = IDLE;
  lastAction = millis();   // ★ 启动 3 秒等待
  route.valid = false;     // ★ 每次传输后强制重新发现，确保路径实时准确
}

// =============================================
void sendData() {
  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);

  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = route.destId;
  frame.msgType = MSG_DATA;
  frame.pathId  = route.pathId;
  frame.count   = route.relayCount;

  memcpy(frame.data, route.relays, route.relayCount);

  uint8_t off = route.relayCount;
  frame.data[off]     = sampleCounter++;
  frame.data[off + 1] = 0xAB;

  calcChecksum(&frame);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&frame, FRAME_SIZE);
  LoRa.endPacket();

  Serial.print(F("DATA sent  ctr="));
  Serial.print(sampleCounter - 1, DEC);
  Serial.print(F("  relays="));
  Serial.println(route.relayCount, DEC);

  // ★ 等待 ACK 确认
  discState   = WAITING_ACK;
  ackSendTime = millis();
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