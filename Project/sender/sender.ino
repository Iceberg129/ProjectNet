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
const uint8_t MSG_ACK_CONFIRM = 0x06;  // ACK 确认回执
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
uint8_t  rreqRetryCount = 0;      // RREQ 当前路径重试计数

enum DiscState { IDLE, WAITING_RREP, WAITING_ACK };
DiscState discState    = IDLE;
uint32_t  rreqSendTime = 0;
uint32_t  ackSendTime  = 0;
uint32_t  lastAction   = 0;       // 全局，ACK 后重置用
const uint32_t RREP_TIMEOUT   = 2000;
const uint32_t ACK_TIMEOUT    = 3500;  // >= mainTerm 的 3×800ms + 回程余量
const uint32_t ROUTE_MAX_AGE  = 10000;  // 10 秒路由老化（每轮 ACK 后主动清掉）
const uint8_t  MAX_RREQ_RETRIES = 3;    // RREQ 最多重试次数（同一路径连续失败才视作不可达）

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
  Serial.print(F(" ->0x"));
  Serial.print(DEST_ID, HEX);
  Serial.println(F(" OK  530MHz SF7"));
}

// =============================================
void loop() {
  checkForPacket();

  uint32_t now = millis();

  if (discState == IDLE) {
    if (now - lastAction >= 10000) {
      lastAction = now;
      rreqRetryCount = 0;  // ★ 新周期开始，重置重试计数

      if (!route.valid || (now - route.timestamp > ROUTE_MAX_AGE)) {
        Serial.print(F("\n->RREQ #"));
        Serial.println(pathIdCounter, DEC);
        sendRREQ();
        dataPending = true;
      } else {
        Serial.print(F("\n->DATA via "));
        if (route.relayCount == 0) {
          Serial.print(F("direct"));
        } else {
          for (uint8_t i = 0; i < route.relayCount; i++) {
            Serial.print(F("0x")); Serial.print(route.relays[i], HEX);
            Serial.write(' ');
          }
        }
        Serial.println();
        sendData();
        dataPending = false;
      }
    }
  }
  else if (discState == WAITING_RREP) {
    if (now - rreqSendTime > RREP_TIMEOUT) {
      if (rreqRetryCount < MAX_RREQ_RETRIES) {
        // ★ 撞包导致 RREQ/RREP 丢失 → 递增退避重试
        rreqRetryCount++;
        Serial.print(F("RREP timeout retry #"));
        Serial.print(rreqRetryCount, DEC);
        Serial.print(F("/"));
        Serial.println(MAX_RREQ_RETRIES, DEC);
        delay(60 + rreqRetryCount * 120);  // 退避: 180ms, 300ms, 420ms（避开撞包窗口）
        sendRREQ();
      } else {
        // ★ 连续多次失败 → 该路径不可达
        discState = IDLE;
        rreqRetryCount = 0;
        route.valid = false;
        lastAction = now;
        Serial.println(F("RREP timeout (max retries) — path unreachable"));
      }
    }
  }
  else if (discState == WAITING_ACK) {
    if (now - ackSendTime > ACK_TIMEOUT) {
      discState = IDLE;
      route.valid = false;  // ★ 路由可能失效，重新发现
      Serial.println(F("ACK timeout"));
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

  Serial.print(F("RREQ tx pid="));
  Serial.println(frame.pathId, DEC);
}

// =============================================
void checkForPacket() {
  int pktSize = LoRa.parsePacket();
  if (pktSize != FRAME_SIZE) {
    if (pktSize > 0) {
      Serial.print(F("!NOISE len="));
      Serial.println(pktSize, DEC);
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
    Serial.println(F("!CHK"));
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
    Serial.print(F("RREP late st="));
    Serial.print(discState, DEC);
    Serial.println(F(" accept"));
  }

  rreqRetryCount = 0;  // ★ RREP 成功收到，重置重试计数

  route.destId     = frame->srcId;
  route.relayCount = frame->count;
  if (route.relayCount > MAX_RELAYS) route.relayCount = MAX_RELAYS;
  memcpy(route.relays, frame->data, route.relayCount);
  route.pathId     = frame->pathId;
  route.timestamp  = millis();
  route.valid      = true;

  int rssi = LoRa.packetRssi();

  // ★ 先发 DATA（等待 relay 完成 RREP 转发，时间按中继数递增）
  // 1跳 relay=10ms退避+50ms TX ≈60ms; 2跳 relay=70ms退避+50ms TX ≈120ms
  if (dataPending) {
    dataPending = false;  // ★ 必须在 while 之前清除，防止循环内 RREP 重复触发
    uint32_t waitMs = 20 + route.relayCount * 60;  // 0→20, 1→80, 2→140
    uint32_t t0 = millis();
    while (millis() - t0 < waitMs) {
      checkForPacket();
    }
    sendData();
  }

  // 再打印日志
  Serial.print(F("RREP pid="));
  Serial.print(frame->pathId, DEC);
  Serial.print(F(" r="));
  Serial.print(rssi, DEC);
  Serial.print(F(" via="));
  if (route.relayCount == 0) {
    Serial.print(F("direct"));
  } else {
    for (uint8_t i = 0; i < route.relayCount; i++) {
      Serial.print(F("0x")); Serial.print(route.relays[i], HEX);
      if (i < route.relayCount - 1) Serial.write(',');
    }
  }
  Serial.println();
  // 如果 dataPending 为 false 但收到了 RREP（重传等），忽略
}

// =============================================
void handleACKFrame(LoRaFrame* frame) {
  if (discState != WAITING_ACK) {
    Serial.print(F("ACK late st="));
    Serial.print(discState, DEC);
    Serial.println(F(" ignore"));
    return;
  }

  int rssi = LoRa.packetRssi();

  // ★ 先发 ACK_CONFIRM（等待 relay 完成 ACK 转发，时间按中继数递增）
  discState  = IDLE;       // ★ 必须在 while 之前，防止循环内 ACK 重复触发
  lastAction = millis();
  route.valid = false;
  {
    uint32_t waitMs = 40 + frame->count * 40;  // 1跳→80ms, 2跳→120ms
    uint32_t t0 = millis();
    while (millis() - t0 < waitMs) {
      checkForPacket();
    }
  }

  LoRaFrame confirm;
  memset(&confirm, 0, FRAME_SIZE);
  confirm.head[0] = FRAME_HEADER_0;
  confirm.head[1] = FRAME_HEADER_1;
  confirm.srcId   = MY_NODE_ID;
  confirm.destId  = frame->srcId;
  confirm.msgType = MSG_ACK_CONFIRM;
  confirm.pathId  = frame->pathId;
  confirm.count   = frame->count;
  if (frame->count > 0) memcpy(confirm.data, frame->data, frame->count);
  calcChecksum(&confirm);
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&confirm, FRAME_SIZE);
  LoRa.endPacket();

  // 再打印日志（不阻塞 ACK_CONFIRM）
  Serial.print(F("ACK ok pid="));
  Serial.print(frame->pathId, DEC);
  Serial.print(F(" r="));
  Serial.print(rssi, DEC);
  Serial.println(F(" v"));
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

  Serial.print(F("DATA tx ctr="));
  Serial.print(sampleCounter - 1, DEC);
  Serial.print(F(" via="));
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