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
const uint8_t MSG_JOIN_REQ = 0x20;
const uint8_t MSG_JOIN_ACK = 0x21;
const uint8_t MSG_JOIN_REJ = 0x22;
const uint8_t MSG_CMD = 0x04;
const uint8_t MSG_CMD_ACK = 0x05;
const uint8_t MSG_HB       = 0x03;   // heartbeat
const uint8_t MSG_KICK     = 0x08;   // 踢出命令
const uint8_t MSG_UNKICK   = 0x09;   // 恢复命令
const uint8_t MAX_RELAYS     = 2;
const uint8_t FRAME_SIZE     = 16;
const uint8_t MSG_CMD_PREPARE = 0x0A;
const uint8_t MSG_CMD_READY   = 0x0B;
const uint8_t MSG_CMD_COMMIT  = 0x0C;
const uint8_t BROADCAST_ID    = 0xFF;

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
// ── 传感器缓存 (D3=DHT22, A0=电位器, D2=红外) ──
int8_t   gTemp   = 0;     // 温度 °C
uint8_t  gHum    = 0;     // 湿度 %
uint16_t gPot    = 0;     // 电位 ADC (0-1023)
uint8_t  gIrTrig = 0;     // 红外触发标志
uint8_t  gAlerts = 0;     // 告警位: bit0=高温 bit1=高湿 bit2=电位低 bit3=红外 bit4=DHT有效
bool     gDhtValid = false; // DHT 读取成功标志
uint32_t gLastSensor = 0;
uint32_t gLastIrTrig  = 0;
bool     dataPending   = false;
uint8_t  rreqRetryCount = 0;      // RREQ 当前路径重试计数
// ★ 已知中继追踪：记住上一轮成功通信的中继，本轮若缺失则重试 RREQ
uint8_t  knownRelays[MAX_RELAYS];
uint8_t  knownRelayCount = 0;
uint8_t  missingRetryCount = 0;
const uint8_t MAX_MISSING_RETRIES = 2;  // 缺失中继最多重试 2 次

enum DiscState { JOINING, IDLE, WAITING_RREP, WAITING_ACK, KICKED };
DiscState discState    = JOINING;
uint32_t  rreqSendTime = 0;
uint32_t  ackSendTime  = 0;
uint32_t  lastAction   = 0;       // 全局，ACK 后重置用
const uint32_t RREP_TIMEOUT   = 2000;
const uint32_t ACK_TIMEOUT    = 3500;  // >= mainTerm 的 3×800ms + 回程余量
const uint32_t ROUTE_MAX_AGE  = 10000;  // 10 秒路由老化（每轮 ACK 后主动清掉）
const uint8_t  MAX_RREQ_RETRIES = 3;    // RREQ 最多重试次数（同一路径连续失败才视作不可达）
bool joinedNetwork = false;
uint8_t joinRetryCount = 0;
const uint8_t MAX_JOIN_RETRIES = 3;
const uint32_t JOIN_TIMEOUT = 3000;
uint32_t joinSendTime = 0;
uint32_t lastHbTime    = 0;
uint16_t hbInterval    = 10000;

// ★ 两阶段提交 — 暂存待切换参数
bool     pendingReady = false;
uint16_t pendingFreq  = 530;
uint8_t  pendingSf    = 7;
bool     pauseAODV    = false;  // 切换期间暂停正常通信
// ★ 非阻塞 READY 退避
bool     readyPending = false;
uint32_t readySendTime = 0;
uint8_t  readyStatusVal = 0;

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

// ── DHT 手动读取 (D3, 约 25ms, 零依赖) ──
//    ★ 全程关中断 — 防 LoRa 在任何阶段干扰时序
// ── DHT 手动读取 (D3, 约 25ms, 零依赖) ──
bool readDHT22(int8_t* temp, uint8_t* hum) {
  uint8_t data[5] = {0,0,0,0,0};
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  delayMicroseconds(16000); delayMicroseconds(2000);  // 18ms
  digitalWrite(3, HIGH);
  delayMicroseconds(40);
  pinMode(3, INPUT_PULLUP);
  delayMicroseconds(10);
  // 等待 DHT 响应
  unsigned long t0 = micros();
  while (digitalRead(3) == HIGH) { if (micros() - t0 > 200) return false; }
  t0 = micros();
  while (digitalRead(3) == LOW)  { if (micros() - t0 > 200) return false; }
  t0 = micros();
  while (digitalRead(3) == HIGH) { if (micros() - t0 > 200) return false; }
  // 40-bit 读取
  for (int i = 0; i < 40; i++) {
    t0 = micros();
    while (digitalRead(3) == LOW)  { if (micros() - t0 > 150) return false; }
    t0 = micros();
    while (digitalRead(3) == HIGH) { if (micros() - t0 > 150) return false; }
    data[i / 8] <<= 1;
    if ((micros() - t0) > 50) data[i / 8] |= 1;
  }
  pinMode(3, OUTPUT); digitalWrite(3, HIGH);
  if ((data[0] + data[1] + data[2] + data[3]) != data[4]) return false;
  *hum = data[0];
  *temp = (data[2] & 0x80) ? -(int8_t)(data[2] & 0x7F) : (int8_t)data[2];
  return true;
}

// ── 读取全部传感器并更新缓存 (需要时调用, DHT 最小间隔 2.5s) ──
void readSensors() {
  if (millis() - gLastSensor < 2500) return;
  gLastSensor = millis();
  int8_t t; uint8_t h;
  if (readDHT22(&t, &h)) { gTemp = t; gHum = h; gDhtValid = true; }
  gPot  = 1023 - analogRead(A0);  // 顺时针=大
  gIrTrig = (digitalRead(2) == LOW) ? 1 : 0;
  gAlerts = 0;
  if (gDhtValid)     gAlerts |= 0x10;
  if (gTemp > 35)    gAlerts |= 0x01;
  if (gHum > 80)     gAlerts |= 0x02;
  if (gPot < 200)  gAlerts |= 0x04;
  if (gIrTrig)       gAlerts |= 0x08;
}

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
  checkForPacket();  // ★ 即使 KICKED 也继续监听，等待 UNKICK 恢复
  if (discState == KICKED) { return; }  // 静默：不收发业务帧
  // ★ 非阻塞 READY 发送：即使 pauseAODV 也要发
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
  if (pauseAODV) return;  // 两阶段提交进行中，暂停正常通信
  // ★ 节点状态已合并入 sendHeartbeat，不再单独发送 MSG_NODE_STATUS
  // Heartbeat: 10s +/- 2s jitter (carries uptime + freeRAM + txPkts + neighbor RSSI)
  if (millis() - lastHbTime >= hbInterval) {
    sendHeartbeat();
    lastHbTime = millis();
    hbInterval = 8000 + random(4000);  // 8-12s
  }

  if (discState == JOINING) {
    if (!joinedNetwork) {
      if (joinRetryCount == 0 || (millis() - joinSendTime > JOIN_TIMEOUT)) {
        if (joinRetryCount >= MAX_JOIN_RETRIES) {
          Serial.println(F("JOIN failed after max retries"));
          discState = IDLE;
          lastAction = millis();
          return;
        }
        if (joinRetryCount > 0) { Serial.print(F("JOIN retry ")); Serial.println(joinRetryCount, DEC); }
        sendJOIN();
        joinRetryCount++;
      }
      return;
    }
    discState = IDLE;
    lastAction = millis();
    Serial.println(F("JOIN complete -> IDLE"));
    return;
  }
  uint32_t now = millis();

  if (discState == IDLE) {
    // ★ 红外触发即时上报：检测到入侵 → 刷新周期，不等 10s
    if (gIrTrig && (now - gLastIrTrig > 5000) && (now - lastAction > 3000)) {
      gLastIrTrig = now;
      lastAction = 0;  // 强制触发下一轮通信
      Serial.println(F("\n⚠ IR triggered — immediate report"));
    }

    if (now - lastAction >= 10000) {
      lastAction = now;
      rreqRetryCount = 0;  // ★ 新周期开始，重置重试计数
      missingRetryCount = 0;

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
// =============================================
// Receive channel config command and apply
void handleCMD(LoRaFrame* rx) {
  uint16_t freq  = ((uint16_t)rx->data[3] << 8) | rx->data[4];
  uint8_t  sf    = rx->data[5];
  uint8_t  power = rx->data[6];
  int rssi = LoRa.packetRssi();

  // 纯功率调整：freq=0, sf=0 表示仅调发射功率，不碰频率/SF
  if (freq == 0 && sf == 0) {
    uint8_t st = (power < 2 || power > 20) ? 3 : 0;
    if (st == 0) LoRa.setTxPower(power);
    Serial.print(F("CMD P")); Serial.print(power, DEC);
    Serial.print(F(" r=")); Serial.print(rssi, DEC);
    Serial.println(st == 0 ? F(" OK") : F(" bad power"));
    LoRaFrame ack; memset(&ack, 0, FRAME_SIZE);
    ack.head[0]=FRAME_HEADER_0; ack.head[1]=FRAME_HEADER_1;
    ack.srcId=MY_NODE_ID; ack.destId=rx->srcId; ack.msgType=MSG_CMD_ACK; ack.data[0]=st;
    calcChecksum(&ack); LoRa.beginPacket(); LoRa.write((uint8_t*)&ack, FRAME_SIZE); LoRa.endPacket(); txPacketCount++;
    return;
  }

  Serial.print(F("CMD F=")); Serial.print(freq, DEC);
  Serial.print(F(" SF")); Serial.print(sf, DEC);
  Serial.print(F(" P")); Serial.print(power, DEC);
  Serial.print(F(" r=")); Serial.print(rssi, DEC);
  uint8_t status = 0;
  if (freq < 137 || freq > 1020) { status = 1; }
  else if (sf < 7 || sf > 12)    { status = 2; }
  else if (power < 2 || power > 20)   { status = 3; }
  else if (!LoRa.begin(freq * 1000000UL)) { status = 4; }
  else { LoRa.setSpreadingFactor(sf); LoRa.setTxPower(power); Serial.println(F(" OK")); }
  if (status != 0) { Serial.print(F(" fail ")); Serial.println(status, DEC); }
  LoRaFrame ack;
  memset(&ack, 0, FRAME_SIZE);
  ack.head[0] = FRAME_HEADER_0;
  ack.head[1] = FRAME_HEADER_1;
  ack.srcId   = MY_NODE_ID;
  ack.destId  = rx->srcId;
  ack.msgType = MSG_CMD_ACK;
  ack.data[0] = status;
  calcChecksum(&ack);
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&ack, FRAME_SIZE);
  LoRa.endPacket(); txPacketCount++;
  Serial.print(F("CMD_ACK tx status=")); Serial.println(status, DEC);
}

void handleTwoPhase(LoRaFrame* rx) {
  uint8_t  msgType = rx->msgType;
  uint16_t freq = ((uint16_t)rx->data[0] << 8) | rx->data[1];
  uint8_t  sf   = rx->data[2];
  int rssi = LoRa.packetRssi();

  if (msgType == MSG_CMD_PREPARE) {
    Serial.print(F("PREPARE F=")); Serial.print(freq, DEC);
    Serial.print(F(" SF")); Serial.print(sf, DEC);
    Serial.print(F(" r=")); Serial.println(rssi, DEC);

    uint8_t status = 0;
    if (freq < 137 || freq > 1020) status = 1;
    else if (sf < 7 || sf > 12)    status = 2;

    if (status == 0) {
      pendingFreq  = freq;
      pendingSf    = sf;
      pendingReady = true;
      pauseAODV    = true;  // 暂停通信周期
    }

    // ★ 非阻塞退避：sender 收到 PREPARE 较晚，短退避
    readyStatusVal = status;
    readySendTime  = millis() + 30;
    readyPending   = true;
  }
  else if (msgType == MSG_CMD_COMMIT) {
    Serial.print(F("COMMIT F=")); Serial.print(freq, DEC);
    Serial.print(F(" SF")); Serial.print(sf, DEC);
    Serial.print(F(" r=")); Serial.println(rssi, DEC);

    if (pendingReady) {
      LoRa.begin(freq * 1000000UL);
      LoRa.setSpreadingFactor(sf);
      LoRa.setTxPower(17);
      pendingReady = false;
      readyPending = false;  // ★ 已切换，取消待发的 READY
      pauseAODV    = false;
      // 重置路由（新频道需要重新发现）
      route.valid = false;
      discState   = IDLE;
      lastAction  = millis();
      Serial.println(F("COMMIT applied — route reset"));
    } else {
      Serial.println(F("COMMIT ignored (no PREPARE)"));
    }
  }
}

void sendHeartbeat() {
  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);
  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = DEST_ID;
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
  Serial.print(lastHeardRssi, DEC);
  Serial.println();
}

void sendJOIN() {
  LoRaFrame frame;
  memset(&frame, 0, FRAME_SIZE);
  frame.head[0] = FRAME_HEADER_0;
  frame.head[1] = FRAME_HEADER_1;
  frame.srcId   = MY_NODE_ID;
  frame.destId  = DEST_ID;
  frame.msgType = MSG_JOIN_REQ;
  frame.data[0] = 2;
  calcChecksum(&frame);
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&frame, FRAME_SIZE);
  LoRa.endPacket(); txPacketCount++;
  joinSendTime = millis();
  Serial.print(F("JOIN tx -> 0x"));
  Serial.println(DEST_ID, HEX);
}

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
  LoRa.endPacket(); txPacketCount++;

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
  rxPacketCount++;  // ★ 统计接收包数

  // 过滤
  if (frame.head[0] != FRAME_HEADER_0 || frame.head[1] != FRAME_HEADER_1) {
    return;
  }
  // ★ 记录邻居 RSSI（用于富心跳上报，即使不处理此帧类型也记录）
  {
    int r = LoRa.packetRssi();
    if (frame.srcId != MY_NODE_ID && frame.srcId != BROADCAST_ID) {
      lastHeardId   = frame.srcId;
      lastHeardRssi = r;
    }
  }
  if (frame.destId != MY_NODE_ID && frame.destId != BROADCAST_ID) {
    return;
  }
  if (discState == JOINING) {
    if (frame.msgType == MSG_JOIN_ACK) {
      joinedNetwork = true;
      Serial.println(F("JOIN_OK <- mainTerm"));
      return;
    } else if (frame.msgType == MSG_JOIN_REJ) {
      Serial.print(F("JOIN_REJ <- mainTerm code=0x"));
      Serial.println(frame.data[0], HEX);
      discState = IDLE;
      lastAction = millis();
      return;
    }
    return;
  }
  if (frame.msgType == MSG_KICK) {
    discState = KICKED;
    Serial.print(F("KICKED by mainTerm r="));
    Serial.println(LoRa.packetRssi(), DEC);
    return;
  }
  if (frame.msgType == MSG_UNKICK) {
    if (discState == KICKED) {
      discState = IDLE;
      lastAction = millis();
      route.valid = false;
      Serial.print(F("UNKICKED — resumed r="));
      Serial.println(LoRa.packetRssi(), DEC);
    }
    return;
  }
  if (frame.msgType == MSG_CMD) {
    handleCMD(&frame);
    return;
  }
  if (frame.msgType == MSG_CMD_PREPARE) {
    handleTwoPhase(&frame);
    return;
  }
  if (frame.msgType == MSG_CMD_COMMIT) {
    handleTwoPhase(&frame);
    return;
  }
  if (frame.msgType != MSG_RREP && frame.msgType != MSG_ACK && frame.msgType != MSG_CMD && frame.msgType != MSG_CMD_PREPARE && frame.msgType != MSG_CMD_COMMIT) {
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

  rreqRetryCount = 0;  // ★ RREP 成功收到，重置超时重试计数

  route.destId     = frame->srcId;
  route.relayCount = frame->count;
  if (route.relayCount > MAX_RELAYS) route.relayCount = MAX_RELAYS;
  memcpy(route.relays, frame->data, route.relayCount);
  route.pathId     = frame->pathId;
  route.timestamp  = millis();
  route.valid      = true;

  int rssi = LoRa.packetRssi();

  // ★ 检查已知中继是否缺失：若上一轮有中继本轮 RREP 路径中未出现，重试 RREQ
  //    dataPending 为 true 时才重试（false 表示已决定发 DATA，不再因延迟 RREP 重复触发）
  if (dataPending && knownRelayCount > 0 && missingRetryCount < MAX_MISSING_RETRIES) {
    bool relayMissing = false;
    for (uint8_t i = 0; i < knownRelayCount; i++) {
      bool found = false;
      for (uint8_t j = 0; j < route.relayCount; j++) {
        if (route.relays[j] == knownRelays[i]) { found = true; break; }
      }
      if (!found) {
        Serial.print(F("Missing relay 0x"));
        Serial.print(knownRelays[i], HEX);
        Serial.print(F(" retry #"));
        Serial.println(missingRetryCount + 1, DEC);
        relayMissing = true;
        break;
      }
    }
    if (relayMissing) {
      missingRetryCount++;
      route.valid = false;  // 暂不使用此路径
      delay(20 + missingRetryCount * 30);  // 快速退避: 50ms, 80ms
      sendRREQ();
      return;  // ★ 不发送 DATA，等待新 RREP
    }
  }

  missingRetryCount = 0;  // ★ 所有已知中继到位（或重试耗尽），重置

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
  LoRa.endPacket(); txPacketCount++;

  // ★ 通信成功 → 更新已知中继列表（供下轮缺失检测用）
  knownRelayCount = route.relayCount;
  memcpy(knownRelays, route.relays, knownRelayCount);

  // 再打印日志（不阻塞 ACK_CONFIRM）
  Serial.print(F("ACK ok pid="));
  Serial.print(frame->pathId, DEC);
  Serial.print(F(" r="));
  Serial.print(rssi, DEC);
  Serial.print(F(" v relays="));
  Serial.println(knownRelayCount, DEC);
}

// =============================================
void sendData() {
  readSensors();  // ★ 发送前读取最新传感器数据
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
  // ★ 传感器数据编码 (5 字节, 确保 off+4 < 8 → off≤3, relayCount≤2 满足)
  frame.data[off]     = (uint8_t)(int8_t)gTemp;           // 温度 int8 °C
  frame.data[off + 1] = gHum;                             // 湿度 uint8 %
  frame.data[off + 2] = (gPot >> 8) & 0xFF;             // 电位高字节
  frame.data[off + 3] = gPot & 0xFF;                    // 电位低字节
  frame.data[off + 4] = gAlerts;                          // 告警位
  sampleCounter++;  // 仍递增用于统计

  calcChecksum(&frame);

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&frame, FRAME_SIZE);
  LoRa.endPacket(); txPacketCount++;

  Serial.print(F("DATA tx T="));
  Serial.print(gTemp, DEC);
  Serial.print(F(" H=")); Serial.print(gHum, DEC);
  Serial.print(F("% L=")); Serial.print(gPot, DEC);
  if (gIrTrig) Serial.print(F(" ⚠IR"));
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