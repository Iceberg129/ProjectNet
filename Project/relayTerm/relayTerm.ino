// =============================================
// relayTerm.ino – 中继节点（AODV-Lite RSSI 盖章 非阻塞版）
// 适用于 Arduino Uno + LoRa SX1278 
// 烧录时修改 MY_NODE_ID：B=0x21, D=0x22 ...
// =============================================
#include <SPI.h>
#include <LoRa.h>

// ---------- 节点身份（烧录时修改） ----------
const uint8_t MY_NODE_ID = 0x21;       // 中继节点 ID

// ---------- 协议常量 ----------
const uint8_t FRAME_HEADER_0 = 0x4C;
const uint8_t FRAME_HEADER_1 = 0x6F;
const uint8_t MSG_RREQ       = 0x10;   // 探路请求
const uint8_t MSG_RREP       = 0x11;   // 探路应答
const uint8_t MSG_DATA       = 0x01;   // 用户数据
const uint8_t MSG_ACK        = 0x02;   // 数据确认
const uint8_t MSG_ACK_CONFIRM = 0x06;   // ACK 确认回执
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
    handlePacket();
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
    LoRa.endPacket();

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
void handlePacket() {
  int pktSize = LoRa.parsePacket();
  if (pktSize != FRAME_SIZE) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  LoRa.readBytes((uint8_t*)&rxFrame, FRAME_SIZE);

  int rssi = LoRa.packetRssi();
  bool ignore = false;
  const char* reason = "";

  if (rxFrame.head[0] != FRAME_HEADER_0 || rxFrame.head[1] != FRAME_HEADER_1) {
    ignore = true; reason = "bad magic";
  } else if (rxFrame.destId == MY_NODE_ID) {
    ignore = true; reason = "dest is me";
  } else if (!verifyChecksum(&rxFrame)) {
    ignore = true; reason = "checksum fail";
  }

  // ★ 先转发（不阻塞），再打印日志（避免串口拖慢转发）
  if (!ignore) {
    switch (rxFrame.msgType) {
      case MSG_RREQ:  handleRREQ(rssi);  break;
      case MSG_RREP:  handleRREP();      break;
      case MSG_DATA:  handleDATA();      break;
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