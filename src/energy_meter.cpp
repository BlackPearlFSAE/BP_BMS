#include <Arduino.h>
#include <RS485.h>

#define RS485_RX_PIN 21
#define RS485_TX_PIN 22
#define RS485_DE_PIN 4

HardwareSerial rs485Serial(2);
RS485 rs485(&rs485Serial, RS485_DE_PIN);

// Broadcast address - works when only 1 meter on the bus
const uint8_t BROADCAST_ADDR[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
// Broadcast addr fr auto discovery
uint8_t meterAddr[6] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

bool addrKnown = false;
static uint8_t rxBuf[128];
static uint8_t rxLen = 0;

// --- Utility ---
void printHex(const uint8_t* data, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    if (i < n - 1) Serial.print(" ");
  }
}

static uint8_t calcCS(const uint8_t* frame, uint8_t len) {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < len; i++) cs += frame[i];
  return cs;
}

// --- DLT645 Send ---
// Builds and sends a DLT645-2007 read request frame.
// dataId: 4-byte identification code (e.g. 0x04000102 for time)
// Passed as uint8_t[4] in protocol byte order (DI0 DI1 DI2 DI3).

void dlt645_send(const uint8_t* addr, uint8_t ctrl, const uint8_t* dataId, uint8_t dataLen) {
  uint8_t frame[32];
  uint8_t idx = 0;

  frame[idx++] = 0x68;
  for (uint8_t i = 0; i < 6; i++) frame[idx++] = addr[i];
  frame[idx++] = 0x68;
  frame[idx++] = ctrl;
  frame[idx++] = dataLen;
  // Add 0x33 to each data byte per DLT645 spec
  for (uint8_t i = 0; i < dataLen; i++) frame[idx++] = dataId[i] + 0x33;

  frame[idx] = calcCS(frame, idx);
  idx++;
  frame[idx++] = 0x16;

  // Send 4x 0xFE preamble
  uint8_t preamble[] = {0xFE, 0xFE, 0xFE, 0xFE};
  rs485.write(preamble, 4);
  rs485.write(frame, idx);

  Serial.print("send:");
  printHex(frame, idx);
  Serial.println();
}

// Convenience: send read request (ctrl=0x11) for a 4-byte data identifier
void dlt645_read(const uint8_t* addr, uint32_t id) {
  uint8_t di[4] = {
    (uint8_t)(id & 0xFF),
    (uint8_t)((id >> 8) & 0xFF),
    (uint8_t)((id >> 16) & 0xFF),
    (uint8_t)((id >> 24) & 0xFF)
  };
  dlt645_send(addr, 0x11, di, 4);
}

// --- DLT645 Receive ---
// Non-blocking receiver. Returns true when a complete valid frame is in rxBuf/rxLen.

bool dlt645_receive() {
  while (rs485.available()) {
    uint8_t b = rs485.read();

    // Skip 0xFE preamble, wait for 0x68
    if (rxLen == 0 && b != 0x68) continue;

    rxBuf[rxLen++] = b;

    // Minimum frame: 68 [addr*6] 68 ctrl len cs 16 = 12 bytes (len=0)
    if (rxLen >= 10) {
      uint8_t dLen = rxBuf[9];
      uint8_t expected = 10 + dLen + 2; // header(10) + data(dLen) + cs(1) + end(1)

      if (rxLen >= expected) {
        // Verify end marker
        if (rxBuf[expected - 1] != 0x16) {
          Serial.println("[BAD END]");
          rxLen = 0;
          return false;
        }
        // Verify checksum
        uint8_t cs = calcCS(rxBuf, expected - 2);
        if (cs != rxBuf[expected - 2]) {
          Serial.println("[BAD CS]");
          rxLen = 0;
          return false;
        }

        rxLen = expected;
        Serial.print("recv:");
        printHex(rxBuf, rxLen);
        Serial.println();
        return true;
      }

      if (rxLen >= sizeof(rxBuf)) {
        Serial.println("[OVF]");
        rxLen = 0;
      }
    }
  }
  return false;
}

// --- Decode helpers ---

// Subtract 0x33 from data bytes in-place, return pointer to decoded data
// Data starts at rxBuf[10], length at rxBuf[9]
void dlt645_decode(uint8_t* out, uint8_t* outLen) {
  *outLen = rxBuf[9];
  for (uint8_t i = 0; i < *outLen; i++) {
    out[i] = rxBuf[10 + i] - 0x33;
  }
}

// --- Address discovery ---
// Send ctrl=0x13 (read address) with broadcast, meter replies with its address

void dlt645_read_addr() {
  dlt645_send(BROADCAST_ADDR, 0x13, nullptr, 0);
}

// Extract address from response frame (bytes 1-6)
void dlt645_parse_addr(uint8_t* dest) {
  for (uint8_t i = 0; i < 6; i++) dest[i] = rxBuf[1 + i];
}

// Convert 6-byte BCD address to 12-char string (MSB first for display)
void addrToStr(const uint8_t* addr, char* str) {
  for (int i = 5; i >= 0; i--) {
    *str++ = '0' + (addr[i] >> 4);
    *str++ = '0' + (addr[i] & 0x0F);
  }
  *str = '\0';
}

// --- Main ---

// Data identifiers (DLT645-2007 / IVY EM630001)
#define DI_TOTAL_ACTIVE_POWER  0x00000000
#define DI_ACTIVE_T1           0x00000100
#define DI_ACTIVE_T2           0x00000200
#define DI_ACTIVE_T3           0x00000300
#define DI_ACTIVE_T4           0x00000400
#define DI_VOLTAGE             0x02010100
#define DI_CURRENT             0x02020100
#define DI_TIME                0x04000102
#define DI_DATE                0x04000101

struct ReadItem {
  uint32_t id;
  const char* name;
};

const ReadItem readList[] = {
  {DI_TIME,                "Time"},
  {DI_VOLTAGE,             "Voltage"},
  {DI_CURRENT,             "Current"},
  {DI_TOTAL_ACTIVE_POWER,  "Total active power"},
  {DI_ACTIVE_T4,           "Active power T4"},
};
const uint8_t READ_COUNT = sizeof(readList) / sizeof(readList[0]);

void setup() {
  Serial.begin(115200);
  rs485Serial.begin(9600, SERIAL_8E1, RS485_RX_PIN, RS485_TX_PIN);
  delay(500);
  Serial.println("\n=== DLT645 Energy Meter ===");

  // Auto-discover meter address using broadcast
  Serial.println("Discovering meter address...");
  dlt645_read_addr();
  uint32_t t = millis();
  while (millis() - t < 1000) {
    if (dlt645_receive()) {
      dlt645_parse_addr(meterAddr);
      addrKnown = true;
      char str[13];
      addrToStr(meterAddr, str);
      Serial.print("Meter found: ");
      Serial.println(str);
      rxLen = 0;
      break;
    }
  }
  if (!addrKnown) {
    Serial.println("No meter found, using broadcast address");
  }
  Serial.println();
}

void loop() {
  static uint8_t idx = 0;
  static uint32_t lastSend = 0;
  static bool waiting = false;

  uint32_t now = millis();
  const uint8_t* addr = addrKnown ? meterAddr : BROADCAST_ADDR;

  // 200 ms delay and queueing mechanism
  if (!waiting && (now - lastSend > 200)) {
    dlt645_read(addr, readList[idx].id);
    lastSend = now;
    waiting = true;
  }

  if (waiting) {
    if (dlt645_receive()) {
      uint8_t data[32], dLen;
      dlt645_decode(data, &dLen);

      Serial.print(readList[idx].name);
      Serial.print(" data: ");
      printHex(data + 4, dLen - 4);
      Serial.println(" OK");

      rxLen = 0;
      waiting = false;
      idx = (idx + 1) % READ_COUNT;
    }

    if (now - lastSend > 500) {
      Serial.print(readList[idx].name);
      Serial.println(" TIMEOUT");
      rxLen = 0;
      waiting = false;
      idx = (idx + 1) % READ_COUNT;
    }
  }
}
