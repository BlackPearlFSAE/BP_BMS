


// TEST DLT645

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// --- Pin Config (LilyGo T-CAN485) ---
#define RS485_TX_PIN 22
#define RS485_RX_PIN 21
#define RS485_SE_PIN 17
#define RS485_PWR_PIN 16

// --- Pin Config (SD Card) ---
#define SD_CS_PIN 5

// --- DLT645 Config ---
// Address: 0xAAAAAAAAAAAA คือ Broadcast Address (ถามทุกตัว)
// 85 12 23120144
// byte meterAddr[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; 
byte meterAddr[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; 
const char* meterADDr = "851223120144";

// Data Tags (ยังไม่บวก 0x33)
const byte TAG_VOLTAGE[] = {0x00, 0x01, 0x01, 0x02};
const byte TAG_CURRENT[] = {0x00, 0x01, 0x02, 0x02};
const byte TAG_POWER[]   = {0x00, 0x01, 0x03, 0x00};
const byte TAG_ENERGY[]  = {0x00, 0x01, 0x00, 0x00};

// Helper: BCD to Decimal
int bcd2dec(byte val) {
  return ((val / 16 * 10) + (val % 16));
}

void encodeMeterAddress(const char* addressStr, uint8_t* addrBytes) {
  // Convert 12-digit string to 6 BCD bytes in reverse order
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t highNibble = addressStr[10 - i*2] - '0';
    uint8_t lowNibble = addressStr[11 - i*2] - '0';
    addrBytes[i] = (highNibble << 4) | lowNibble;
  }
}

// --- ฟังก์ชันบันทึก SD ---
void logToSD(float v, float i, float p, float e) {
  File logFile = SD.open("/dlt645_log.csv", FILE_WRITE);
  if (logFile) {
    logFile.print(millis());
    logFile.print(",");
    logFile.print(v, 1);
    logFile.print(",");
    logFile.print(i, 3);
    logFile.print(",");
    logFile.print(p, 4);
    logFile.print(",");
    logFile.println(e, 2);
    logFile.close();
    Serial.println("Saved to SD");
  } else {
    Serial.println("SD Write Error");
  }
}

// --- ฟังก์ชันหลักในการคุยกับ DLT645 ---
// type: 1=Volt, 2=Amp, 3=Power, 4=Energy (เพื่อกำหนดตัวหารทศนิยม)
float readDLT645(const byte* tag, int type) {
  byte frame[12]; // Start(1)+Addr(6)+Start(1)+Ctrl(1)+Len(1)+Tag(4)+CS(1)+End(1) = 16 bytes แต่ Tag อยู่ใน array แยก
  
  // 1. สร้าง Frame
  byte cmd[] = {
    0x68,
    meterAddr[0], meterAddr[1], meterAddr[2], meterAddr[3], meterAddr[4], meterAddr[5],
    0x68,
    0x11, // Control: Read Data
    0x04, // Data Length
    (byte)(tag[3] + 0x33), // Tag ต้องกลับด้านและ +0x33
    (byte)(tag[2] + 0x33),
    (byte)(tag[1] + 0x33),
    (byte)(tag[0] + 0x33),
    0x00, // CS
    0x16  // End
  };

  // คำนวณ Checksum
  byte sum = 0;
  for(int i=0; i<14; i++) sum += cmd[i];
  cmd[14] = sum;

  // 2. ส่งข้อมูล (Wake up + Command)
  digitalWrite(RS485_SE_PIN, HIGH);
  for(int i=0; i<4; i++) Serial2.write(0xFE); // Wake up
  delay(10);
  Serial2.write(cmd, 16);
  Serial2.flush();
  digitalWrite(RS485_SE_PIN, LOW); // กลับมารอรับ

  // 3. รอรับข้อมูล
  unsigned long timeout = millis();
  while(Serial2.available() == 0) {
    if(millis() - timeout > 1000) return -1.0; // Timeout
  }

  // 4. อ่าน Response
  byte resp[30];
  int len = 0;
  while(Serial2.available()) {
    resp[len++] = Serial2.read();
    delay(5);
    if(len >= 30) break;
  }

  // 5. ตรวจสอบและแปลงค่า (DLT645 Response: ... Ctrl(1) Len(1) [DATA] CS End)
  // Data จะเริ่มประมาณ index 14 (ถ้า address 6 byte)
  // ข้อมูลมาแบบ Little Endian (Low byte first) และต้อง -0x33
  if(len > 14 && resp[0] == 0x68 && resp[len-1] == 0x16) {
    long rawVal = 0;
    
    // Voltage (2 bytes): XX.X
    if (type == 1) { 
       int b1 = resp[14] - 0x33; 
       int b2 = resp[15] - 0x33;
       rawVal = bcd2dec(b2)*100 + bcd2dec(b1);
       return rawVal / 10.0;
    }
    // Current (3 bytes): XXX.XXX
    else if (type == 2) {
       int b1 = resp[14] - 0x33;
       int b2 = resp[15] - 0x33;
       int b3 = resp[16] - 0x33;
       rawVal = bcd2dec(b3)*10000 + bcd2dec(b2)*100 + bcd2dec(b1);
       return rawVal / 1000.0;
    }
    // Power (Active) (3 bytes): XX.XXXX kW (หน่วยมักเป็น kW)
    else if (type == 3) {
       int b1 = resp[14] - 0x33;
       int b2 = resp[15] - 0x33;
       int b3 = resp[16] - 0x33;
       rawVal = bcd2dec(b3)*10000 + bcd2dec(b2)*100 + bcd2dec(b1);
       return rawVal / 10000.0; 
    }
    // Energy (4 bytes): XXXXXX.XX
    else if (type == 4) {
       int b1 = resp[14] - 0x33;
       int b2 = resp[15] - 0x33;
       int b3 = resp[16] - 0x33;
       int b4 = resp[17] - 0x33;
       rawVal = bcd2dec(b4)*1000000 + bcd2dec(b3)*10000 + bcd2dec(b2)*100 + bcd2dec(b1);
       return rawVal / 100.0;
    }
  }

  return -2.0; // Error decoding
}


void setup() {
  Serial.begin(115200);

  // 1. เปิดไฟเลี้ยงและตั้งค่า RS485
  pinMode(RS485_PWR_PIN, OUTPUT);
  digitalWrite(RS485_PWR_PIN, HIGH); // สำคัญ: เปิดไฟเลี้ยงชิป
  delay(100);

  pinMode(RS485_SE_PIN, OUTPUT);
  digitalWrite(RS485_SE_PIN, LOW);   // Default: Receive Mode

  // 2. Start Serial2 (DLT645 ปกติใช้ 2400bps, 8E1 Even Parity)
  // *ถ้าอ่านไม่ได้ ลองเปลี่ยนเป็น 1200 หรือ 9600*
  Serial2.begin(2400, SERIAL_8E1, RS485_RX_PIN, RS485_TX_PIN);

  // 3. Init SD Card
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Failed!");
  } else {
    Serial.println("Done.");
    // สร้าง Header ไฟล์
    if (!SD.exists("/dlt645_log.csv")) {
      File logFile = SD.open("/dlt645_log.csv", FILE_WRITE);
      if (logFile) {
        logFile.println("Time(ms),Voltage(V),Current(A),Power(kW),Energy(kWh)");
        logFile.close();
      }
    }
  }
}

void loop() {
  float volt = readDLT645(TAG_VOLTAGE, 1); // Type 1 format XXX.X
  delay(200);
  float amp  = readDLT645(TAG_CURRENT, 2); // Type 2 format XXX.XXX
  delay(200);
  float watt = readDLT645(TAG_POWER,   3); // Type 3 format XX.XXXX (kW)
  delay(200);
  float kwh  = readDLT645(TAG_ENERGY,  4); // Type 4 format XXXXXX.XX
  delay(200);

  // แสดงผล
  Serial.printf("V: %.1f V, I: %.3f A, P: %.4f kW, E: %.2f kWh\n", volt, amp, watt, kwh);

  // บันทึก SD
  if (volt >= 0 && amp >= 0) { // บันทึกเฉพาะเมื่ออ่านค่าได้ (ไม่ติดลบ error)
    logToSD(volt, amp, watt, kwh);
  } else {
    Serial.println("Error reading meter");
  }

  delay(5000); // รอ 5 วินาที
}