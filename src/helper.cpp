#include <Arduino.h>
#include <helper.h>
/* ==================================================== General Functions 
*/
// Split uint16_t to High byte and Low byte
unsigned char* splitHLbyte(unsigned int num){
  static uint8_t temp[2]; // initialize
  temp[0] = (num >> 8) & 255;  // Extract the high byte
  temp[1] = num & 255;         // Extract the low byte
  return temp;
}

// Merge 2 bytes into uint16_t
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte){
  uint16_t temp = (Hbyte << 8) | Lbyte; // bitshiftLeft by 8 OR with low byte
  return temp;
}

// Convert float or uint16 into byte arrays
unsigned char* Encode_bytearray(float f) { 
    static uint8_t c[sizeof(f)]; 
    memcpy(c, &f, sizeof(f));
    return c; 
}

// Convert byte arrays into float or uint16
float Decode_bytearray(unsigned char* c) {
    float f;
    memcpy(&f, c, sizeof(f));
    return f;
}

// Convert Binary to Binary digit array
        // Split and Check bit from MSB -> LSB
        // 1st shift will shift to right by 7 position
                // 1 => 0b00000001 , then AND with 1 so anything that isn't one at 1st pos will be cut off
                // Ex. 42 = 0b00101010
                // 00101010 >> 0 = 00101010 & 00000001 = 0 (point at 1st bit 1 encounter from left . shift to right by 0 pos)
                // 00101010 >> 1 = 00010101 & 00000001 = 1
                // 00101010 >> 2 = 00001010 & 00000001 = 0
            // Check for bit 1 for immediate shutdown

// Convert N bit binary to 16 bit array from MSB-first (big endian) 
bool *toBitarrayMSB(uint16_t num){
  static bool bitarr[16]; // array to hold 8 binary number
  for (int i = 15; i >= 0; i--){
    bool bit = num & 1;
    bitarr[i] = bit;
    num >>= 1; // Right Shift num by 1 pos. before next loop , we AND with 1 again
  } 
  return bitarr; 
}

// Convert N bit binary to 16 bit array from LSB-first (little endian)
bool *toBitarrayLSB(uint16_t num){
  static bool bitarr[16]; // array to hold 8 binary number
  for (int i = 0; i < 16; i++){
    bool bit = num & 1;
    bitarr[i] = bit;
    num >>= 1; // Right Shift num by 1 pos. before next loop , we AND with 1 again
  } 
  return bitarr; 
}

// Convert MSB-first bit array back to uint16_t
uint16_t toUint16FromBitarrayMSB(const bool *bitarr) {
    uint16_t num = 0;
    for (int i = 0; i < 16; i++) {
        if (bitarr[i]) {
            num |= (1 << (15 - i));
        }
    }
    return num;
}

// Convert LSB-first bit array back to uint16_t
uint16_t toUint16FromBitarrayLSB(const bool *bitarr) {
    uint16_t num = 0;
    for (int i = 0; i < 16; i++) {
        if (bitarr[i]) {
            num |= (1 << i);
        }
    }
    return num;
}

/*=============================================================== CAN bus
 * Extended CAN ID Protocol: 0x18 [Prio] [SRC_ADDRESS] 00 [Msg]
 * Format: 0x18PMOOMM where P=Priority, SRC_ADDRESS = (Module ID 1-8), MM=MsgNum
 * Example: 0x18210001 = Priority 2, SRC_ADDRESS 1, Message 1
 */

// Create 11-bit Standard CAN ID: [Prio:4][Src:4][Msg:4]
uint16_t createCANID(uint8_t PRIORITY, uint8_t SRC_ADDRESS, uint8_t MSG_NUM) {
    uint16_t canID = 0;
    canID |= ((PRIORITY & 0x0F) << 8);
    canID |= ((SRC_ADDRESS & 0x0F) << 4);
    canID |= (MSG_NUM & 0x0F);
    return canID;
}

// Create 29-bit Extended CAN ID: 0x18 [Prio:4] [SRC_ADDRESS:4] 00 [Msg:4]
uint32_t createExtendedCANID(uint8_t PRIORITY, uint8_t SRC_ADDRESS, uint8_t MSG_NUM) {
    uint32_t canID = 0x18000000;
    canID |= ((PRIORITY & 0x0F) << 20);
    canID |= ((SRC_ADDRESS & 0x0F) << 16);
    canID |= (MSG_NUM & 0x0F);
    return canID;
}

// Decode 29-bit Extended CAN ID
void decodeExtendedCANID(struct extCANIDDecoded *myCAN, uint32_t canID) {
    myCAN->BASE_ID = (canID >> 24) & 0xFF;         // 0x18
    myCAN->PRIORITY = (canID >> 20) & 0x0F;        // Priority nibble
    myCAN->SRC = (canID >> 16) & 0x0F;             // SRC_ADDRESS number (1-8)
    myCAN->MSG_NUM = canID & 0x0F;                 // Message number
}

// Decode 11-bit Standard CAN ID
void decodeStandardCANID(struct StandardCANIDDecoded *myCAN, uint32_t canID) {
    myCAN->PRIORITY = (canID >> 8) & 0x0F;
    myCAN->SRC = (canID >> 4) & 0x0F;
    myCAN->MSG_NUM = canID & 0x0F;
}

#ifdef ARDUINO_ARCH_AVR
#endif

#ifdef ESP32
/*========================================================== UART and USB
*/
#endif










