#include <iostream>
#include <stdio.h>
#include <cstring>

// =================================================== Data Conversion Methods

// Split and merge High byte low byte of 16 bit unsigned integer
unsigned char* splitHLbyte(unsigned int num);
unsigned int mergeHLbyte(unsigned char Hbyte, unsigned char Lbyte);

// --Encode and Decode byte array
// Encode float or uint16 into arrays of uint8
unsigned char *Encode_bytearray(float f);
// This is for voltage monitoring
float Decode_bytearray(unsigned char* c);

// Return array of 16 binary digit from 16 bit Binary input
bool *toBitarrayMSB(uint16_t num);
bool *toBitarrayLSB(uint16_t num);


uint16_t toUint16FromBitarrayMSB(const bool *bitarr);
uint16_t toUint16FromBitarrayLSB(const bool *bitarr);

//==================================================== CAN bus Methods

// For ID custom protocol
struct extCANIDDecoded {
    uint8_t PRIORITY;
    uint8_t BASE_ID;
    uint8_t MSG_NUM;
    uint8_t SRC;
    uint8_t DEST;
};

//standard CAN edit by jackie
struct StandardCANIDDecoded {
    uint8_t PRIORITY;
    uint8_t MSG_NUM;
    uint8_t SRC;
};

uint16_t createCANID(uint8_t PRIORITY, uint8_t SRC_ADDRESS, uint8_t MSG_NUM);
// Structure of CAN ID :: Receiver side
void decodeExtendedCANID(struct extCANIDDecoded* myCAN ,uint32_t canID);
void decodeStandardCANID(struct StandardCANIDDecoded *myCAN, uint32_t canID);
// typedef uint16_t __canidExtr;

uint16_t toUint16FromBitarrayMSB(const bool *bitarr);
uint16_t toUint16FromBitarrayLSB(const bool *bitarr);