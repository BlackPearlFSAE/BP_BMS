#ifndef NTSTERMISTOR_H
#define NTSTERMISTOR_H

#include <stdint.h>

#define R_SERIES    10000.0f
#define ADC_MAX     4095.0f
// #define NTC_ADC_PIN 1

#define TEMP_MIN    -40
#define TEMP_MAX    85
#define TABLE_SIZE  126

extern const uint32_t res_table[TABLE_SIZE];

float read_ntc_resistance(int pin);
float resistance_to_celsius(float r_ntc);
uint8_t temp_to_can_byte(int8_t temp_c);

#endif









