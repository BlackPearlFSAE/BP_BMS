#include <Arduino.h>
#include "ntstermistor.h"

const uint32_t res_table[TABLE_SIZE] = {
    193278, 183037, 173310, 164116, 155454, 147302, 139635, 132422, 125631, 119234,  // -40 to -31
    113203, 107511, 102136, 97058,  92258,  87717,  83422,  79358,  75511,  71871,   // -30 to -21
    68424,  65161,  62072,  59146,  56376,  53752,  51267,  48912,  46680,  44564,   // -20 to -11
    42558,  40655,  38850,  37136,  35509,  33963,  32495,  31098,  29771,  28508,   // -10 to -1
    27305,  26161,  25070,  24032,  23041,  22097,  21196,  20337,  19517,  18733,   //  0 to 9
    17986,  17271,  16588,  15936,  15312,  14716,  14146,  13600,  13078,  12579,   //  10 to 19
    12101,  11644,  11206,  10786,  10385,  10000,  9632,   9278,   8940,   8616,    //  20 to 29
    8305,   8007,   7721,   7447,   7184,   6932,   6690,   6458,   6235,   6021,    //  30 to 39
    5815,   5618,   5428,   5246,   5071,   4902,   4741,   4585,   4435,   4291,    //  40 to 49
    4152,   4019,   3891,   3767,   3648,   3533,   3422,   3316,   3213,   3114,    //  50 to 59
    3019,   2926,   2838,   2752,   2669,   2589,   2512,   2438,   2366,   2296,    //  60 to 69
    2229,   2164,   2101,   2041,   1982,   1926,   1871,   1818,   1767,   1717,    //  70 to 79
    1669,   1623,   1578,   1534,   1492,   1451                                     //  80 to 85
};

// ============================================
// Read ADC → Resistance
// ============================================
float read_ntc_resistance(int pin) {
    int adc_raw = analogRead(pin);
    
    if (adc_raw >= ADC_MAX) adc_raw = ADC_MAX - 1;
    if (adc_raw <= 0) adc_raw = 1;
    
    return R_SERIES * ((float)adc_raw / (ADC_MAX - (float)adc_raw));
}

// ============================================
// Resistance → Temperature (direct lookup)
// ============================================
float resistance_to_celsius(float r_ntc) {
    uint32_t r = (uint32_t)r_ntc;
    
    // Out of range
    if (r >= res_table[0]) return TEMP_MIN;
    if (r <= res_table[TABLE_SIZE - 1]) return TEMP_MAX;
    
    // Find closest match (table is descending)
    for (int i = 0; i < TABLE_SIZE - 1; i++) {
        if (r <= res_table[i] && r > res_table[i + 1]) {
            // Pick closer one
            if ((res_table[i] - r) < (r - res_table[i + 1])) {
                return TEMP_MIN + i;
            } else {
                return TEMP_MIN + i + 1;
            }
        }
    }
    return 25;  // Fallback
}

// ============================================
// Temperature → CAN byte
// ============================================
uint8_t temp_to_can_byte(int8_t temp_c) {
    return (uint8_t)((temp_c + 40) * 2);
}


























