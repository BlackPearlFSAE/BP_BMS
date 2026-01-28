// =======================================================================
// BMS Data , Cells specs , etc.
// =======================================================================
#include <cstdint>

// Cell Configuration 
#define CELL_NUM 10
#define MODULE_NUM 9
// #define MODULE_NUM 2 // Test config
// #define MODULE_NUM 0 // Headless config

/*LG34 Battery*/
#define VMAX_CELL 4.2
#define VNOM_CELL 3.7
#define VMIN_CELL 3.2
#define AH_CELL 34 // Ah
#define DVMAX 0.2

/* Thermistor specs */
#define TEMP_MAX_CELL 60 // C
#define TEMP_SENSOR_NUM 2
// other data here

// AMS Communication
#define STANDARD_BIT_RATE TWAI_TIMING_CONFIG_250KBITS()
#define OBC_COMMUNICATE_TIME  500
#define BMS_COMMUNICATE_TIME  1000
#define DISCONNENCTION_TIMEOUT BMS_COMMUNICATE_TIME * 2.3
#define BCU_ADD 0x18000000
#define OBC_ADD 0x1806E5F4

struct BMUdata {
  // Basic BMU Data
  uint32_t BMU_ID = 0x00; 
  uint8_t V_CELL[CELL_NUM] = {0};
  uint16_t TEMP_SENSE[TEMP_SENSOR_NUM] = {0};
  uint16_t V_MODULE = 0;
  uint8_t DV = 0;
  // FaultCode 10 bit binary representation of C
  uint16_t OVERVOLTAGE_WARNING = 0;
  uint16_t OVERVOLTAGE_CRITICAL = 0;  
  uint16_t LOWVOLTAGE_WARNING = 0;
  uint16_t LOWVOLTAGE_CRITICAL = 0; 
  uint16_t OVERTEMP_WARNING = 0;
  uint16_t OVERTEMP_CRITICAL = 0;
  uint16_t OVERDIV_VOLTAGE_WARNING = 0 ; // Trigger cell balancing of the cell at fault
  uint16_t OVERDIV_VOLTAGE_CRITICAL = 0; // Trigger Charger disable in addition to Cell balancing
  // Status
  uint16_t BalancingDischarge_Cells = 0;
  bool BMUconnected = 0;   // Default as Active true , means each BMU is on the bus
  bool BMUneedBalance = 0;
}; 

// ACCUMULATOR Data , Local to BCU (Make this a struct later , or not? , I don't want over access)
struct AMSdata {

  float ACCUM_VOLTAGE = 0.0; 
  float ACCUM_MAXVOLTAGE = (VMAX_CELL * CELL_NUM * MODULE_NUM); // Default value
  float ACCUM_MINVOLTAGE = (VMIN_CELL * CELL_NUM * MODULE_NUM); // Defualt value assum 8 module
  // float ACCUM_MAXVOLTAGE = (0); // For headless test
  // float ACCUM_MINVOLTAGE = (0); // For headless test
  bool ACCUM_CHG_READY = 0;

  bool OVERVOLT_WARNING = 0;
  bool LOWVOLT_WARNING = 0;
  bool OVERTEMP_WARNING = 0;
  bool OVERDIV_WARNING = 0;

  bool OVERVOLT_CRITICAL = 0;
  bool LOWVOLT_CRITICAL =  0;
  bool OVERTEMP_CRITICAL = 0;
  bool OVERDIV_CRITICAL = 0;

  // bool AMS_OK = 0; // Use this for Active Low Output
  bool AMS_OK = 0; // Use this for Active High Output
};

// Physical condition of OBC On board charger
struct OBCdata {
  uint16_t OBCVolt = 0;
  uint16_t OBCAmp = 0;
  uint8_t OBCstatusbit = 0 ;   // Saftety information
  bool OBC_OK = 1;
};

void debugAMSstate(AMSdata *myAMS);
void debugBMUModule(BMUdata *myBMU,int moduleNum);
void debugOBCmsg(OBCdata *myOCB);

// Teleplot-compatible debug functions (VSCode Teleplot extension format)
// Format: >variable_name:value or >variable_name:v1,v2,v3...
void teleplotAMSstate(AMSdata *myAMS);
void teleplotBMUModule(BMUdata *myBMU, int moduleNum);
void teleplotBMUCellVoltages(BMUdata *myBMU, int moduleNum);
void teleplotBMUTemperatures(BMUdata *myBMU, int moduleNum);
void teleplotBMUFaults(BMUdata *myBMU, int moduleNum);
void teleplotOBCmsg(OBCdata *myOBC);
void teleplotAllModules(BMUdata *BMU_Package, int moduleCount);
void teleplotLocalCells(float *cellvoltages, int cellCount, const char* prefix);



