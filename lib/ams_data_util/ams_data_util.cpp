/************************* Debug Functions ***************************/

#include <Arduino.h>
#include <ams_data_util.h>

void debugAMSstate(AMSdata* myAMS) {
  Serial.printf("AMS_OK: %d\n", myAMS->AMS_OK);
  Serial.printf("AMS_VOLT: %.2f Low: %d Full: %d \n", myAMS->ACCUM_VOLTAGE);
  Serial.printf("AMS_MAX: %.2f \n", myAMS->ACCUM_MAXVOLTAGE);
  Serial.printf("AMS_MIN: %.2f\n", myAMS->ACCUM_MINVOLTAGE);

  Serial.printf("--Fault status of [%d] Modules--\n",MODULE_NUM);

  Serial.print("OV_WARN: ");
  Serial.println(myAMS->OVERVOLT_WARNING, BIN);
  Serial.print("OV_CRIT: ");
  Serial.println(myAMS->OVERVOLT_CRITICAL, BIN);
  Serial.print("LV_WARN: ");
  Serial.println(myAMS->LOWVOLT_WARNING, BIN);
  Serial.print("LV_CRIT: ");
  Serial.println(myAMS->LOWVOLT_CRITICAL, BIN);
  Serial.print("OT_WARN: ");
  Serial.println(myAMS->OVERTEMP_WARNING, BIN);
  Serial.print("OT_CRIT: ");
  Serial.println(myAMS->OVERTEMP_CRITICAL, BIN);
  Serial.print("DV_WARN: ");
  Serial.println(myAMS->OVERDIV_WARNING, BIN);
  Serial.print("DV_CRIT: ");
  Serial.println(myAMS->OVERDIV_CRITICAL, BIN);
}

void debugBMUModule(BMUdata* myBMU,int moduleNum) {
  Serial.printf("=== BMU %d (ID: %X) ===\n", moduleNum + 1, myBMU[moduleNum].BMU_ID);
  Serial.printf("V_MODULE: %.2fV\n", myBMU[moduleNum].V_MODULE * 0.02f);
  Serial.print("V_CELL: ");
  for (int i = 0; i < CELL_NUM; i++) {
    Serial.printf("%.2f ", myBMU[moduleNum].V_CELL[i] * 0.02f);
  } Serial.println("V");

  Serial.printf("DV: %.2fV\n", myBMU[moduleNum].DV * 0.2f);
  Serial.printf("TEMP: %.1fv, %.1fv\n",
    myBMU[moduleNum].TEMP_SENSE[0] * 0.0125f + 2,
    myBMU[moduleNum].TEMP_SENSE[1] * 0.0125f + 2);
  Serial.printf("Ready to Charge: %d, Connected: %d\n",
    myBMU[moduleNum].BMUneedBalance,
    myBMU[moduleNum].BMUconnected);
  
    
  Serial.printf("--Fault status of [%d] Cells--\n",CELL_NUM);

  Serial.print("  OV warn:");
  Serial.println(myBMU[moduleNum].OVERVOLTAGE_WARNING, BIN);
  Serial.print("  OV crit:");
  Serial.println(myBMU[moduleNum].OVERVOLTAGE_CRITICAL, BIN);

  Serial.print("  LV warn:");
  Serial.println(myBMU[moduleNum].LOWVOLTAGE_WARNING, BIN);
  Serial.print("  LV crit:");
  Serial.println(myBMU[moduleNum].LOWVOLTAGE_CRITICAL, BIN);

  Serial.print("  OT warn:");
  Serial.println(myBMU[moduleNum].OVERTEMP_WARNING, BIN);
  Serial.print("  OT warn:");
  Serial.println(myBMU[moduleNum].OVERTEMP_CRITICAL, BIN);

  Serial.print("  DV warn:");
  Serial.println(myBMU[moduleNum].OVERDIV_VOLTAGE_WARNING, BIN);
  Serial.print("  DV crit:");
  Serial.println(myBMU[moduleNum].OVERDIV_VOLTAGE_CRITICAL, BIN);

  Serial.println();
}

void debugOBCmsg(OBCdata* myOBC) {
  Serial.printf("OBC: %dV, %dA\n",
    myOBC->OBCVolt, myOBC->OBCAmp);
  Serial.printf("Timeout|No_batt|AC_reversed|Overheat|HW_fault");
  Serial.println(myOBC->OBCstatusbit,BIN);
}

/************************* Teleplot Functions ***************************/
// Teleplot extension format: >name:value (single) or >name:v1,v2,v3 (array)
// Use | for multiple values on same line: >name1:v1|name2:v2

void teleplotAMSstate(AMSdata* myAMS) {
  // Accumulator voltage and limits
  Serial.printf(">AMS_Volt:%.2f\n", myAMS->ACCUM_VOLTAGE);
  Serial.printf(">AMS_MaxV:%.2f\n", myAMS->ACCUM_MAXVOLTAGE);
  Serial.printf(">AMS_MinV:%.2f\n", myAMS->ACCUM_MINVOLTAGE);

  // AMS status flags (as integers for plotting)
  Serial.printf(">AMS_OK:%d\n", myAMS->AMS_OK ? 1 : 0);
  Serial.printf(">AMS_ChgReady:%d\n", myAMS->ACCUM_CHG_READY ? 1 : 0);

  // Fault flags (warning/critical as separate traces)
  Serial.printf(">AMS_OV_Warn:%d|AMS_OV_Crit:%d\n",
    myAMS->OVERVOLT_WARNING ? 1 : 0,
    myAMS->OVERVOLT_CRITICAL ? 1 : 0);
  Serial.printf(">AMS_LV_Warn:%d|AMS_LV_Crit:%d\n",
    myAMS->LOWVOLT_WARNING ? 1 : 0,
    myAMS->LOWVOLT_CRITICAL ? 1 : 0);
  Serial.printf(">AMS_OT_Warn:%d|AMS_OT_Crit:%d\n",
    myAMS->OVERTEMP_WARNING ? 1 : 0,
    myAMS->OVERTEMP_CRITICAL ? 1 : 0);
  Serial.printf(">AMS_DV_Warn:%d|AMS_DV_Crit:%d\n",
    myAMS->OVERDIV_WARNING ? 1 : 0,
    myAMS->OVERDIV_CRITICAL ? 1 : 0);
}

void teleplotBMUModule(BMUdata* myBMU, int moduleNum) {
  // Module voltage (convert from 0.02V factor)
  Serial.printf(">M%d_Volt:%.2f\n", moduleNum + 1, myBMU[moduleNum].V_MODULE * 0.02f);

  // Delta voltage (convert from 0.1V factor)
  Serial.printf(">M%d_DV:%.2f\n", moduleNum + 1, myBMU[moduleNum].DV * 0.1f);

  // Status flags
  Serial.printf(">M%d_NeedBal:%d|M%d_Conn:%d\n",
    moduleNum + 1, myBMU[moduleNum].BMUneedBalance ? 1 : 0,
    moduleNum + 1, myBMU[moduleNum].BMUconnected ? 1 : 0);
}

void teleplotBMUCellVoltages(BMUdata* myBMU, int moduleNum) {
  // Individual cell voltages (convert from 0.02V factor)
  for (int i = 0; i < CELL_NUM; i++) {
    Serial.printf(">M%d_C%d:%.2f\n", moduleNum + 1, i + 1, myBMU[moduleNum].V_CELL[i] * 0.02f);
  }
}

void teleplotBMUTemperatures(BMUdata* myBMU, int moduleNum) {
  // Temperature sensors (convert from encoded format: offset -40, factor 0.5C)
  // Stored as raw ADC or encoded value, decode: (value * 0.0125) + 2 for voltage
  // Or if already in temp format: direct value
  Serial.printf(">M%d_T1:%.1f|M%d_T2:%.1f\n",
    moduleNum + 1, myBMU[moduleNum].TEMP_SENSE[0] * 0.0125f + 2,
    moduleNum + 1, myBMU[moduleNum].TEMP_SENSE[1] * 0.0125f + 2);
}

void teleplotBMUFaults(BMUdata* myBMU, int moduleNum) {
  // Count bits set in fault flags for quick visualization
  int ovWarn = __builtin_popcount(myBMU[moduleNum].OVERVOLTAGE_WARNING);
  int ovCrit = __builtin_popcount(myBMU[moduleNum].OVERVOLTAGE_CRITICAL);
  int lvWarn = __builtin_popcount(myBMU[moduleNum].LOWVOLTAGE_WARNING);
  int lvCrit = __builtin_popcount(myBMU[moduleNum].LOWVOLTAGE_CRITICAL);
  int otWarn = __builtin_popcount(myBMU[moduleNum].OVERTEMP_WARNING);
  int otCrit = __builtin_popcount(myBMU[moduleNum].OVERTEMP_CRITICAL);
  int dvWarn = __builtin_popcount(myBMU[moduleNum].OVERDIV_VOLTAGE_WARNING);
  int dvCrit = __builtin_popcount(myBMU[moduleNum].OVERDIV_VOLTAGE_CRITICAL);

  Serial.printf(">M%d_FaultOV:%d|M%d_FaultLV:%d\n",
    moduleNum + 1, ovWarn + ovCrit * 10,  // Critical weighted higher
    moduleNum + 1, lvWarn + lvCrit * 10);
  Serial.printf(">M%d_FaultOT:%d|M%d_FaultDV:%d\n",
    moduleNum + 1, otWarn + otCrit * 10,
    moduleNum + 1, dvWarn + dvCrit * 10);
}

void teleplotOBCmsg(OBCdata* myOBC) {
  Serial.printf(">OBC_Volt:%d|OBC_Amp:%d\n", myOBC->OBCVolt, myOBC->OBCAmp);
  Serial.printf(">OBC_OK:%d|OBC_Status:%d\n", myOBC->OBC_OK ? 1 : 0, myOBC->OBCstatusbit);
}

void teleplotAllModules(BMUdata* BMU_Package, int moduleCount) {
  // Quick overview of all modules on single plot
  for (int i = 0; i < moduleCount; i++) {
    if (BMU_Package[i].BMUconnected) {
      Serial.printf(">ModuleV%d:%.2f\n", i + 1, BMU_Package[i].V_MODULE * 0.02f);
    }
  }
}

void teleplotLocalCells(float* cellvoltages, int cellCount, const char* prefix) {
  // Plot raw float cell voltages (for BCU/BMU local monitoring)
  for (int i = 0; i < cellCount; i++) {
    Serial.printf(">%s_C%d:%.3f\n", prefix, i + 1, cellvoltages[i]);
  }
}

