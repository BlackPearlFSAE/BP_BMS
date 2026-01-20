# BCU Code Issues Analysis and Fix Plan

## Current Configuration
- `BMU_NUM = 2` (testing with 2 modules)
- `DISCONNENCTION_TIMEOUT = 2300ms`
- `BMS_COMMUNICATE_TIME = 1000ms`

## Message Flow
**BMU sends every 1000ms:**
- MSG1 (0x182X0001): Operation status
- MSG2 (0x182X0002): Cells 1-8
- MSG3 (0x182X0003): Cells 9-10

With 2 BMUs → 6 messages arrive in burst every ~1 second

---

## Issues Identified

### Issue 1: Single Message Per Loop
**Location:** `bcu.cpp:154-170`

```cpp
if (CAN32_receiveCAN(&receivedMsg, canbusready) == ESP_OK) {
    // Only processes ONE message per loop iteration
}
```

**Problem:** 6 messages arrive in burst, but only 1 processed per loop. Others queue up.

**Why `while` caused problems:** The `while` loop drained ALL messages but the heavy processing inside (AMS reset, module loop) ran for EACH message, causing starvation of other tasks.

### Issue 2: Heavy Processing Per Message
**Location:** `bcu.cpp:158-167`

```cpp
// Inside the if block - runs for EVERY message received
AMS_Package = AMSdata();  // Reinit entire struct
for (int j = 0; j < BMU_NUM; j++) {
    if (isModuleActive(j)) {
        BMU_Package[j].BMUconnected = true;
        packing_AMSstruct(j);  // Heavy nested loops
    }
}
```

**Problem:** This aggregation should happen ONCE after all messages processed, not per-message.

### Issue 3: V_MODULE Reset Bug
**Location:** `bcu.cpp:290`

```cpp
BMU_Package[i].V_MODULE = 0; // reset before recalculate
```

**Problem:** Resets on EVERY message. MSG2 sets cells 1-8, MSG3 resets V_MODULE to 0, then adds only cells 9-10.

### Issue 4: packing_AMSstruct() Inefficiency
**Location:** `bcu.cpp:341-347`

```cpp
void packing_AMSstruct(int moduleIndex) {
    for (int i = 0; i < BMU_NUM; i++) {  // Loops ALL modules
        BMU_Package[i].V_MODULE = 0;
        for (int j = 0; j < CELL_NUM; j++)
            BMU_Package[i].V_MODULE += BMU_Package[i].V_CELL[j];
    }
    // Then uses only moduleIndex...
}
```

**Problem:** Called with moduleIndex, but recalculates ALL modules every time.

### Issue 5: lastModuleResponse[] Initialization
**Location:** `bcu.cpp:41`

```cpp
unsigned long lastModuleResponse[BMU_NUM];  // Initialized to 0
```

**Problem:** At startup, `isModuleActive()` checks `millis() - 0 <= 2300`, which is true for first 2.3 seconds, making all modules appear "active" initially.

---

## Proposed Fix

### Fix Strategy: Separate RX from Processing

```cpp
void loop() {
  unsigned long SESSION_TIME = millis();

  // === CAN TX (unchanged) ===
  if (CAN_SEND_FLG1) { ... }
  if (CAN_SEND_FLG2 && CHARGER_PLUGGED) { ... }

  // === CAN RX: Drain queue, store data only ===
  while (CAN32_receiveCAN(&receivedMsg, canbusready) == ESP_OK) {
    processReceived_BMUmsg(&receivedMsg, BMU_Package);
    if (CHARGER_PLUGGED) processReceived_OBCmsg(&receivedMsg);
    Sustained_Communicate_Time = millis();
  }

  // === Periodic aggregation (every 500ms or on timer) ===
  if (SESSION_TIME - aggregation_timer >= 500) {
    AMS_Package = AMSdata();
    for (int j = 0; j < BMU_NUM; j++) {
      // Calculate V_MODULE here
      BMU_Package[j].V_MODULE = 0;
      for (int k = 0; k < CELL_NUM; k++)
        BMU_Package[j].V_MODULE += BMU_Package[j].V_CELL[k];

      if (isModuleActive(j)) {
        BMU_Package[j].BMUconnected = true;
        packing_AMSstruct(j);
      } else {
        BMU_Package[j].BMUconnected = false;
      }
    }
    aggregation_timer = SESSION_TIME;
  }

  // === Timeout check ===
  if (SESSION_TIME - Sustained_Communicate_Time >= DISCONNENCTION_TIMEOUT) {
    // handle timeout
  }

  // === AMS_OK determination (unchanged) ===
}
```

### Changes Required:

1. **Remove V_MODULE=0 from processReceived_BMUmsg()** (line 290)

2. **Simplify packing_AMSstruct()** - Remove the inner V_MODULE calculation loop:
```cpp
void packing_AMSstruct(int k) {
  AMS_Package.ACCUM_VOLTAGE += (float)(BMU_Package[k].V_MODULE) * 0.02;
  AMS_Package.OVERTEMP_WARNING |= BMU_Package[k].OVERTEMP_WARNING;
  // ... rest unchanged
}
```

3. **Initialize lastModuleResponse[] properly in setup():**
```cpp
for (int i = 0; i < BMU_NUM; i++) {
  lastModuleResponse[i] = 0;  // Or millis() if you want grace period
}
```

4. **Add aggregation_timer variable**

---

## Files to Modify
- `src/bcu.cpp` - Main changes

## Verification
1. Upload fixed code
2. Monitor serial - should see both modules with correct IDs
3. Check `V_MODULE` calculation is correct (sum of 10 cells)
4. Verify no `NO_BYTE_RECV` during normal operation
