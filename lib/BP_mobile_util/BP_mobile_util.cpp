#include <Arduino.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <BP_mobile_util.h>
// ============================================================================
// BP Mobile helper
// ============================================================================
/* Local instance (only use here) */

// typedef void (*RegisterTopicFn)(const char* name);
// typedef uint64_t (*TimeProviderFn)(uint64_t time);

// // // static stored callbacks + client name
// static const char* stored_client_name = nullptr;
// static RegisterTopicFn registerClientTopics_fn = nullptr;
// static TimeProviderFn serverSyncTime_fn  = nullptr;

// // // Callback function setter
// void setRegisterCallback(RegisterTopicFn cb) { registerClientTopics_fn = cb; }
// void setTimeProvider(TimeProviderFn fn) {serverSyncTime_fn = fn; }


// User calls these in their Arduino Sketch
void BPMobileConfig::setRegisterCallback(RegisterTopicFn callback) { _registration_cb = callback; }
void BPMobileConfig::setTimeProvider(TimeProviderFn callback) { _timesourceProvider_fn = callback;}
void BPMobileConfig::setClientName(const char* name) { _client_name = name; }


void BPMobileConfig::registerMCUTopic() {
        if (_registration_cb!= nullptr)
            _registration_cb(_client_name); 
    }
void BPMobileConfig::syncMCUtime_with_provider(uint64_t time){
        if (_timesourceProvider_fn!= nullptr)
            _timesourceProvider_fn(time); 
    }

void BPMobileConfig::initWebSocket(const char* serverHost, const int serverPort, const char* clientName) {
  // Serial mutex protected prints (initWebSocket called from setup, but good practice)
  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println("--- WebSocket Initialization ---");
    Serial.print("Connecting to: ws://");
    Serial.print(serverHost);Serial.print(":");Serial.println(serverPort);
    xSemaphoreGive(serialMutex);
  }

  // // Debug: Check pointers before using
  // Serial.print("[initWebSocket] webSocket=0x");
  // Serial.print((unsigned long)this->webSocket, HEX);
  // Serial.print(", webSocketstatus=0x");
  // Serial.println((unsigned long)this->webSocketstatus, HEX);

  /* 
  * IMPORTANT: Set event handler BEFORE calling begin()
  * avoid race conditions 
  * */
  this->webSocket->onEvent([this](WStype_t type, uint8_t* payload, size_t length) {
    this->webSocketEvent(type, payload, length);
  });
  this->webSocket->setReconnectInterval(5000);
  this->webSocket->begin(serverHost, serverPort, "/");

  if (this->webSocketstatus != nullptr) {
    this->webSocketstatus->connectionStartTime = millis();
  } else if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println("[ERROR] webSocketstatus is NULL!");
    xSemaphoreGive(serialMutex);
  }

}
void BPMobileConfig::webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {

  // Null pointer handling
  if (!this) {
    if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      Serial.println("[ERROR] this pointer is NULL in webSocketEvent!");
      xSemaphoreGive(serialMutex);
    }
    return;
  }

  switch(type) {
    case WStype_DISCONNECTED:
      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.println("[WebSocket] Disconnected");
        xSemaphoreGive(serialMutex);
      }
      if (this->webSocketstatus) {
        this->webSocketstatus->isConnected = false;
        this->webSocketstatus->isRegistered = false;
      }
      break;

    case WStype_CONNECTED:
      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.println("[WebSocket] Connected!");
        xSemaphoreGive(serialMutex);
      }
      if (this->webSocketstatus) {
        this->webSocketstatus->isConnected = true;
      }
      this->registerMCUTopic();
      break;

    case WStype_TEXT:
      handleMessage((char*)payload);
      break;

    case WStype_ERROR:
      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.print("[WebSocket] Error: ");
        Serial.println((char*)payload);
        xSemaphoreGive(serialMutex);
      }
      break;

    default:
      break;
  }
}
void BPMobileConfig::handleMessage(const char* message) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      Serial.print("[WebSocket] JSON error: ");
      Serial.println(error.c_str());
      xSemaphoreGive(serialMutex);
    }
    return;
  }
  String type = doc["type"] | "";

  if (type == "registration_response") {
    String status = doc["status"] | "";

    if (status == "accepted") {
      this->webSocketstatus->isRegistered = true;

      // Sync time
      if (doc["system_time"].is<unsigned long long>()) {
        JsonObject sysTime = doc["system_time"];
        if (sysTime["timestamp_ms"].is<unsigned long long>()) {
          uint64_t serverTime = sysTime["timestamp_ms"];
          this->syncMCUtime_with_provider(serverTime);
        }
      }

      // Mutex-protected registration success prints
      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.println("[WebSocket] ✓ Registration ACCEPTED!");
        Serial.println("[WebSocket] Ready to stream data!");
        xSemaphoreGive(serialMutex);
      }

    } else if (status == "rejected") {
      String msg = doc["message"] | "Unknown error";
      this->webSocketstatus->isRegistered = false;

      // Mutex-protected rejection prints
      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.println("[WebSocket] ✗ Registration REJECTED!");
        Serial.print("  Reason: ");
        Serial.println(msg);
        if (msg.indexOf("already exists") >= 0) {
          Serial.println("\n*** Change 'clientName' to a unique value! ***");
        }
        xSemaphoreGive(serialMutex);
      }
    }

  } else if (type == "ping") {
    String pingId = doc["ping_id"] | "";
    this->webSocketstatus->lastPingReceived = millis();

    JsonDocument pongDoc;
    pongDoc["type"] = "pong";
    pongDoc["ping_id"] = pingId;
    pongDoc["timestamp"] = millis();

    String pong;
    serializeJson(pongDoc, pong);
    webSocket->sendTXT(pong);

    // Mutex-protected ping/pong log (this was the main culprit for garbled output)
    if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      Serial.print("[WebSocket] Ping/Pong (ID: ");
      Serial.print(pingId);
      Serial.println(")");
      xSemaphoreGive(serialMutex);
    }
  }
}

// constructor
BPMobileConfig::BPMobileConfig(WebSocketsClient* ws,socketstatus* status)
{
  // if websocket object were provided
  if (ws) {
    this->webSocket = ws;
    this->_owns_webSocket = false;
  // if not provided , instantiate here
  } else {
    this->webSocket = new WebSocketsClient();
    this->_owns_webSocket = true;
  }

  // Handle socketstatus object
  if (status) {
    this->webSocketstatus = status;
    this->_owns_socketstatus = false;
  } else {
    this->webSocketstatus = new socketstatus();  // Create if not provided
    this->_owns_socketstatus = true;
  }

  // Debug: Verify pointers are valid
  Serial.print("[BPMobileConfig] Constructor: webSocket=0x");
  Serial.print((unsigned long)this->webSocket, HEX);
  Serial.print(", webSocketstatus=0x");
  Serial.println((unsigned long)this->webSocketstatus, HEX);

  // ensure callbacks default to null (header already set, but be explicit)
  _registration_cb = nullptr;
  _timesourceProvider_fn = nullptr;
  _client_name = "Unknown";
}

// destructor
BPMobileConfig::~BPMobileConfig()
{
  if (_owns_webSocket && this->webSocket) {
    delete this->webSocket;
    this->webSocket = nullptr;
  }
  if (_owns_socketstatus && this->webSocketstatus) {
    delete this->webSocketstatus;
    this->webSocketstatus = nullptr;
  }
}

// ============================================================================
// SERVER TIME SYNC IMPLEMENTATION (Standalone functions)
// ============================================================================

static uint64_t _lastServerTime = 0;

void BPMobile_requestServerTime(WebSocketsClient* ws) {
  if (ws == nullptr) return;

  // Send time request to server
  ws->sendTXT("{\"type\":\"time_request\"}");
  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println("[BPMobile] Requested server time");
    xSemaphoreGive(serialMutex);
  }
}

uint64_t BPMobile_parseServerTime(const char* payload) {
  // Simple parsing - look for "server_time" field
  // Expected format: {"type": "time_response", "server_time": 1736000000000}

  const char* timeKey = "\"server_time\":";
  const char* pos = strstr(payload, timeKey);
  if (pos == nullptr) return 0;

  pos += strlen(timeKey);
  // Skip whitespace
  while (*pos == ' ' || *pos == '\t') pos++;

  // Parse number
  uint64_t serverTime = strtoull(pos, nullptr, 10);

  if (serverTime > 1000000000000ULL) {  // Valid timestamp (after year 2001)
    _lastServerTime = serverTime;
    if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      Serial.printf("[BPMobile] Received server time: %llu\n", serverTime);
      xSemaphoreGive(serialMutex);
    }
  }

  return serverTime;
}

uint64_t BPMobile_getLastServerTime() {
  return _lastServerTime;
}
