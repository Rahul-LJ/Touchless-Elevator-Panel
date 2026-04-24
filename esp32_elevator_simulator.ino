/*
 * ESP32 Elevator Simulator - Final Unified Version
 * ====================================================
 * Whisper STT API + Anti-Conflict Hover Sensor Logic.
 * Sensor detection window strictly set to 1cm - 5cm.
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ==================== CONFIGURATION ====================
const char* WIFI_SSID       = "potatoserver";
const char* WIFI_PASSWORD   = "12345678";

// Ensure this IP matches your laptop's current hotspot IP!
const char* SPEECH_API_URL  = "http://172.17.228.77:8080/api/speech";
const char* COMMAND_API_URL = "http://172.17.228.77:8080/api/command";

// ==================== PIN DEFINITIONS ====================
#define SENSOR1_TRIG  32
#define SENSOR1_ECHO  35   
#define SENSOR2_TRIG  17
#define SENSOR2_ECHO  16
#define SENSOR3_TRIG  27
#define SENSOR3_ECHO  14

#define I2S_WS   25
#define I2S_SCK  26
#define I2S_SD   33

#define LED_FLOOR1  5
#define LED_FLOOR2  12
#define LED_FLOOR3  13
#define LED_UP      19
#define LED_DOWN    18
#define LED_STATUS  2

// ==================== CONSTANTS ====================
// Fixed Sensor Window as requested
#define SENSOR_MIN_DIST_CM        1
#define SENSOR_MAX_DIST_CM        5
#define HOVER_TIME             1000   // 1 sec hover to trigger
#define CONFLICT_BLINK_TIME    2000   // 2s warning blink

#define FLOOR_TRAVEL_TIME_MS   2000
#define FLOOR_DOOR_TIME_MS     3000
#define API_TIMEOUT_MS        30000
#define WIFI_RECONNECT_MS     10000

// I2S Configuration
#define I2S_PORT           I2S_NUM_0
#define I2S_BUFFER_SAMPLES      256
#define AUDIO_SAMPLE_RATE     16000
#define AUDIO_CAPTURE_SEC         2
#define AUDIO_BUF_SAMPLES  (AUDIO_SAMPLE_RATE * AUDIO_CAPTURE_SEC)

#define WAKE_ENERGY_THRESH      800  
#define WAKE_MIN_MS             150   
#define WAKE_MAX_MS            1000   
#define WAKE_GAP_MS             200   
#define WAKE_COOLDOWN_MS       2000   

#define CMD_SILENCE_MS         1500   
#define CMD_MIN_MS              200   
#define CMD_MAX_MS             4000   
#define VAD_FRAME_SAMPLES       160   

// ==================== ENUMS ====================
enum LiftState  { LIFT_IDLE, LIFT_MOVING_UP, LIFT_MOVING_DOWN, LIFT_ARRIVED };
enum Direction  { DIRECTION_NONE, DIRECTION_UP, DIRECTION_DOWN };

// ==================== GLOBALS ====================
volatile LiftState currentLiftState = LIFT_IDLE;
volatile int       currentFloor     = 1;
volatile Direction currentDirection = DIRECTION_NONE;

QueueHandle_t      floorQueue;
bool               activeRequests[3] = {false, false, false}; // LED tracking

// Advanced Sensor Globals 
bool conflictActive = false;
unsigned long conflictEnd = 0;
int activeSensor = -1;
unsigned long hoverStart = 0;
bool sensorLocked[3] = {false, false, false};

// System Globals
bool     isRecording      = false;
int16_t* audioBuffer      = nullptr;
size_t   audioBufferCount = 0;
bool          wifiConnected = false;
unsigned long lastWifiCheck = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  ESP32 Elevator - Final Unified v5.0  ");
  Serial.println("========================================");

  floorQueue = xQueueCreate(10, sizeof(int));
  
  size_t bufBytes = AUDIO_BUF_SAMPLES * sizeof(int16_t);
  if (psramFound()) { audioBuffer = (int16_t*)ps_malloc(bufBytes); } 
  else { audioBuffer = (int16_t*)malloc(bufBytes); }
  
  if (!audioBuffer) {
    Serial.println("FATAL: audio buffer alloc failed!");
    while (true) delay(1000);
  }

  // Explicit pin assignment to prevent hardware mapping bugs
  pinMode(SENSOR1_TRIG, OUTPUT); pinMode(SENSOR1_ECHO, INPUT);
  pinMode(SENSOR2_TRIG, OUTPUT); pinMode(SENSOR2_ECHO, INPUT);
  pinMode(SENSOR3_TRIG, OUTPUT); pinMode(SENSOR3_ECHO, INPUT);

  int allLeds[] = {LED_FLOOR1, LED_FLOOR2, LED_FLOOR3, LED_UP, LED_DOWN, LED_STATUS};
  for (int pin : allLeds) { pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); }
  delay(500);
  for (int pin : allLeds) { digitalWrite(pin, LOW); }
  
  currentFloor = 1;
  currentLiftState = LIFT_IDLE;

  // I2S Setup
  i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = AUDIO_SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 8,
    .dma_buf_len          = I2S_BUFFER_SAMPLES,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK, .ws_io_num = I2S_WS, .data_out_num = I2S_PIN_NO_CHANGE, .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);

  // WiFi Setup
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 15) {
    delay(500); Serial.print("."); attempts++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  }

  // Create Tasks
  xTaskCreatePinnedToCore(audioTask,  "AudioTask",  8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(liftTask,   "LiftTask",   4096, NULL, 3, NULL, 0);
  
  Serial.println("Tasks started. Say 'arise' to activate mic.\n");
}

// ==================== ADVANCED SENSOR LOGIC ====================
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 30000);
  if (dur == 0) return 999;
  return (int)(dur * 0.034 / 2.0);
}

void processFloorRequest(int floor) {
  if (floor < 1 || floor > 3) return;
  if (floor == currentFloor && currentLiftState == LIFT_IDLE) return;
  if (activeRequests[floor - 1]) return; 

  activeRequests[floor - 1] = true;
  int f = floor;
  xQueueSend(floorQueue, &f, portMAX_DELAY);
  Serial.printf("[LIFT] Floor %d request queued\n", floor);
}

void readUltrasonicSensors() {
  if (conflictActive) {
    if (millis() >= conflictEnd) {
      conflictActive = false;
      Serial.println("Conflict cleared, ready again");
    }
    return; 
  }

  const int trigs[3] = {SENSOR1_TRIG, SENSOR2_TRIG, SENSOR3_TRIG};
  const int echos[3] = {SENSOR1_ECHO, SENSOR2_ECHO, SENSOR3_ECHO};
  int distances[3];
  int activeCount = 0;

  for (int i = 0; i < 3; i++) {
    distances[i] = getDistance(trigs[i], echos[i]);
    // Uses the strictly requested 1 to 5cm window
    if (distances[i] >= SENSOR_MIN_DIST_CM && distances[i] <= SENSOR_MAX_DIST_CM) {
      activeCount++;
    }
  }

  if (activeCount > 1) {
    Serial.println("⚠️ Conflict: Multiple sensors detected! Rejecting input.");
    conflictActive = true;
    conflictEnd = millis() + CONFLICT_BLINK_TIME;
    return;
  }

  for (int i = 0; i < 3; i++) {
    if (sensorLocked[i] && (distances[i] < SENSOR_MIN_DIST_CM || distances[i] > SENSOR_MAX_DIST_CM)) {
      sensorLocked[i] = false;
    }
  }

  int closest = -1;
  int minDist = 999;
  for (int i = 0; i < 3; i++) {
    bool detected = (distances[i] >= SENSOR_MIN_DIST_CM && distances[i] <= SENSOR_MAX_DIST_CM);
    if (!sensorLocked[i] && detected && distances[i] < minDist) {
      minDist = distances[i];
      closest = i;
    }
  }

  if (closest == -1) {
    activeSensor = -1;
    hoverStart = 0;
    return;
  }

  if (activeSensor != closest) {
    activeSensor = closest;
    hoverStart = millis();
  }

  if (millis() - hoverStart >= HOVER_TIME) {
    int targetFloor = closest + 1;
    sensorLocked[closest] = true;
    Serial.printf("✅ Floor %d selected (ultrasonic) - Distance: %d cm\n", targetFloor, minDist);
    processFloorRequest(targetFloor);
  }
}

// ==================== LIFT & LED LOGIC ====================
void liftStateMachine() {
  static unsigned long stateStartTime = 0;
  static int targetFloor = 0;

  switch (currentLiftState) {
    case LIFT_IDLE:
      if (uxQueueMessagesWaiting(floorQueue) > 0) {
        xQueueReceive(floorQueue, &targetFloor, portMAX_DELAY);
        if (targetFloor > currentFloor) {
          currentLiftState = LIFT_MOVING_UP; 
          currentDirection = DIRECTION_UP;
        } else if (targetFloor < currentFloor) {
          currentLiftState = LIFT_MOVING_DOWN;
          currentDirection = DIRECTION_DOWN;
        }
        stateStartTime = millis();
      }
      break;

    case LIFT_MOVING_UP:
    case LIFT_MOVING_DOWN:
      if (millis() - stateStartTime >= FLOOR_TRAVEL_TIME_MS) {
        currentFloor += (currentLiftState == LIFT_MOVING_UP) ? 1 : -1;
        
        if (currentFloor == targetFloor) {
          currentLiftState = LIFT_ARRIVED; 
          currentDirection = DIRECTION_NONE;
          activeRequests[currentFloor - 1] = false; 
          Serial.printf("[LIFT] Arrived at Floor %d\n", currentFloor);
        }
        stateStartTime = millis();
      }
      break;

    case LIFT_ARRIVED:
      if (millis() - stateStartTime >= FLOOR_DOOR_TIME_MS) {
        currentLiftState = LIFT_IDLE;
      }
      break;
  }
}

void updateLiftLEDs() {
  unsigned long now = millis();

  // Status LED: Fast blink on conflict, solid when recording/wifi
  if (conflictActive) {
    static unsigned long lastConflictBlink = 0;
    if (now - lastConflictBlink >= 50) {
      lastConflictBlink = now;
      digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
    }
  } else {
    digitalWrite(LED_STATUS, isRecording ? HIGH : (wifiConnected ? HIGH : LOW));
  }

  // Floor LEDs: On if we are at the floor OR it is requested
  digitalWrite(LED_FLOOR1, (currentFloor == 1 || activeRequests[0]) ? HIGH : LOW);
  digitalWrite(LED_FLOOR2, (currentFloor == 2 || activeRequests[1]) ? HIGH : LOW);
  digitalWrite(LED_FLOOR3, (currentFloor == 3 || activeRequests[2]) ? HIGH : LOW);

  // Direction LEDs: Blink when moving
  static bool dirBlinkState = false;
  static unsigned long lastDirBlink = 0;
  
  if (now - lastDirBlink >= 150) {
     dirBlinkState = !dirBlinkState;
     lastDirBlink = now;
  }

  if (currentLiftState == LIFT_IDLE || currentLiftState == LIFT_ARRIVED) {
     digitalWrite(LED_UP, LOW); digitalWrite(LED_DOWN, LOW);
  } else if (currentLiftState == LIFT_MOVING_UP) {
     digitalWrite(LED_UP, dirBlinkState); digitalWrite(LED_DOWN, LOW);
  } else if (currentLiftState == LIFT_MOVING_DOWN) {
     digitalWrite(LED_UP, LOW); digitalWrite(LED_DOWN, dirBlinkState);
  }
}

// ==================== VOICE & HTTP ====================
void processVoiceCommand(const char* text) {
  String cmd = String(text);
  cmd.toLowerCase();
  
  // Send data to python server
  HTTPClient http;
  http.begin(COMMAND_API_URL);
  http.addHeader("Content-Type", "application/json");
  StaticJsonDocument<256> doc;
  
  if (cmd.indexOf("floor 1") >= 0 || cmd.indexOf("first") >= 0 || cmd.indexOf("ground") >= 0) {
    processFloorRequest(1); doc["command"] = "floor_1";
  } else if (cmd.indexOf("floor 2") >= 0 || cmd.indexOf("second") >= 0) {
    processFloorRequest(2); doc["command"] = "floor_2";
  } else if (cmd.indexOf("floor 3") >= 0 || cmd.indexOf("third") >= 0 || cmd.indexOf("top") >= 0) {
    processFloorRequest(3); doc["command"] = "floor_3";
  }
  
  doc["currentFloor"] = currentFloor;
  String body; serializeJson(doc, body);
  if (wifiConnected) http.POST(body);
}

void parseSpeechResponse(const String& response) {
  StaticJsonDocument<512> doc;
  if (!deserializeJson(doc, response)) {
    const char* status = doc["status"];
    const char* text   = doc["text"];
    if (status && strcmp(status, "success") == 0 && text && strlen(text) > 0) {
      Serial.printf("\n>>> Whisper heard: \"%s\" <<<\n\n", text);
      processVoiceCommand(text);
    }
  }
}

bool sendAudioToServer() {
  if (!wifiConnected) return false;
  uint32_t dataSize = audioBufferCount * sizeof(int16_t);
  uint32_t fileSize = dataSize + 36;
  size_t totalSize = 44 + dataSize;
  uint8_t* wavData = (uint8_t*)(psramFound() ? ps_malloc(totalSize) : malloc(totalSize));
  
  uint8_t hdr[44] = {
    'R','I','F','F', (uint8_t)fileSize,(uint8_t)(fileSize>>8),(uint8_t)(fileSize>>16),(uint8_t)(fileSize>>24),
    'W','A','V','E','f','m','t',' ', 16,0,0,0, 1,0, 1,0,
    (uint8_t)AUDIO_SAMPLE_RATE,(uint8_t)(AUDIO_SAMPLE_RATE>>8),0,0,
    (uint8_t)(AUDIO_SAMPLE_RATE*2),(uint8_t)((AUDIO_SAMPLE_RATE*2)>>8),0,0, 2,0, 16,0,
    'd','a','t','a', (uint8_t)dataSize,(uint8_t)(dataSize>>8),(uint8_t)(dataSize>>16),(uint8_t)(dataSize>>24)
  };
  memcpy(wavData, hdr, 44);
  memcpy(wavData + 44, audioBuffer, dataSize);

  HTTPClient http;
  http.begin(SPEECH_API_URL);
  http.addHeader("Content-Type", "audio/wav");
  int code = http.POST(wavData, totalSize);
  free(wavData);
  if (code > 0) { parseSpeechResponse(http.getString()); return true; }
  return false;
}

// ==================== FREERTOS TASKS ====================
void audioTask(void* parameter) {
  Serial.println("[MIC] Audio task started - say 'arise' to activate");
  enum AudioState { IDLE, WAKE_HOLD, RECORDING, SENDING };
  AudioState state = IDLE;
  int16_t frame[VAD_FRAME_SAMPLES];
  
  unsigned long burstStartMs   = 0;
  unsigned long lastVoicedMs   = 0;
  unsigned long recordStartMs  = 0;
  unsigned long lastWakeMs     = 0;
  unsigned long lastPrintMs    = 0;
  unsigned long lastRecPrintMs = 0;

  while (1) {
    size_t bytesRead = 0;
    // Proper 20ms FreeRTOS tick delay to prevent task starvation
    i2s_read(I2S_PORT, frame, VAD_FRAME_SAMPLES * sizeof(int16_t), &bytesRead, 20 / portTICK_PERIOD_MS);
    int n = bytesRead / sizeof(int16_t);
    if (n <= 0) { vTaskDelay(5 / portTICK_PERIOD_MS); continue; }

    int64_t sum = 0;
    for (int i = 0; i < n; i++) sum += (int64_t)frame[i] * frame[i];
    int32_t rms = (int32_t)sqrt((double)sum / n);
    bool voiced = (rms > WAKE_ENERGY_THRESH);
    unsigned long now = millis();

    switch (state) {
      case IDLE:
        // Print energy bar every 500ms so you can see mic is alive
        if (now - lastPrintMs > 500) {
          lastPrintMs = now;
          int bars = min((int)(rms / 80), 25);
          char bar[27] = {0};
          for (int b = 0; b < bars; b++) bar[b] = '|';
          const char* hint = (rms > WAKE_ENERGY_THRESH * 0.5 && rms < WAKE_ENERGY_THRESH) ? " <- louder" : "";
          Serial.printf("[MIC] Listening  rms=%-5d [%-25s]%s\n", rms, bar, hint);
        }
        
        if (voiced && (now - lastWakeMs > WAKE_COOLDOWN_MS)) {
          burstStartMs = lastVoicedMs = now; state = WAKE_HOLD;
        }
        break;

      case WAKE_HOLD:
        if (voiced) lastVoicedMs = now;
        if (now - lastVoicedMs > WAKE_GAP_MS) {
          unsigned long burstLen = lastVoicedMs - burstStartMs;
          if (burstLen >= WAKE_MIN_MS && burstLen <= WAKE_MAX_MS) {
            Serial.println("\n[MIC] 'arise' detected! Speak command...");
            audioBufferCount = 0; isRecording = true;
            recordStartMs = lastVoicedMs = lastWakeMs = lastRecPrintMs = now;
            state = RECORDING;
          } else { state = IDLE; }
        }
        break;

      case RECORDING:
        {
          int canStore = min((size_t)n, (size_t)(AUDIO_BUF_SAMPLES - audioBufferCount));
          if (canStore > 0) {
            memcpy(audioBuffer + audioBufferCount, frame, canStore * sizeof(int16_t));
            audioBufferCount += canStore;
          }
          if (voiced) lastVoicedMs = now;
          unsigned long elapsed = now - recordStartMs;
          
          // Print recording progress
          if (now - lastRecPrintMs > 400) {
            lastRecPrintMs = now;
            int bars = min((int)(rms / 80), 25);
            char bar[27] = {0};
            for (int b = 0; b < bars; b++) bar[b] = '|';
            Serial.printf("[MIC] Recording %4lums  rms=%-5d [%-25s]\n", elapsed, rms, bar);
          }

          if ((now - lastVoicedMs >= CMD_SILENCE_MS && elapsed >= CMD_MIN_MS) || 
              elapsed >= CMD_MAX_MS || audioBufferCount >= AUDIO_BUF_SAMPLES - 1) {
            isRecording = false;
            Serial.println("[MIC] Sending to Whisper...");
            state = SENDING;
          }
        }
        break;

      case SENDING:
        if (audioBufferCount > (size_t)(AUDIO_SAMPLE_RATE * CMD_MIN_MS / 1000)) sendAudioToServer();
        audioBufferCount = 0;
        state = IDLE;
        break;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void sensorTask(void* parameter) {
  while (1) {
    readUltrasonicSensors();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void liftTask(void* parameter) {
  while (1) {
    liftStateMachine();
    updateLiftLEDs();
    vTaskDelay(20 / portTICK_PERIOD_MS); 
  }
}

// ==================== MAIN LOOP (FIXED WIFI LOGIC) ====================
void loop() {
  if (millis() - lastWifiCheck >= WIFI_RECONNECT_MS) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost - reconnecting...");
      wifiConnected = false;
      
      // Explicitly disconnect and reset mode before begin() to prevent crash
      WiFi.disconnect(true); 
      delay(500);
      WiFi.mode(WIFI_STA);   
      delay(100);
      
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 10) {
        delay(500); 
        Serial.print("."); 
        attempts++;
      }
      Serial.println();

      if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.printf("WiFi back! IP: %s\n", WiFi.localIP().toString().c_str());
      } else {
        Serial.println("Reconnect failed - will retry");
      }
    }
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
}