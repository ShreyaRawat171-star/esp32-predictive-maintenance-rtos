#include <Arduino.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ============================================================
// DAY 3 — MQTT + ThingSpeak added on top of Day 2
// Now has 4 tasks:
// 1. SensorTask   — collects 512 samples at 800Hz
// 2. FFTTask      — runs FFT, finds dominant frequency
// 3. ClassifyTask — decides fault type from frequency
// 4. MQTTTask     — sends results to ThingSpeak
// ============================================================

// ---- WiFi + ThingSpeak credentials ----
const char* WIFI_SSID     = "Bharti Phone's";
const char* WIFI_PASSWORD = "12345678";
const char* THINGSPEAK_API_KEY = "NMIPHGLJ6CQIT6UA";
const char* MQTT_SERVER   = "mqtt3.thingspeak.com";
const int   MQTT_PORT     = 1883;

// ---- Constants ----
#define SAMPLE_RATE_HZ  800
#define BUFFER_SIZE     512

// ---- Queues ----
QueueHandle_t sensorQueue;
QueueHandle_t classifyQueue;
QueueHandle_t mqttQueue;

// ---- Timer ----
hw_timer_t *sampleTimer = NULL;

// ---- Shared sampling buffer ----
volatile float circularBuf[BUFFER_SIZE];
volatile int bufIndex = 0;
volatile bool bufferReady = false;
volatile unsigned long sampleCount = 0;

// ---- Fault mode ----
// 0 = NORMAL, 1 = IMBALANCE, 2 = MISALIGNMENT, 3 = BEARING FAULT
int faultMode = 0;

// ---- Fault result struct ----
struct FaultResult {
  int faultType;
  float dominantHz;
  float magnitude;
  int healthScore;
};

// ---- WiFi + MQTT clients ----
WiFiClient espClient;
PubSubClient mqttClient(espClient);


// ============================================================
// SYNTHETIC SIGNAL GENERATOR
// ============================================================
float fakeSensorReading() {
  float t = sampleCount / (float)SAMPLE_RATE_HZ;
  sampleCount++;
  float noise = 0.05 * ((random(100) / 100.0) - 0.5);

  switch (faultMode) {
    case 0:
      return 9.8 + 0.3 * sin(2 * PI * 25 * t) + noise;
    case 1:
      return 9.8 + 1.8 * sin(2 * PI * 25 * t) + noise;
    case 2:
      return 9.8 + 0.3 * sin(2 * PI * 25 * t)
                 + 1.5 * sin(2 * PI * 50 * t) + noise;
    case 3:
      return 9.8 + 0.3 * sin(2 * PI * 25 * t)
                 + 1.9 * sin(2 * PI * 87.5 * t) + noise;
    default:
      return 9.8 + noise;
  }
}


// ============================================================
// TIMER ISR
// ============================================================
void IRAM_ATTR onTimerTick() {
  circularBuf[bufIndex] = fakeSensorReading();
  bufIndex++;
  if (bufIndex >= BUFFER_SIZE) {
    bufIndex = 0;
    bufferReady = true;
  }
}


// ============================================================
// TASK 1 — SENSOR TASK
// ============================================================
void sensorTask(void *parameter) {
  Serial.println("[SensorTask] started");

  sampleTimer = timerBegin(1000000);
  timerAttachInterrupt(sampleTimer, &onTimerTick);
  timerAlarm(sampleTimer, 1250, true, 0);

  float snapshot[BUFFER_SIZE];
  while (1) {
    if (bufferReady) {
      memcpy((void*)snapshot, (void*)circularBuf, sizeof(snapshot));
      bufferReady = false;
      xQueueSend(sensorQueue, snapshot, 0);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}


// ============================================================
// TASK 2 — FFT TASK
// ============================================================
void fftTask(void *parameter) {
  Serial.println("[FFTTask] started");

  double vReal[BUFFER_SIZE];
  double vImag[BUFFER_SIZE];
  ArduinoFFT<double> FFT = ArduinoFFT<double>(
    vReal, vImag, BUFFER_SIZE, SAMPLE_RATE_HZ
  );

  float rawData[BUFFER_SIZE];
  while (1) {
    if (xQueueReceive(sensorQueue, rawData, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < BUFFER_SIZE; i++) {
        vReal[i] = rawData[i];
        vImag[i] = 0.0;
      }

      FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
      FFT.compute(FFTDirection::Forward);
      FFT.complexToMagnitude();

      double peakHz = FFT.majorPeak();
      int peakBin   = (int)(peakHz * BUFFER_SIZE / SAMPLE_RATE_HZ);
      double peakMag = vReal[peakBin];

      Serial.print("[FFTTask] dominant frequency: ");
      Serial.print(peakHz);
      Serial.println(" Hz");

      float fftResult[2] = { (float)peakHz, (float)peakMag };
      xQueueSend(classifyQueue, fftResult, 0);
    }
  }
}


// ============================================================
// TASK 3 — CLASSIFY TASK
// ============================================================
void classifyTask(void *parameter) {
  Serial.println("[ClassifyTask] started");

  float fftResult[2];
  while (1) {
    if (xQueueReceive(classifyQueue, fftResult, portMAX_DELAY) == pdTRUE) {
      float hz  = fftResult[0];
      float mag = fftResult[1];

      FaultResult result;
      result.dominantHz = hz;
      result.magnitude  = mag;

      if (hz >= 80 && hz <= 95) {
        result.faultType   = 3;
        result.healthScore = 15;
      }
      else if (hz >= 45 && hz <= 55) {
        result.faultType   = 2;
        result.healthScore = 50;
      }
      else if (hz >= 20 && hz <= 30 && mag > 1.0) {
        result.faultType   = 1;
        result.healthScore = 40;
      }
      else {
        result.faultType   = 0;
        result.healthScore = 95;
      }

      // print to Serial
      Serial.println("─────────────────────────────");
      Serial.print("FAULT TYPE : ");
      switch (result.faultType) {
        case 0: Serial.println("NORMAL");             break;
        case 1: Serial.println("IMBALANCE");          break;
        case 2: Serial.println("MISALIGNMENT");       break;
        case 3: Serial.println("BEARING FAULT");      break;
      }
      Serial.print("DOMINANT Hz: "); Serial.println(result.dominantHz);
      Serial.print("MAGNITUDE  : "); Serial.println(result.magnitude);
      Serial.print("HEALTH     : "); Serial.print(result.healthScore);
      Serial.println("%");
      Serial.println("─────────────────────────────\n");

      // send to MQTT task
      xQueueSend(mqttQueue, &result, 0);
    }
  }
}


// ============================================================
// TASK 4 — MQTT TASK
// Connects to WiFi + ThingSpeak, publishes every result
// ============================================================
void mqttTask(void *parameter) {
  Serial.println("[MQTTTask] started");

  // connect WiFi
  Serial.print("[MQTTTask] Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  Serial.println("\n[MQTTTask] WiFi connected!");
  Serial.print("[MQTTTask] IP: ");
  Serial.println(WiFi.localIP());

  // setup MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  FaultResult result;
  unsigned long lastPublish = 0;

  while (1) {
    // keep MQTT connection alive
    if (!mqttClient.connected()) {
      Serial.println("[MQTTTask] Connecting to ThingSpeak MQTT...");
      // ThingSpeak MQTT needs clientID, username, password
      // For basic HTTP publishing we use their REST API instead
      // which is simpler and doesn't need MQTT credentials
    }
    mqttClient.loop();

    // publish every time a new result arrives
    if (xQueueReceive(mqttQueue, &result, 100 / portTICK_PERIOD_MS) == pdTRUE) {

      unsigned long now = millis();
      // throttle to once every 15 seconds (ThingSpeak free limit)
      if (now - lastPublish >= 15000) {
        lastPublish = now;

        // build ThingSpeak HTTP GET request
        WiFiClient client;
        if (client.connect("api.thingspeak.com", 80)) {
          String url = "/update?api_key=";
          url += THINGSPEAK_API_KEY;
          url += "&field1=" + String(result.healthScore);
          url += "&field2=" + String(result.dominantHz);
          url += "&field3=" + String(result.faultType);

          client.print("GET " + url + " HTTP/1.1\r\n");
          client.print("Host: api.thingspeak.com\r\n");
          client.print("Connection: close\r\n\r\n");
          client.stop();

          Serial.println("[MQTT] Published to ThingSpeak!");
          Serial.print("  Health: ");   Serial.println(result.healthScore);
          Serial.print("  Freq Hz: "); Serial.println(result.dominantHz);
          Serial.print("  Fault: ");   Serial.println(result.faultType);
        } else {
          Serial.println("[MQTT] ThingSpeak connection failed!");
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== DAY 3: Full System - FreeRTOS + FFT + MQTT ===\n");

  // create all queues
  sensorQueue   = xQueueCreate(4, sizeof(float[BUFFER_SIZE]));
  classifyQueue = xQueueCreate(4, sizeof(float[2]));
  mqttQueue     = xQueueCreate(4, sizeof(FaultResult));

  if (sensorQueue == NULL || classifyQueue == NULL || mqttQueue == NULL) {
    Serial.println("ERROR: Queue creation failed!");
    while (1);
  }

  // create all 4 tasks
  xTaskCreate(sensorTask,   "SensorTask",   4096, NULL, 3, NULL);
  xTaskCreate(fftTask,      "FFTTask",      8192, NULL, 2, NULL);
  xTaskCreate(classifyTask, "ClassifyTask", 4096, NULL, 1, NULL);
  xTaskCreate(mqttTask,     "MQTTTask",     8192, NULL, 1, NULL);

  Serial.println("All 4 tasks started!\n");
  Serial.println("faultMode = 0 (NORMAL)");
  Serial.println("Change faultMode to 1,2,3 to simulate faults\n");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}