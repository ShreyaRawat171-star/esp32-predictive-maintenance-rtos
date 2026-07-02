## ESP32 Predictive Maintenance System

> Real-time bearing fault detection using FreeRTOS, FFT, and MQTT on ESP32

## What This Does

This system detects industrial motor bearing faults BEFORE they cause failure.
It runs entirely on an ESP32 microcontroller with no cloud ML needed.

- Samples vibration at 800Hz using a hardware timer interrupt
- Runs FFT on 512 samples to extract frequency spectrum
- Classifies fault type from frequency peaks (on-device, no cloud)
- Publishes health score and fault alerts to ThingSpeak via HTTP

## System Architecture
SensorTask (Priority 3)

└── Collects 512 vibration samples at 800Hz via timer ISR

└── Sends batch to FFTTask via Queue 1
FFTTask (Priority 2)

└── Applies Hann window + runs arduinoFFT on 512 samples

└── Finds dominant frequency peak

└── Sends result to ClassifyTask via Queue 2
ClassifyTask (Priority 1)

└── 25Hz spike  = Motor Imbalance

└── 50Hz spike  = Misalignment

└── 87.5Hz spike = Bearing Outer Race Fault

└── No spike    = Healthy

└── Sends result to MQTTTask via Queue 3
MQTTTask (Priority 1)

└── Publishes to ThingSpeak every 15 seconds

└── Field 1 = Health Score (0-100)

└── Field 2 = Dominant Frequency Hz

└── Field 3 = Fault Type (0-3)

## Tech Stack

| Layer | Tool |
|---|---|
| Microcontroller | ESP32 Dev Module |
| RTOS | FreeRTOS (4 tasks, 3 queues, timer ISR) |
| Signal Processing | arduinoFFT v2.0.4 |
| Cloud | ThingSpeak HTTP |
| IDE | Arduino IDE 2.x |

## Fault Detection Logic

For a 1500 RPM motor (25Hz fundamental):

| Fault Type | Frequency | Health Score |
|---|---|---|
| Normal | ~25Hz low magnitude | 95% |
| Imbalance | 25Hz high magnitude | 40% |
| Misalignment | 50Hz | 50% |
| Bearing Outer Race Fault | 87.5Hz | 15% |

## How to Run

1. Install Arduino IDE 2.x
2. Install ESP32 board support from Espressif
3. Install arduinoFFT (v2.0.4) and PubSubClient libraries
4. Open predictive_maintenance_esp32.ino
5. Update WiFi name, password and ThingSpeak API key at the top
6. Select Tools → Board → ESP32 Dev Module
7. Hit Upload

## What I Learned

- FreeRTOS task scheduling and inter-task communication via queues
- Hardware timer interrupts for precise 800Hz sampling without drift
- FFT-based frequency domain analysis running on a microcontroller
- Edge computing: fault detection on-device, only alerts go to cloud
- MQTT/HTTP IoT data publishing to cloud dashboard
