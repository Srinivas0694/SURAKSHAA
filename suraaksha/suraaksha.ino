#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include "MAX30105.h"
//#include "heartRate.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <BLE2902.h>


#define LED_BUILTIN 2
#define SENSOR_PIN 36
#define LORA_ON 33
#define BATTERY_PIN 39
#define SERVICE_UUID "0000180F-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "00002A19-0000-1000-8000-00805F9B34FB"
BLECharacteristic *pCharacteristic;
BLEServer *pServer;
bool deviceConnected = false;

Adafruit_AHTX0 aht;
MAX30105 particleSensor;

// Heart rate calculation variables
const byte RATE_SIZE = 4; // Buffer size for averaging
byte rates[RATE_SIZE];     // Array to store heart rate readings
byte rateSpot = 0;         // Index for the rate array
long lastBeat = 0;         // Time of the last detected beat
float beatsPerMinute = 0;  // Latest heart rate
int beatAvg = 0;           // Average heart rate

float spo2 = 0.0;          // SpO2 value
bool validSpO2 = false;    // Valid SpO2 flag

// Heartbeat detection threshold (adjust as needed)
const int HEARTBEAT_THRESHOLD = 50000; // Adjust this value based on your sensor readings

// *** SpO2 Calibration Constants (YOU MUST CALIBRATE THESE) ***
const float SPO2_A = 135.5;   // Adjusted constant A (calibrate based on your data)
const float SPO2_B = 15.0;    // Adjusted constant B (calibrate based on your data)
const float RATIO_MIN = 2.0;  // Minimum Red/IR Ratio (calibrate based on your data) (Adjust if necessary to reflect that finger is there and its reading value)
const float RATIO_MAX = 3.0;   // Maximum Red/IR Ratio (calibrate based on your data)

// OTA Configuration
const char* ssid = "Awfis-Temp";      // Replace with your WiFi credentials
const char* password = "Awfis#2025#";  // Replace with your WiFi credentials
bool otaEnabled = false;
bool updateInProgress = false;
const unsigned long OTA_WINDOW = 240000;    // 3 minutes OTA window (ms)
const unsigned long MAX_UPDATE_TIME = 300000; // 5 minutes max update time (ms)

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      pServer->getAdvertising()->start();
    }
};

void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();

  // Attempt WiFi connection for 20 seconds
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    delay(500);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED during connection
  }
  Serial.println("wifi connected");
  if (WiFi.status() == WL_CONNECTED) {
    otaEnabled = true;

    ArduinoOTA
    .onStart([]() {
      updateInProgress = true;
      digitalWrite(LED_BUILTIN, HIGH); // Solid LED during update
    })
    .onEnd([]() {
      updateInProgress = false;
      ESP.restart();
    })
    .onError([](ota_error_t error) {
      updateInProgress = false;
      digitalWrite(LED_BUILTIN, LOW);
    });

    ArduinoOTA.begin();
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LORA_ON, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  digitalWrite(LED_BUILTIN, LOW); // Start with LED off
  Serial.begin(115200);

  // Start OTA Update Window (3 minutes)
  setupOTA();
  unsigned long otaStartTime = millis();

  // OTA Handling Loop
  while (millis() - otaStartTime < OTA_WINDOW) {
    if (otaEnabled) {
      ArduinoOTA.handle();
      Serial.println("ota window open");
      // Blink LED when waiting for update
      static unsigned long lastBlink = 0;
      if (!updateInProgress && millis() - lastBlink > 500) {
        lastBlink = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
    }

    // Exit early if update completes
    if (updateInProgress && millis() - otaStartTime > MAX_UPDATE_TIME) break;
    delay(10);
  }

  // Cleanup OTA
  if (otaEnabled) {
    ArduinoOTA.end();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  digitalWrite(LED_BUILTIN, LOW); // Ensure LED is off after OTA
  Serial.println("ota window closed");

  // Proceed with normal setup
  Wire.begin();

  BLEDevice::init("Suraksha B3");
  Serial.println("line 1 done");
  pServer = BLEDevice::createServer();
  Serial.println("line 2 done");
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.println("line 3 done");
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("line 4 done");
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );

  // --- ADD THIS LINE ---
  pCharacteristic->addDescriptor(new BLE2902());
  // --------------------
  Serial.println("line 5 done");
  pService->start();
  Serial.println("line 6 done");
  pServer->getAdvertising()->start();
  Serial.println("line 7 done");

  // Initialize MAX30102 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0xFF);   // MAX power Red LED
  particleSensor.setPulseAmplitudeGreen(0);    // Turn off Green LED

  // Initialize AHT20
  if (!aht.begin()) {
    while (1);  // Halt if AHT20 initialization fails
  }
}

int readSmoothADC(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return sum / samples;
}

float getBatteryPercentage() {
  int rawValue = readSmoothADC(BATTERY_PIN, 30);

  // Constants for Min/Max Voltages (ADJUST THESE)
  const float MIN_VOLTAGE = 2.0; // Minimum battery voltage
  const float MAX_VOLTAGE = 4.0; // Maximum battery voltage
  float dividedVoltage = (rawValue / 4095.0) * 3.3; // ADC value to voltage *BEFORE* Divider
  float voltage = dividedVoltage * 2.0; //Scale for the voltage Divider
  float percentage = ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0; //Percentage Formula
  percentage = constrain(percentage, 0, 100);
  return percentage;
}

bool mycheckForBeat(long irValue) {
  static long lastIRValue = irValue;
  static bool lookingForPeak = true;
  bool beatDetected = false; // Flag to track beat detection

  if (lookingForPeak) {
    // Look for a rising edge above the threshold
    if (irValue > lastIRValue && irValue > HEARTBEAT_THRESHOLD) {
      lookingForPeak = false;
      beatDetected = true; // Beat detected
    }
  } else {
    // Look for a falling edge (after the peak)
    if (irValue < lastIRValue) {
      lookingForPeak = true; // Start looking for the next peak
    }
  }
  lastIRValue = irValue;

  return beatDetected; // Return whether a beat was detected
}

float calculateSpO2(long redValue, long irValue, bool *valid) {
  if (irValue <= 0 || redValue <= 0) {
    *valid = false;
    return 0.0;
  }

  float ratio = (float)redValue / irValue;
  float spo2 = SPO2_A - (SPO2_B * ratio);
  spo2 += 3;

  if (spo2 < 0) spo2 = 0;
  if (spo2 > 100) spo2 = 100;

  *valid = (ratio >= RATIO_MIN && ratio <= RATIO_MAX);
  return spo2;
}

void loop() {
  Serial.println("entering loop");
  if (!deviceConnected) return;

  digitalWrite(LORA_ON, HIGH);

  int sensorValue = readSmoothADC(SENSOR_PIN, 30);
  float voltage = (sensorValue / 4095.0) * 3.3;

  sensors_event_t humidityEvent, tempEvent;
  aht.getEvent(&humidityEvent, &tempEvent);
  float humidity = humidityEvent.relative_humidity;
  float temperature = tempEvent.temperature;

  // Get IR and Red values from MAX30102
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // Check for heartbeat
  bool beatDetected = mycheckForBeat(irValue);

  if (beatDetected) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute > 20 && beatsPerMinute < 180) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      // Calculate average heart rate
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;
    }
  }

  // Calculate SpO2
  spo2 = calculateSpO2(redValue, irValue, &validSpO2);

  // Create and send packets
  String packet1 = String(voltage, 2) + ", " + String(beatAvg) + ", " + String(validSpO2 ? spo2 : 0.0, 1);
  String packet2 = String(temperature, 2) + ", " + String(humidity, 2) + ", " + String(getBatteryPercentage(), 0);
  if (pCharacteristic != nullptr) {
    pCharacteristic->setValue(packet1.c_str());
    pCharacteristic->notify();
    delay(50);
    pCharacteristic->setValue(packet2.c_str());
    pCharacteristic->notify();
  }
  delay(100); // Overall loop delay
}
