// ======================= User Configuration =======================
// Please enter your WiFi and ThingsBoard credentials here.
// You can get your device access token from your device on ThingsBoard.
constexpr char WIFI_SSID[] = "HUAWEI-YpND";
constexpr char WIFI_PASSWORD[] = "Tataycruz";
constexpr char TOKEN[] = "C3dvKW5eHmxvJKbXoSVO";

// ThingsBoard server details
constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

// ======================= Library Includes =======================
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <WiFi.h>
#include <ThingsBoard.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// ======================= Sensor and Pin Definitions =======================
#define GPS_RX_PIN 2 // Replace the number with the RX Pin
#define GPS_TX_PIN 3 // Replace the number with the TX Pin
HardwareSerial gpsSerial(2);

#define rightSensor 14 // Replace the number with the right sensor pin (people boarding)
#define leftSensor 12 // Replace the number with the left sensor pin (people unboarding)
#define relay 0

// ======================= Global Variables =======================
int peopleCount = 0;
unsigned long firstTriggerTime = 0;
String firstTriggered = "";
const unsigned long maxInterval = 1000;
unsigned long resetDelayStartTime = 0;
const unsigned long resetDelay = 200;

// GPS object
TinyGPSPlus gps;

// ThingsBoard client setup
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient);

// Interval for sending telemetry data to ThingsBoard
constexpr uint32_t telemetrySendInterval = 5000U;
unsigned long lastTelemetrySendTime = 0;

// ======================= Function Prototypes =======================
void InitWiFi();
const bool reconnect();
void resetSequence();
void sendTelemetry();

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  pinMode(rightSensor, INPUT);
  pinMode(leftSensor, INPUT);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);

  InitWiFi();
}

void loop() {
  // Check WiFi and ThingsBoard connection
  if (!reconnect()) {
    return;
  }
  if (!tb.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect to ThingsBoard.");
    } else {
      Serial.println("Connected.");
    }
  }

  // Process GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Sensor reading logic
  int rightState = digitalRead(rightSensor);
  int leftState = digitalRead(leftSensor);

  // Non-blocking sensor reset logic
  if (resetDelayStartTime > 0 && millis() - resetDelayStartTime >= resetDelay) {
    firstTriggered = "";
    firstTriggerTime = 0;
    resetDelayStartTime = 0;
  }

  // Main sensor logic
  if (firstTriggered == "") {
    if (rightState == LOW) {
      firstTriggered = "RIGHT";
      firstTriggerTime = millis();
    } else if (leftState == LOW) {
      firstTriggered = "LEFT";
      firstTriggerTime = millis();
    }
  } else {
    if (millis() - firstTriggerTime <= maxInterval) {
      if (firstTriggered == "RIGHT" && leftState == LOW) {
        peopleCount++;
        Serial.println("Enter detected. Count: " + String(peopleCount));
        resetSequence();
      } else if (firstTriggered == "LEFT" && rightState == LOW) {
        peopleCount--;
        if (peopleCount < 0) peopleCount = 0;
        Serial.println("Exit detected. Count: " + String(peopleCount));
        resetSequence();
      }
    } else {
      Serial.println("Sequence timeout, ignoring.");
      resetSequence();
    }
  }

  // Control relay based on people count
  digitalWrite(relay, peopleCount > 0 ? LOW : HIGH);

  // Send data to ThingsBoard at a set interval
  if (millis() - lastTelemetrySendTime > telemetrySendInterval) {
    sendTelemetry();
    lastTelemetrySendTime = millis();
  }

  // Process ThingsBoard MQTT messages
  tb.loop();
  
  delay(10);
}

// Function to initialize WiFi connection
void InitWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi.");
}

// Function to reconnect if WiFi connection is lost
const bool reconnect() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  InitWiFi();
  return true;
}

// Resets the sensor state in a non-blocking way
void resetSequence() {
  resetDelayStartTime = millis();
}

// Sends sensor and GPS data to ThingsBoard
void sendTelemetry() {
  // Send people count
  tb.sendTelemetryData("peopleCount", peopleCount);

  // Send GPS data if it's valid
  if (gps.location.isValid()) {
    tb.sendTelemetryData("latitude", gps.location.lat());
    tb.sendTelemetryData("longitude", gps.location.lng());
    tb.sendTelemetryData("satellites", gps.satellites.value());
    Serial.println("Sending GPS data to ThingsBoard.");
  } else {
    Serial.println("GPS data not valid, skipping.");
  }
}