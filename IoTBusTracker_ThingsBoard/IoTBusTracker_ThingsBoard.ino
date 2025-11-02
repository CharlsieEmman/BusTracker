// WiFi and WebSerial
#include <Wire.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

// GPS
#include <WifiLocation.h>

// ThingsBoard
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>
#include <ThingsBoard.h>

#define rightSensor 1
#define leftSensor 2
#define relay 0    

const char* googleApiKey = "YOUR_GOOGLE_API_KEY";
const char* ssid = "SSID";
const char* password = "PASSWORD";
const char* token = "YOUR_THINGSBOARD_TOKEN";

// ThingsBoard Server Connection
constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr size_t MAX_ATTRIBUTES = 3U;
constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 5000U * 1000U;

// MQTT Client Instance
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);

// API Initialization
Server_Side_RPC<3U, 5U> rpc;
Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;
Shared_Attribute_Update<3U, MAX_ATTRIBUTES> shared_update;

const std::array<IAPI_Implementation*, 3U> apis = {
  &rpc,
  &attr_request,
  &shared_update
};

// ThingsBoard Instance Initialization
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, Default_Max_Stack_Size);

// Telemetry Initialization
constexpr uint32_t telemetrySendInterval = 2000U;
unsigned long lastTelemetrySendTime = 0;
uint32_t previousDataSend;

AsyncWebServer server(80);
WifiLocation location (googleApiKey);

int inStatus;
int outStatus;

int countin = 0;
int countout = 0;

int in;
int out;
int now;

// Set time via NTP, as required for x.509 validation
void setClock () 
{
    configTime (0, 0, "pool.ntp.org", "time.nist.gov");

    WebSerial.print ("Waiting for NTP time sync: ");
    time_t now = time (nullptr);
    while (now < 8 * 3600 * 2) 
    {
        delay (500);
        Serial.print (".");
        now = time (nullptr);
    }
    struct tm timeinfo;
    gmtime_r (&now, &timeinfo);
    WebSerial.print ("\n");
    WebSerial.print ("Current time: ");
    WebSerial.print (asctime (&timeinfo));
}

void connectToThingsBoard()
{
  if (!tb.connected())
  {
    WebSerial.println("Connecting to ThingsBoard server");
  }

  if (!tb.connect(THINGSBOARD_SERVER, token))
  {
    WebSerial.println("Failed to connect to ThingsBoard");
  }
  else 
  {
    WebSerial.println("Connected to ThingsBoard");
  }
}

void sendLocation(double lat, double lon)
{
  tb.sendTelemetryData("latitude", lat);
  tb.sendTelemetryData("longitude", lon);
  WebSerial.println("Location sent to ThingsBoard.");
}

void sendPassengerCount(int now)
{
  tb.sendTelemetryData("peopleCount", now);
  WebSerial.println("Passenger count sent to ThingsBoard.");
}

void getLocation() 
{
  location_t loc = location.getGeoFromWiFi();

  WebSerial.println("Location request data");
  WebSerial.println ("Latitude: " + String(loc.lat, 7));
  WebSerial.println("Longitude: " + String(loc.lon, 7));

  sendLocation(loc.lat, loc.lon);
}

void checkEntrance()
{
  inStatus = digitalRead(rightSensor);
  outStatus = digitalRead(leftSensor);

  if (inStatus == 0)
  {
    in = countin++;
  }
 
  if (outStatus == 0)
  {
    out = countout++;
  }
 
  now = in - out;
 
  if (now <= 0)
  {
    digitalWrite(relay, HIGH);
    WebSerial.println("No Visitors! Light Off");
    sendPassengerCount(0);

    delay(500);
  }
  else
  {
    WebSerial.print("Current Passengers: ");
    WebSerial.println(now);
    WebSerial.print("IN: ");
    WebSerial.println(in);
    WebSerial.print("OUT: ");
    WebSerial.println(out);
    sendPassengerCount(now);

    delay(500);
  }
}

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  pinMode(rightSensor, INPUT);
  pinMode(leftSensor, INPUT);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);

  // Initialize WebSerial
  WebSerial.begin(&server);

  // Attach a callback function to handle incoming messages
  WebSerial.onMessage([](uint8_t *data, size_t len) 
  {
    Serial.printf("Received %lu bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
    WebSerial.println("Received Data...");
    String d = "";
    for(size_t i = 0; i < len; i++)
    {
      d += char(data[i]);
    }
    WebSerial.println(d);
  });

  // Start the server
  server.begin();

  // End ng Web Serial na need i-copy paste

  WebSerial.println("Connected to WiFi!");
  WebSerial.println(WiFi.localIP());
  connectToThingsBoard();

  setClock ();
  delay(2000);
}

void loop() 
{
  if(!tb.connected())
  {
    connectToThingsBoard();
  }
  
  getLocation();
  checkEntrance();
  
  delay(100);
}
