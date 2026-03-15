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

int rightSensor = 27;
int leftSensor = 14;
#define relay 17

const char* googleApiKey = "YOUR_API_KEY";
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
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

int in = 0;
int out = 0;
int now = 0;

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

void getLocation() 
{
  location_t loc = location.getGeoFromWiFi();

  WebSerial.println("Location request data");
  WebSerial.println ("Latitude: " + String(loc.lat, 7));
  WebSerial.println("Longitude: " + String(loc.lon, 7));

  if(loc.lat != 0 && loc.lon != 0)
  {
    sendLocation(loc.lat, loc.lon);
  }
}

void sendLocation(double lat, double lon)
{
  if (tb.sendTelemetryData("latitude", lat) && tb.sendTelemetryData("longitude", lon))
  {
    WebSerial.println("Location sent to ThingsBoard.");
  }
  else
  {
    WebSerial.println("Failed to send data.");
  }
}

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  pinMode(rightSensor, INPUT_PULLUP);
  pinMode(leftSensor, INPUT_PULLUP);
  pinMode(relay, OUTPUT);

  digitalWrite(relay, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

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
    WebSerial.println("ThingsBoard connection lost. Reconnecting.");
    connectToThingsBoard();
    return;
  }

  tb.loop();

  //checkEntrance();
  getLocation();
  
  delay(50);
}
