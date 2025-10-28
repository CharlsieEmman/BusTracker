#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <WifiLocation.h>

const char* googleApiKey = "MY_API_KEY";
const char* ssid = "Charlsie";
const char* password = "kissmomunaako";

AsyncWebServer server(80);
WiFiLocation location (googleApiKey);

// Set time via NTP, as required for x.509 validation
void setClock () {
    configTime (0, 0, "pool.ntp.org", "time.nist.gov");

    WebSerial.print ("Waiting for NTP time sync: ");
    time_t now = time (nullptr);
    while (now < 8 * 3600 * 2) {
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

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  WebSerial.println("Connected to WiFi!");
  WebSerial.println(WiFi.localIP());

  while (WiFi.status() != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        // wait 5 seconds for connection:
        Serial.print("Status = ");
        Serial.println(WiFi.status());
        delay(500);
    }
    Serial.println ("Connected");
    setClock ();

  // Initialize WebSerial
  WebSerial.begin(&server);

  // Attach a callback function to handle incoming messages
  WebSerial.onMessage([](uint8_t *data, size_t len) {
    Serial.printf("Received %lu bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
    WebSerial.println("Received Data...");
    String d = "";
    for(size_t i = 0; i < len; i++){
      d += char(data[i]);
    }
    WebSerial.println(d);
  });

  // Start the server
  server.begin();

  // End ng Web Serial na need i-copy paste

  WebSerial.println("Connected to WiFi!");
  WebSerial.println(WiFi.localIP());
}

void loop() {
  location_t loc = location.getGeoFromWiFi();

  while(loc.lat != null || loc.lon != null){
    Serial.println("Location request data");
    Serial.println(location.getSurroundingWiFiJson()+"\n");
    Serial.println ("Latitude: " + String (loc.lat, 7));
    Serial.println("Longitude: " + String(loc.lon, 7));
    Serial.println ("Accuracy: " + String (loc.accuracy));
    Serial.println ("Result: " + location.wlStatusStr (location.getStatus ()));
  }
  
  delay(1000);
}