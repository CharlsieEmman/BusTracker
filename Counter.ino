#include <Wire.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <SoftwareSerial.h>

const char* ssid = "ssid";
const char* password = "password";

AsyncWebServer server(80);

int rightSensor = 27;
int leftSensor = 14;
#define relay 17

int inStatus;
int outStatus;

int countin = 0;
int countout = 0;

const unsigned long timeout = 50;

int in;
int out;
int now;

void setup() {
  Serial.begin(115200);

  pinMode(rightSensor, INPUT);
  pinMode(leftSensor, INPUT);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  //pinMode(4, OUTPUT);
  //digitalWrite(4, LOW);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

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

  delay(3000);
}

void loop() {
  int rightStatus = digitalRead(rightSensor); 
  int leftStatus = digitalRead(leftSensor); 

  // --- RAW READING DEBUG ---
  WebSerial.print("R/L Raw: "); 
  WebSerial.print(rightStatus);
  WebSerial.print(" / "); 
  WebSerial.println(leftStatus); 
  countPass1();
  delay(50);
}

void countPass1()
{
  // Passenger Counter
  inStatus = digitalRead(rightSensor);
  outStatus = digitalRead(leftSensor);

  if (digitalRead(rightSensor) == 1)
  {
    in = countin++;
  }
 
  if (digitalRead(leftSensor) == 1)
  {
    out = countout++;
  }
 
  now = in - out;
 
  if (now <= 0)
  {
    digitalWrite(relay, HIGH);
    WebSerial.println("No Visitors! Light Off");
    delay(500);
  }
  else
  {
    WebSerial.print("Current Visitor: ");
    WebSerial.println(now);
    WebSerial.print("IN: ");
    WebSerial.println(in);
    WebSerial.print("OUT: ");
    WebSerial.println(out);
    delay(500);
  }
}

void countPass2() 
{
  // Passenger Counter Ver. 2
  if (digitalRead(rightSensor) == HIGH) {
    unsigned long startTime = millis();
    while ((millis() - startTime) < timeout) {
      if(digitalRead(leftSensor) == HIGH){
         ++in;
         updateDisplay();
         break;
      }
    }
    //wait until both sensors return to a normal state
    while(!digitalRead(rightSensor) || !digitalRead(leftSensor));
  }

  
  // Check if the second sensor is triggered 

  else if (digitalRead(leftSensor) == HIGH) {
    unsigned long startTime = millis();
    while ((millis() - startTime) < timeout) {
      if(digitalRead(rightSensor) == HIGH){
        if(out < in){
           ++out;
           updateDisplay();
           break;
        }
      }
    }
    //wait until both sensors return to a normal state
    while(!digitalRead(rightSensor) || !digitalRead(leftSensor));
  }
}

void countPass3()
{

}

void updateDisplay() 
{
  now = in - out;
  if (now <= 0)
  {
    digitalWrite(relay, HIGH);
    WebSerial.println("No Visitors! Light Off");
    //delay(500);
  }
  else
  {
    WebSerial.print("Current Visitor: ");
    WebSerial.println(now);
    WebSerial.print("IN: ");
    WebSerial.println(in);
    WebSerial.print("OUT: ");
    WebSerial.println(out);
    //delay(500);
  }
}
