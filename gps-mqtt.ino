#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Update these with values suitable for your network.

static const int RXPin = D5, TXPin = D3;
static const uint32_t GPSBaud = 9600;

const char* ssid = "OnePlus5";
const char* password = "1234567890";
const char* mqtt_server = "broker.mqttdashboard.com";
const char* clientID = "guest";
const char* outTopic = "gpstest";
const char* inTopic = "gps";
char message_buff[100];

long lastMsg = 0;   
long lastRecu = 0;
bool debug = false;
// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long previousMillis = 0;        // will store last temp was read
const long interval = 2000;              // interval at which to read sensor
 
char msg[50];

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Conver the incoming byte array to a string
  payload[length] = '\0'; // Null terminator used to terminate the char array
  String message = (char*)payload;

  Serial.print("Message arrived on topic: [");
  Serial.print(topic);
  Serial.print("], ");
  Serial.println(message);

  if(message == "gps"){
    char str1[16];
    char str2[16];
    dtostrf(gps.location.lat(), 10, 6, str1);
    dtostrf(gps.location.lng(), 10, 6, str2);

    char str3[4]= ",";
    char str [160] = "https://www.google.com/maps/place/";
    strcat(str, str1);
    strcat(str, str3);
    strcat(str, str2);
    
    client.publish(outTopic, str, true);   
    
  } 

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientID)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(outTopic, clientID);
      // ... and resubscribe
      client.subscribe(inTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  ss.begin(GPSBaud);          // initialize gps sensor

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  while (ss.available() > 0)
   if (gps.encode(ss.read()))
     displayInfo();
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  { 
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }  
  Serial.println();
}
