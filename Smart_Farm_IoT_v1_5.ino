/*
Smart farm monitoring IoT device, to measure moisture and temperature and transmitting it using MQTT protocol,
which is fast and reliable and requires very little processing power and also power efficient.
 
Components: 
Microcontroller: ESP8266 Generic Module (bare module).
Temperature Sensor: DS18B20 sensor 
Soil Moisture Sensor: LM393 soil moisture sensor


Calibrated LM393 Soil Moisture Sensor Sketch
--------------------------------------------
Type of Soil:  Loamy soil (Note: recaliberate according to your soil type)
Uses real-world sensor calibration:
- Wet soil (660) = 100%
- Dry soil (922) = 0%

Sensor: LM393 Analog Moisture Sensor
Board: ESP8266 (NodeMCU, Wemos D1 Mini, etc.)
*/

#include <OneWire.h> 
#include <DallasTemperature.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

// Pin Definitions
#define ONE_WIRE_BUS 14       // DS18B20 sensor connected to D5 (GPIO4)
#define MOISTURE_PIN A0      // LM393 sensor connected to A0 (ADC0)
#define MOISTURE_ON_PIN 12  // GPIO Switching to power moisture sensor during reading D6 (GPIO12)


// Calibration values from your testing
const int sensorWet = 660;  // Reading in very wet soil
const int sensorDry = 922;  // Reading in very dry soil

// WiFi and MQTT Configuration
const char* mqtt_server = "192.168.101.111"; // Change to your MQTT broker IP
const char* mqtt_topic = "farm/sensors"; //change mqtt topi to what you wish

// Initialize the OneWire and DallasTemperature libraries
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// MQTT Client Setup
WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to Wi-Fi
void setup_wifi() {
  WiFiManager wifiManager;
  wifiManager.autoConnect("ESP8266_Farm");  // AP mode to configure Wi-Fi
  Serial.println("Connected to Wi-Fi");
}

// Function to read soil moisture level and convert to percentage
int readSoilMoisture() {
    int moistureLevel = analogRead(MOISTURE_PIN); // Read from LM393 analog pin

    // Map the raw moistureLevel to 0–100% (inverted: wet = high %, dry = low %)
    int moisturePercentage = map(moistureLevel, sensorDry, sensorWet, 0, 100);

    // Clamp the result to valid percentage range
    moisturePercentage = constrain(moisturePercentage, 0, 100);
    
    return moisturePercentage;
    yield();                  
     
}


// Function to send MQTT data
void sendMQTTData(float temperature, int moisture) {
  String payload = String("{\"temperature\":") + temperature + String(",\"moisture\":") + moisture + String("}");
  client.publish(mqtt_topic, payload.c_str());
  Serial.println("Data sent to MQTT: " + payload);
  yield();
}

// Function to reconnect to MQTT server
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP8266_Farm")) {
      Serial.println("Connected to MQTT broker");
    } else {
      delay(5000);
      yield();
      Serial.println("Reconnecting to MQTT...");
    }
  }
}

void setup() {
  pinMode(MOISTURE_ON_PIN, OUTPUT);
  digitalWrite(MOISTURE_ON_PIN, LOW); // Start with sensor OFF
  Timer.setInterval(5000); // Timer interval for sending sensor reading

  Serial.begin(115200);
  
  // Initialize the DS18B20 sensor
  sensors.begin();

  // Connect to Wi-Fi
  setup_wifi();

  // Initialize MQTT client
  client.setServer(mqtt_server, 1883);

  // Attempt to connect to MQTT
  reconnect();
}

void loop() {

  // Ensure we are connected to MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

   //Turn on the moisture sensor
      digitalWrite(MOISTURE_ON_PIN, HIGH);
      

  // Request temperature reading from DS18B20
   sensors.requestTemperatures();
   float temperature = sensors.getTempCByIndex(0);  // Assuming only one sensor
   int moisture = readSoilMoisture();  // Get the soil moisture percentage
   delay(1000);

   // Send data to MQTT
   sendMQTTData(temperature, moisture);

  // Sleep for 2 minutes before the next reading
  ESP.deepSleep(2 * 60 * 1000000);  // Deep sleep for 2 minutes
  yield();
}
