#include <OneWire.h> 
#include <DallasTemperature.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

// Pin Definitions
#define ONE_WIRE_BUS 14       // DS18B20 sensor connected to D5 (GPIO4)
#define MOISTURE_PIN A0      // LM393 sensor connected to A0 (ADC0)

// WiFi and MQTT Configuration
const char* mqtt_server = "192.168.101.111"; // Change to your MQTT broker IP
const char* mqtt_topic = "farm/sensors";

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
  // Map the ADC value (0-1023) to a moisture percentage (0-100)
  int moisturePercentage = map(moistureLevel, 1023, 0, 100, 0);
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

  // Request temperature reading from DS18B20
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);  // Assuming only one sensor
  int moisture = readSoilMoisture();  // Get the soil moisture percentage

  // Send data to MQTT
  sendMQTTData(temperature, moisture);

  // Sleep for 2 minutes before the next reading
  ESP.deepSleep(2 * 60 * 1000000);  // Deep sleep for 2 minutes
   //delay(30000);
   yield();
}