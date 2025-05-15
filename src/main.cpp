#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <Adafruit_AHTX0.h>

#include <secrets.h> // Include passwords, SSID and other info from secrets.h

// === Device Information ===
const char* sensor_id = "gps_car_2-abc-123";
const char* device_type = "GPS-multi-tracker";
const char* firmware_version = "v2025.05.05a";
const char* connection_type = "Wi-Fi";
const char* driver_id = "driver_002";

WiFiClient espClient;
PubSubClient client(espClient);

// === GPS Setup ===
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use Serial1
const int GPS_RX = 16;        // ESP32 RX <- GPS TX
const int GPS_TX = 17;        // ESP32 TX -> GPS RX

// === AHT25 Sensor ===
Adafruit_AHTX0 aht;

// === LDR ===
const int LDR_PIN = 34; // ADC1 channel 6 (works while Wi-Fi is enabled/in use)

// === Timing ===
unsigned long lastPublish = 0;
unsigned long startMillis;
const int publish_interval = 5000; // 5 seconds

// === Setup Wi-Fi connection ===
void setup_wifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected. IP: " + WiFi.localIP().toString());
}

// === MQTT (re-)connection function ===
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5s");
      delay(5000);
    }
  }
}

// === Read sensors and publish data ===
void publish_gps_data() {
  if (!gps.location.isValid()) {
    Serial.println("No valid GPS data!");
    return;
  }

  // === Read sensors ===
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  int light_level = analogRead(LDR_PIN);

  JsonDocument doc;
  JsonObject location = doc["location"].to<JsonObject>();
  location["lat"] = gps.location.lat();
  location["lon"] = gps.location.lng();

  doc["speed"] = gps.speed.kmph();
  doc["light_level"] = light_level;
  doc["temperature"] = temp.temperature;
  doc["humidity"] = humidity.relative_humidity;
  doc["sensor_id"] = sensor_id;
  doc["device_type"] = device_type;
  doc["signal_strength"] = WiFi.RSSI();
  doc["estimated_altitude"] = gps.altitude.meters();
  doc["noise_level"] = 0; // placeholder
  doc["firmware_version"] = firmware_version;
  doc["connection_type"] = connection_type;
  doc["uptime_seconds"] = (millis() - startMillis) / 1000;
  doc["gnss_satellites_count"] = gps.satellites.value();
  doc["battery_level"] = 100; // placeholder
  doc["critical_battery"] = false; // placeholder
  doc["status"] = "active_normal"; // placeholder until made dynamic
  doc["driver_id"] = driver_id;
  doc["error_code"] = 0;

  char buffer[512];
  size_t len = serializeJson(doc, buffer);
  client.publish("gpstracker001/data", buffer, len);
  Serial.println("Published JSON:");
  Serial.println(buffer);
}

void setup() {
  Serial.begin(115200); // Main serial
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // GPS serial
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  startMillis = millis();
}

void loop() {
  // check Wi-Fi connection and connect to MQTT broker if not connected already
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  // Read GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Publish every 5 seconds
  if (millis() - lastPublish > publish_interval) {
    lastPublish = millis();
    publish_gps_data();
  }
}
