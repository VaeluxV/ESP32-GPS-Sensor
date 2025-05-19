#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>

#include <secrets.h> // Include passwords, SSID and other info

// === Device Information ===
const char* sensor_id = "gps_car_2-abc-123";
const char* device_type = "GPS-multi-tracker";
const char* firmware_version = "v2025.05.19c";
const char* connection_type = "Wi-Fi";
const char* driver_id = "driver_002";

WiFiClient espClient;
PubSubClient client(espClient);

// === GPS Setup ===
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use Serial1
const int GPS_RX = 16;        // ESP32 RX <- GPS TX
const int GPS_TX = 17;        // ESP32 TX -> GPS RX

// === Timing ===
unsigned long lastPublish = 0;
unsigned long startMillis;
const int publish_interval = 5000; // 5 seconds

// === Wi-Fi Connection ===
void setup_wifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected. IP: " + WiFi.localIP().toString());
}

// === MQTT Connection ===
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      delay(100);  // Give MQTT time to settle before publishing
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
  int light_level = 0;

  JsonDocument doc;

  if (gps.location.isValid()) {
    JsonObject location = doc.createNestedObject("location");
    location["lat"] = gps.location.lat();
    location["lon"] = gps.location.lng();
    doc["speed"] = gps.speed.kmph();
    doc["estimated_altitude"] = gps.altitude.meters();
    doc["gnss_satellites_count"] = gps.satellites.value();
    doc["error_code"] = 0;
    doc["status"] = "active_normal";
  } else if (gps.satellites.value() == 0) {
    doc["gnss_satellites_count"] = 0;
    doc["error_code"] = 7;
    doc["status"] = "active_no_gps_satellites";
  } else if (WiFi.RSSI() < -69) {
    doc["gnss_satellites_count"] = gps.satellites.value();
    doc["error_code"] = 3;
    doc["status"] = "active_degraded_signal";
  } else {
    doc["gnss_satellites_count"] = gps.satellites.value();
    doc["error_code"] = 8;
    doc["status"] = "active_no_gps_signal";
  }

  doc["light_level"] = light_level;
  doc["sensor_id"] = sensor_id;
  doc["device_type"] = device_type;
  doc["signal_strength"] = WiFi.RSSI();
  doc["firmware_version"] = firmware_version;
  doc["connection_type"] = connection_type;
  doc["uptime_seconds"] = (millis() - startMillis) / 1000;
  doc["battery_level"] = 100;
  doc["critical_battery"] = false;
  doc["driver_id"] = driver_id;

  String jsonStr;
  serializeJson(doc, jsonStr);  // Serialize directly into a String

  Serial.println("Topic: gpstracker001/data");
  Serial.printf("Publishing JSON length: %u\n", jsonStr.length());
  Serial.println(jsonStr);

  // Publish using the String's c_str()
  bool ok = client.publish("gpstracker001/data", jsonStr.c_str());

  Serial.println(ok ? "Publish success" : "Publish FAILED!");

  // // Publish a test string for comparison - debugging only
  // bool ok2 = client.publish("test/esp", "hello from esp32");
  // Serial.println(ok2 ? "Published test message" : "Publish test FAILED!");
}


void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(1024);
  startMillis = millis();
}

void loop() {
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (millis() - lastPublish > publish_interval) {
    lastPublish = millis();
    publish_gps_data();
  }

  delay(5);
}
