#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <time.h>

// MPU6050
Adafruit_MPU6050 mpu;

// WiFi Credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT Broker
const char* mqtt_server = "102.133.146.49";  // e.g. "192.168.1.100"
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/imu";
const char* mqtt_user = "thula";
const char* mqtt_pass = "irfsjy21";

// NTP Time Settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 2 * 3600;     // GMT+2
const int daylightOffset_sec = 0;

// Network clients
WiFiClient espClient;
PubSubClient client(espClient);

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ WiFi connected.");
}

void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32S3Client"), mqtt_user, mqtt_pass) {
      Serial.println(" connected.");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2s...");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // MPU6050 via I2C on GPIO8 (SDA) and GPIO9 (SCL)
  Wire.begin(8, 9);
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found. Check wiring.");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("✅ MPU6050 ready.");

  connectWiFi();

  // NTP Time Sync
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.print("Syncing time");
  while (time(nullptr) < 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n🕒 Time synced.");

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  connectMQTT();
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  // Read MPU6050
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Get timestamp
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  char timeStr[25];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%dT%H:%M:%S", &timeinfo);

  // Build JSON payload
  String payload = "{";
  payload += "\"timestamp\": \"" + String(timeStr) + "\", ";
  payload += "\"accel\": {\"x\": " + String(accel.acceleration.x, 2) +
             ", \"y\": " + String(accel.acceleration.y, 2) +
             ", \"z\": " + String(accel.acceleration.z, 2) + "}, ";
  payload += "\"gyro\": {\"x\": " + String(gyro.gyro.x, 2) +
             ", \"y\": " + String(gyro.gyro.y, 2) +
             ", \"z\": " + String(gyro.gyro.z, 2) + "}, ";
  payload += "\"temp\": " + String(temp.temperature, 2);
  payload += "}";

  Serial.println("📤 Sending:");
  Serial.println(payload);
  client.publish(mqtt_topic, payload.c_str());
  Serial.println("🎯 MPU6050 DATA:");
  Serial.print("Accel X: "); Serial.print(ax, 2); Serial.print(" m/s² | ");
  Serial.print("Y: "); Serial.print(ay, 2); Serial.print(" m/s² | ");
  Serial.print("Z: "); Serial.print(az, 2); Serial.println(" m/s²");

  Serial.print("Pitch: "); Serial.print(pitch, 2); Serial.print("° | ");
  Serial.print("Roll: "); Serial.print(roll, 2); Serial.println("°");

  Serial.println("--------------------------------------------------");
  delay(500);  // send every 500ms
}
