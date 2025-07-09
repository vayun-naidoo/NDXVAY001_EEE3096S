// #include <TinyGPS++.h>
// #include <HardwareSerial.h>

// #define RX 18
// #define TX 17

// // Create GPS and Serial port instance
// TinyGPSPlus gps;
// HardwareSerial gpsSerial(2);  // Use UART2

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) { delay(10); }  // Wait for USB Serial to initialize (important for native USB)
//   gpsSerial.begin(9600, SERIAL_8N1, RX, TX);  // RX, TX

//   Serial.println("GPS Module Test - ESP32S3");
// }

// void loop() {
//   while (gpsSerial.available()) {
//     gps.encode(gpsSerial.read());
//   }

//   if (gps.location.isUpdated()) {
//     Serial.print("Latitude: ");
//     Serial.println(gps.location.lat(), 6);
//     Serial.print("Longitude: ");
//     Serial.println(gps.location.lng(), 6);
//     Serial.print("Satellites: ");
//     Serial.println(gps.satellites.value());
//     Serial.print("Speed (km/h): ");
//     Serial.println(gps.speed.kmph());
//     Serial.println("-----------");
//   }
// }

#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  delay(1000);

  Serial.println("Scanning I2C devices...");

  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(address, HEX);
      count++;
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found.");
  }
}

void loop() {}

// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <math.h>

// Adafruit_MPU6050 mpu;

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(8, 9);  // SDA, SCL for ESP32-S3

//   if (!mpu.begin()) {
//     Serial.println("⚠️ MPU6050 not found. Check wiring!");
//     while (1) delay(10);
//   }

//   Serial.println("✅ MPU6050 connected.");
//   mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
//   mpu.setGyroRange(MPU6050_RANGE_250_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
// }

// void loop() {
//   sensors_event_t accel, gyro, temp;
//   mpu.getEvent(&accel, &gyro, &temp);

//   float ax = accel.acceleration.x;
//   float ay = accel.acceleration.y;
//   float az = accel.acceleration.z;

//   // Print raw acceleration
//   Serial.print("Accel X: "); Serial.print(ax, 2);
//   Serial.print(" m/s² | Y: "); Serial.print(ay, 2);
//   Serial.print(" m/s² | Z: "); Serial.print(az, 2);
//   Serial.println(" m/s²");

//   // Compute and print inclination
//   float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
//   float roll  = atan2(-ax, az) * 180.0 / PI;

//   Serial.print("Pitch: ");
//   Serial.print(pitch, 2);
//   Serial.print(" ° | Roll: ");
//   Serial.print(roll, 2);
//   Serial.println(" °");

//   Serial.println("-------------");
//   delay(500);
// }

// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <TinyGPS++.h>
// #include <HardwareSerial.h>
// #include <math.h>

// // MPU6050 setup
// Adafruit_MPU6050 mpu;

// // GPS setup
// // #define GPS_RX 18  // ESP32 receives from GPS
// // #define GPS_TX 17  // ESP32 transmits to GPS (often unused)
// HardwareSerial gpsSerial(2);
// TinyGPSPlus gps;

// void setup() {
//   Serial.begin(115200);
//   delay(500);

//   // === I2C for MPU6050 ===
//   Wire.begin(8, 9);  // SDA, SCL
//   if (!mpu.begin()) {
//     Serial.println("⚠️ MPU6050 not found. Check wiring!");
//     while (1) delay(10);
//   }
//   mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
//   mpu.setGyroRange(MPU6050_RANGE_250_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//   Serial.println("✅ MPU6050 ready.");

//   // === UART for GPS ===
//   // gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
//   // Serial.println("✅ GPS serial started.");
// }

// void loop() {
//   // === GPS: Feed characters to TinyGPS++ ===
//   // while (gpsSerial.available()) {
//   //   gps.encode(gpsSerial.read());
//   // }

//   // === MPU6050: Read sensor events ===
//   sensors_event_t accel, gyro, temp;
//   mpu.getEvent(&accel, &gyro, &temp);

//   float ax = accel.acceleration.x;
//   float ay = accel.acceleration.y;
//   float az = accel.acceleration.z;

//   float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
//   float roll  = atan2(-ax, az) * 180.0 / PI;

//   // // === Print GPS info, default to 0 if no update ===
//   // Serial.println("📡 GPS DATA:");
//   // if (gps.location.isUpdated()) {
//   //   Serial.print("Lat: ");
//   //   Serial.print(gps.location.lat(), 6);
//   //   Serial.print(" | Lon: ");
//   //   Serial.print(gps.location.lng(), 6);
//   //   Serial.print(" | Sats: ");
//   //   Serial.print(gps.satellites.value());
//   //   Serial.print(" | Speed: ");
//   //   Serial.print(gps.speed.kmph());
//   //   Serial.println(" km/h");
//   // } else {
//   //   Serial.println("Lat: 0.000000 | Lon: 0.000000 | Sats: 0 | Speed: 0.0 km/h");
//   // }

//   // === Print MPU data ===
//   Serial.println("🎯 MPU6050 DATA:");
//   Serial.print("Accel X: "); Serial.print(ax, 2); Serial.print(" m/s² | ");
//   Serial.print("Y: "); Serial.print(ay, 2); Serial.print(" m/s² | ");
//   Serial.print("Z: "); Serial.print(az, 2); Serial.println(" m/s²");

//   Serial.print("Pitch: "); Serial.print(pitch, 2); Serial.print("° | ");
//   Serial.print("Roll: "); Serial.print(roll, 2); Serial.println("°");

//   Serial.println("--------------------------------------------------");
//   delay(500);
// }




