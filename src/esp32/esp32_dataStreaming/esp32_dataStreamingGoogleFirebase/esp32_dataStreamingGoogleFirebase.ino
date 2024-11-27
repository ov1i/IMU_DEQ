#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <addons/RTDBHelper.h>
#include "MPU9250.h"

const char* ssid = "DIGI_eb21f8";
const char* password = "ddcc14de";

#define FIREBASE_HOST "https://projectiot-ee562-default-rtdb.europe-west1.firebasedatabase.app/"
#define FIREBASE_SECRET "UnJ5jFpZhqC2gRhkxKPBcZyhokwk5JFZmWeJ7kn3"   

FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

MPU9250 imu;

void setup() {
  Serial.begin(115200);
  Serial.flush();

  WiFi.begin(ssid, password);

  // Cloud api connection init
  Serial.print("Connecting to Wi-Fi");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to Wi-Fi with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_SECRET;

  Firebase.reconnectNetwork(true);
  firebaseData.setBSSLBufferSize(4096 /* Rx buffer size: [ 512 - 16384 ] bytes */ , 1024 /* Tx buffer size: [ 512 - 16384 ] bytes */);
  Firebase.begin(&config, &auth);
  // Firebase.begin(DATABASE_URL, DATABASE_SECRET); // LEGACY Connection
  

  // IMU init
  Wire.begin(14, 15);
  Serial.println("Initializing MPU9250...");
  imu.setup(0x68);
  delay(3000);

  // Calibrate accelerometer and gyroscope
  Serial.println("Calibration of GYRO and ACCELEROMETER started please DO NOT move the device...");
  // imu.calibrateAccelGyro();
  Serial.println("Calibration of the GYRO and ACCELEROMETER complete!");

  delay(3000);

  // Calibrate magnetometer
  Serial.println("Calibration of the magnetometer started please move the sensor in all directions...");
  // imu.calibrateMag();
  Serial.println("Magnetometer calibration complete!");


  Serial.println("Calibration offsets: ");
  Serial.print("Accel Bias X: "); Serial.println(imu.getAccBiasX());
  Serial.print("Accel Bias Y: "); Serial.println(imu.getAccBiasY());
  Serial.print("Accel Bias Z: "); Serial.println(imu.getAccBiasZ());

  Serial.print("Gyro Bias X: "); Serial.println(imu.getGyroBiasX());
  Serial.print("Gyro Bias Y: "); Serial.println(imu.getGyroBiasY());
  Serial.print("Gyro Bias Z: "); Serial.println(imu.getGyroBiasZ());

  Serial.print("Mag Bias X: "); Serial.println(imu.getMagBiasX());
  Serial.print("Mag Bias Y: "); Serial.println(imu.getMagBiasY());
  Serial.print("Mag Bias Z: "); Serial.println(imu.getMagBiasZ());
}

void loop() {
  float yaw, pitch, roll;

  if(imu.update()) {
    float yawData = imu.getYaw();    // In degrees
    float pitchData = imu.getPitch(); // In degrees
    float rollData = imu.getRoll();   // In degrees

    Serial.print("Yaw: "); Serial.print(yawData);
    Serial.print(" | Pitch: "); Serial.print(pitchData);
    Serial.print(" | Roll: "); Serial.println(rollData);
    
    if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Yaw", yawData)) {
      Serial.println("Yaw data sent to Firebase!");
    } else {
      Serial.print("Failed to send Yaw. Reason: ");
      Serial.println(firebaseData.errorReason());
    }

    if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Pitch", pitchData)) {
      Serial.println("Pitch data sent to Firebase!");
    } else {
      Serial.print("Failed to send Pitch. Reason: ");
      Serial.println(firebaseData.errorReason());
    }

    if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Roll", rollData)) {
      Serial.println("Roll data sent to Firebase!");
    } else {
      Serial.print("Failed to send Roll. Reason: ");
      Serial.println(firebaseData.errorReason());
    }
  }

  delay(1000);
}


