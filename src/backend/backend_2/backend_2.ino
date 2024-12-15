#include "FastIMU.h"
#include "Madgwick.h"
#include <Wire.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <addons/RTDBHelper.h>

// GLOBAL VARS
#define IMU_ADDRESS 0x68
#define PERFORM_CALIBRATION //Comment to disable startup calibration

const char *ssid = "DIGI_eb21f8";
const char *password = "ddcc14de";

#define FIREBASE_HOST "https://projectiot-ee562-default-rtdb.europe-west1.firebasedatabase.app/"
#define FIREBASE_SECRET "UnJ5jFpZhqC2gRhkxKPBcZyhokwk5JFZmWeJ7kn3"

FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

MPU9250 IMU;

calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
MagData IMUMag;
Madgwick filter;
// !GLOBAL VARS

// PRIVATE FUNC
void displayQuat();
void publishQuat();
// !PRIVATE FUNC

void setup() {
  Wire.begin(14,15);
  Wire.setClock(400000); //400khz clock

  Serial.begin(115200);
  Serial.flush();

  while (!Serial) { ; }

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
  firebaseData.setBSSLBufferSize(4096 /* Rx buffer size: [ 512 - 16384 ] bytes */, 1024 /* Tx buffer size: [ 512 - 16384 ] bytes */);
  Firebase.begin(&config, &auth);
  // Firebase.begin(DATABASE_URL, DATABASE_SECRET); // LEGACY Connection

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

#ifdef PERFORM_CALIBRATION
  Serial.println("IMU Calibration started");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(1000);
  }
  Serial.println("Keep IMU leveled.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);

  filter.begin(0.2f);
#endif
}

void loop() {
  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);
  if (IMU.hasMagnetometer()) {
    IMU.getMag(&IMUMag);
    filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX, IMUMag.magY, IMUMag.magZ);
  }
  // else {
    // filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  // }
  displayQuat();
  publishQuat();
  delay(50);
}

void publishQuat() {
  if (!Firebase.RTDB.setFloat(&firebaseData, "/SensorData/QuatW", filter.getQuatW())) {
    Firebase.RTDB.setFloat(&firebaseData, "/SensorData/QuatW", 0);
  } 
    
  if (!Firebase.RTDB.setFloat(&firebaseData, "/SensorData/QuatX", filter.getQuatX())) {
    Firebase.RTDB.setFloat(&firebaseData, "/SensorData/QuatX", 0);
  } 

  if (!Firebase.RTDB.setFloat(&firebaseData, "/SensorData/QuatY", filter.getQuatY())) {
    Firebase.RTDB.setFloat(&firebaseData, "/SensorData/QuatY", 0);
  }

  if (!Firebase.RTDB.setFloat(&firebaseData, "/SensorData/QuatZ", filter.getQuatZ())) {
    Firebase.RTDB.setFloat(&firebaseData, "/SensorData/QuatZ", 0);
  }

  if (IMU.hasTemperature()) {
    if (!Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Temp", IMU.getTemp())) {
      Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Temp", 0);
    }
  }
}

void displayQuat() {
  Serial.print(filter.getQuatW());
  Serial.print(",");
  Serial.print(filter.getQuatX());
  Serial.print(",");
  Serial.print(filter.getQuatY());
  Serial.print(",");
  Serial.println(filter.getQuatZ());
}