#include "mpu9250.h"
#include "Wire.h"
#include <WiFi.h>
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <addons/RTDBHelper.h>
#include <MadgwickAHRS.h>

// DATA TYPES
typedef struct {
  float gyroRawX, gyroRawY, gyroRawZ;
  float accRawX, accRawY, accRawZ;
  float magRawX, magRawY, magRawZ;
  float rawSensTemp;

  bool dataAvailability = false;
} imuRawData;

typedef struct {
  float yaw = 0, pitch = 0, roll = 0;
  float sensTemp;

  bool dataAvailibility = false;
} procOrientationData;

typedef struct {
  float PredictionConfidence[2][2] = { { 1, 0 }, { 0, 1 } };  // Confidence in predicted values for YPR
  float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
} kalmanParams;

typedef struct {
  float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
  float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
  float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;
  float magScaleX = 1, magScaleY = 1, magScaleZ = 1;
} imuCalibOffsets;
// !DATA TYPES

// GLOBAL VARS
bfs::Mpu9250 imu;
float yawBiasVal = 0, pitchBiasVal = 0, rollBiasVal = 0;
float dt = 0;
kalmanParams filterParamsRoll, filterParamsPitch, filterParamsYaw;
imuRawData rawDataBuffer;
procOrientationData processedDataBuffer;
imuCalibOffsets imuOffsets;
unsigned long prevTimestamp = 0;

const char *ssid = "DIGI_eb21f8";
const char *password = "ddcc14de";

#define FIREBASE_HOST "https://projectiot-ee562-default-rtdb.europe-west1.firebasedatabase.app/"
#define FIREBASE_SECRET "UnJ5jFpZhqC2gRhkxKPBcZyhokwk5JFZmWeJ7kn3"

FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

Madgwick filter;
// !GLOBAL VARS

// PRIVATE FUNC
void fillRawDataBuffer();
void transfRaw2YPR_unfiltered();
void transfRaw2YPR_filtered();
void transf_6ax_Raw2YPR_filtered();
void kalmanFilter(kalmanParams &filterParams, float measeuredAngle, float &estimatedAngle, float &bias);
void calibrateAccelGyro();
void calibrateMagnetometer();
void publishData();
// !PRIVATE FUNC

void setup() {
  Serial.begin(115200);
  Serial.flush();
  while (!Serial) {;}

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

  Wire.begin(14, 15);
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while (1) {}
  }

  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured Sample Rate Divider");  // We want as many samples as possible(or readings from the sensor): sample = 1kHz / (19 + 1)
    while (1) {}
  }

  // delay(2000);
  // Calibrate IMU
  // calibrateAccelGyro();
  // calibrateMagnetometer();
  // delay(2000);

  filter.begin(100.0);

  delay(3000);
}

void loop() {
  unsigned long currentTimestamp = millis();
  dt = (currentTimestamp - prevTimestamp) / 1000.0;
  prevTimestamp = currentTimestamp;

  fillRawDataBuffer();
  // Serial.print("Raw Data:");
  // Serial.print("\nACC(X,Y,Z): \t");  Serial.print(rawDataBuffer.accRawX); Serial.print("\t");  Serial.print(rawDataBuffer.accRawY); Serial.print("\t");  Serial.print(rawDataBuffer.accRawZ);
  // Serial.print("\nMAG(X,Y,Z): \t");  Serial.print(rawDataBuffer.magRawX); Serial.print("\t");  Serial.print(rawDataBuffer.magRawY); Serial.print("\t");  Serial.print(rawDataBuffer.magRawZ);
  // Serial.print("\nGyro(X,Y,Z): \t"); Serial.print(rawDataBuffer.gyroRawX); Serial.print("\t"); Serial.print(rawDataBuffer.gyroRawY); Serial.print("\t"); Serial.print(rawDataBuffer.gyroRawZ);

  // transfRaw2YPR_unfiltered();
  // Serial.print("\nYPR Data(unfiltered):");
  // Serial.print("\nYPR_UNFILTERED(Y,P,R): \t"); Serial.print(processedDataBuffer.yaw); Serial.print("\t"); Serial.print(processedDataBuffer.pitch); Serial.print("\t"); Serial.print(processedDataBuffer.roll);

  // transfRaw2YPR_filtered();
  // Serial.print("\nYPR Data(filtered):");
  // Serial.print("\nYPR_FILTERED(Y,P,R): \t"); Serial.print(processedDataBuffer.yaw * RAD_TO_DEG); Serial.print("\t"); Serial.print(processedDataBuffer.pitch * RAD_TO_DEG); Serial.print("\t"); Serial.print(processedDataBuffer.roll * RAD_TO_DEG);

  transf_6ax_Raw2YPR_filtered();
  // Serial.print("\nYPR Data(6-axis filtered):");
  Serial.print("\nYPR_6-axis_FILTERED(Y,P,R): \t"); Serial.print(processedDataBuffer.yaw); Serial.print("\t"); Serial.print(processedDataBuffer.pitch); Serial.print("\t"); Serial.print(processedDataBuffer.roll);

  Serial.print("\n");

  // publishData();
}


void fillRawDataBuffer() {
  bool readFlag = false;
  if ((readFlag = imu.Read())) {
    // Store locally gyro raw data in m/s
    rawDataBuffer.gyroRawX = imu.gyro_x_radps() - imuOffsets.gyroOffsetX;
    rawDataBuffer.gyroRawY = imu.gyro_y_radps() - imuOffsets.gyroOffsetY;
    rawDataBuffer.gyroRawZ = imu.gyro_z_radps() - imuOffsets.gyroOffsetZ;

    // Store locally accelerometer raw data in rad/s
    rawDataBuffer.accRawX = imu.accel_x_mps2() - imuOffsets.accOffsetX;
    rawDataBuffer.accRawY = imu.accel_y_mps2() - imuOffsets.accOffsetY;
    rawDataBuffer.accRawZ = imu.accel_z_mps2() - imuOffsets.accOffsetZ;

    // Store locally magnetometer raw data uT (micro teslas -> magnetic field meas units)
    rawDataBuffer.magRawX = (imu.mag_x_ut() - imuOffsets.magOffsetX) / imuOffsets.magScaleX;
    rawDataBuffer.magRawY = (imu.mag_y_ut() - imuOffsets.magOffsetY) / imuOffsets.magScaleY;
    rawDataBuffer.magRawZ = (imu.mag_z_ut() - imuOffsets.magOffsetZ) / imuOffsets.magScaleZ;

    // Store locally sensor temperature data in C
    rawDataBuffer.rawSensTemp = imu.die_temp_c();

    // Update flag for data availability from sensor
    rawDataBuffer.dataAvailability = readFlag;
  }
}
void transfRaw2YPR_unfiltered() {
  if (rawDataBuffer.dataAvailability) {

    // Copy temp to output data type
    processedDataBuffer.sensTemp = rawDataBuffer.rawSensTemp;

    // Automatically converted from RAD->DEGREE
    processedDataBuffer.roll = atan2(rawDataBuffer.accRawY, rawDataBuffer.accRawZ) * RAD_TO_DEG;
    processedDataBuffer.pitch = atan2(-rawDataBuffer.accRawX, sqrt(rawDataBuffer.accRawY * rawDataBuffer.accRawY + rawDataBuffer.accRawZ * rawDataBuffer.accRawZ)) * RAD_TO_DEG;

    // Correct mag readings
    float magX_int = rawDataBuffer.magRawX * cos(processedDataBuffer.pitch * DEG_TO_RAD) + rawDataBuffer.magRawZ * sin(processedDataBuffer.pitch * DEG_TO_RAD);
    float magY_int = rawDataBuffer.magRawX * sin(processedDataBuffer.roll * DEG_TO_RAD) * sin(processedDataBuffer.pitch * DEG_TO_RAD) + rawDataBuffer.magRawY * cos(processedDataBuffer.roll * DEG_TO_RAD) - rawDataBuffer.magRawZ * sin(processedDataBuffer.roll * DEG_TO_RAD) * cos(processedDataBuffer.pitch * DEG_TO_RAD);

    // Compute yaw from the corrected mag readings
    float yaw_int = atan2(magY_int, magX_int) * RAD_TO_DEG;

    processedDataBuffer.yaw = yaw_int;

    processedDataBuffer.dataAvailibility = true;
  }
}

void transfRaw2YPR_filtered() {
  if (rawDataBuffer.dataAvailability) {
    // Copy temp to output data type
    processedDataBuffer.sensTemp = rawDataBuffer.rawSensTemp;

    // Automatically converted from RAD->DEGREE
    float roll_int = atan2f(rawDataBuffer.accRawY, rawDataBuffer.accRawZ);
    float pitch_int = atan2f(-rawDataBuffer.accRawX, sqrt(rawDataBuffer.accRawY * rawDataBuffer.accRawY + rawDataBuffer.accRawZ * rawDataBuffer.accRawZ));

    // Correct mag readings
    float magX_int = rawDataBuffer.magRawX * cos(pitch_int) + rawDataBuffer.magRawZ * sin(pitch_int);
    float magY_int = rawDataBuffer.magRawX * sin(roll_int) * sin(pitch_int) + rawDataBuffer.magRawY * cos(roll_int) - rawDataBuffer.magRawZ * sin(roll_int) * cos(pitch_int);

    // Compute yaw from the corrected mag readings
    float yaw_int = atan2f(magY_int, magX_int);

    // Read gyro data + it's time drift
    processedDataBuffer.roll += (rawDataBuffer.gyroRawX - rollBiasVal) * dt;
    processedDataBuffer.pitch += (rawDataBuffer.gyroRawY - pitchBiasVal) * dt;
    processedDataBuffer.yaw += (rawDataBuffer.gyroRawZ - yawBiasVal) * dt;
    // processedDataBuffer.yaw += rawDataBuffer.gyroRawZ * dt;

    // Filter and combine all 9 axis of the IMU
    kalmanFilter(filterParamsRoll, roll_int, processedDataBuffer.roll, rollBiasVal);
    kalmanFilter(filterParamsPitch, pitch_int, processedDataBuffer.pitch, pitchBiasVal);
    kalmanFilter(filterParamsYaw, yaw_int, processedDataBuffer.yaw, yawBiasVal);

    // Complementary filter for yaw
    // processedDataBuffer.yaw = 0.98 * (processedDataBuffer.yaw) + 0.02 * yaw_int;


    processedDataBuffer.dataAvailibility = true;
  }
}

void transf_6ax_Raw2YPR_filtered() {
  if (rawDataBuffer.dataAvailability) {
    // Copy temp to output data type
    processedDataBuffer.sensTemp = rawDataBuffer.rawSensTemp;

    filter.update(rawDataBuffer.gyroRawX, rawDataBuffer.gyroRawY, rawDataBuffer.gyroRawZ, rawDataBuffer.accRawX, rawDataBuffer.accRawY, rawDataBuffer.accRawZ, rawDataBuffer.magRawX, rawDataBuffer.magRawY, rawDataBuffer.magRawZ);

    processedDataBuffer.roll = filter.getRoll();
    processedDataBuffer.pitch = filter.getPitch();
    processedDataBuffer.yaw = filter.getYaw();

    processedDataBuffer.dataAvailibility = true;
  }
}

void kalmanFilter(kalmanParams &filterParams, float measeuredAngle, float &estimatedAngle, float &bias) {
  float K_gain[2];

  // Prediction of the desired angle
  filterParams.PredictionConfidence[0][0] += dt * (dt * filterParams.PredictionConfidence[1][1] - filterParams.PredictionConfidence[0][1] - filterParams.PredictionConfidence[1][0] + filterParams.Q_angle);
  filterParams.PredictionConfidence[0][1] -= dt * filterParams.PredictionConfidence[1][1];
  filterParams.PredictionConfidence[1][0] -= dt * filterParams.PredictionConfidence[1][1];
  filterParams.PredictionConfidence[1][1] += filterParams.Q_bias * dt;

  // Update the step of the filter
  float S = filterParams.PredictionConfidence[0][0] + filterParams.R_measure;
  K_gain[0] = filterParams.PredictionConfidence[0][0] / S;
  K_gain[1] = filterParams.PredictionConfidence[1][0] / S;

  float y = measeuredAngle - estimatedAngle;
  estimatedAngle += K_gain[0] * y;
  bias += K_gain[1] * y;

  // Update the covaraiance
  filterParams.PredictionConfidence[0][0] -= K_gain[0] * filterParams.PredictionConfidence[0][0];
  filterParams.PredictionConfidence[0][1] -= K_gain[0] * filterParams.PredictionConfidence[0][1];
  filterParams.PredictionConfidence[1][0] -= K_gain[1] * filterParams.PredictionConfidence[0][0];
  filterParams.PredictionConfidence[1][1] -= K_gain[1] * filterParams.PredictionConfidence[0][1];
}

void calibrateAccelGyro() {
  Serial.println("Calibration of the accelerometer and gyroscope will begin in 3s...");
  delay(3000);
  float accelSum[3] = { 0, 0, 0 };
  float gyroSum[3] = { 0, 0, 0 };
  float samples = 15000;

  for (int i = 0; i < samples; i++) {
    if (imu.Read()) {
      accelSum[0] += imu.accel_x_mps2();
      accelSum[1] += imu.accel_y_mps2();
      accelSum[2] += imu.accel_z_mps2();
      gyroSum[0] += imu.gyro_x_radps();
      gyroSum[1] += imu.gyro_y_radps();
      gyroSum[2] += imu.gyro_z_radps();
    }
    // delay(5);
  }

  imuOffsets.accOffsetX = accelSum[0] / samples;
  imuOffsets.accOffsetY = accelSum[1] / samples;
  imuOffsets.accOffsetZ = accelSum[2] / samples;

  imuOffsets.gyroOffsetX = gyroSum[0] / samples;
  imuOffsets.gyroOffsetY = gyroSum[1] / samples;
  imuOffsets.gyroOffsetZ = gyroSum[2] / samples;

  // Adapt bias for static velocity (assuming flat calibration surface)
  imuOffsets.accOffsetZ -= 9.81;  // Gravity in m/s^2

  Serial.println("Accelerometer and Gyroscope calibration complete.");
  Serial.println("\nAccel Bias: ");
  Serial.print(imuOffsets.accOffsetX);
  Serial.print("\t");
  Serial.print(imuOffsets.accOffsetY);
  Serial.print("\t");
  Serial.print(imuOffsets.accOffsetZ);
  Serial.println("\nGyro Bias: ");
  Serial.print(imuOffsets.gyroOffsetX);
  Serial.print("\t");
  Serial.print(imuOffsets.gyroOffsetY);
  Serial.print("\t");
  Serial.print(imuOffsets.gyroOffsetZ);
  Serial.println();
}

void calibrateMagnetometer() {
  Serial.println("Calibration of the magnetometer will begin in 3s...");
  delay(3000);
  float magMin[3] = { 1000, 1000, 1000 };
  float magMax[3] = { -1000, -1000, -1000 };

  Serial.println("Rotate the sensor in all directions until calibration completes.");
  unsigned long startTime = millis();
  while (millis() - startTime < 15000) {  // Run calibration for 15 seconds
    if (imu.Read()) {
      float magX = imu.mag_x_ut();
      float magY = imu.mag_y_ut();
      float magZ = imu.mag_z_ut();

      if (magX < magMin[0]) magMin[0] = magX;
      if (magY < magMin[1]) magMin[1] = magY;
      if (magZ < magMin[2]) magMin[2] = magZ;

      if (magX > magMax[0]) magMax[0] = magX;
      if (magY > magMax[1]) magMax[1] = magY;
      if (magZ > magMax[2]) magMax[2] = magZ;
    }
    // delay(50);
  }

  // Compute hard iron offset
  imuOffsets.magOffsetX = (magMin[0] + magMax[0]) / 2;
  imuOffsets.magOffsetY = (magMin[1] + magMax[1]) / 2;
  imuOffsets.magOffsetZ = (magMin[2] + magMax[2]) / 2;

  imuOffsets.magScaleX = (magMax[0] - magMin[0]) / 2;
  imuOffsets.magScaleY = (magMax[1] - magMin[1]) / 2;
  imuOffsets.magScaleZ = (magMax[2] - magMin[2]) / 2;

  Serial.println("Magnetometer calibration complete.");
  Serial.println("\nMag Bias: ");
  Serial.print(imuOffsets.magOffsetX);
  Serial.print("\t");
  Serial.print(imuOffsets.magOffsetY);
  Serial.print("\t");
  Serial.print(imuOffsets.magOffsetZ);
  Serial.println("\nMag Scale: ");
  Serial.print(imuOffsets.magScaleX);
  Serial.print("\t");
  Serial.print(imuOffsets.magScaleY);
  Serial.print("\t");
  Serial.print(imuOffsets.magScaleZ);
  Serial.println();
}

void publishData() {
  if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Yaw", processedDataBuffer.yaw * RAD_TO_DEG)) {
    Serial.println("Yaw data sent to Firebase!");
  } else {
    Serial.print("Failed to send Yaw. Reason: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Pitch", processedDataBuffer.pitch * RAD_TO_DEG)) {
    Serial.println("Pitch data sent to Firebase!");
  } else {
    Serial.print("Failed to send Pitch. Reason: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Roll", processedDataBuffer.roll * RAD_TO_DEG)) {
    Serial.println("Roll data sent to Firebase!");
  } else {
    Serial.print("Failed to send Roll. Reason: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Temp", processedDataBuffer.sensTemp)) {
    Serial.println("Temp data sent to Firebase!");
  } else {
    Serial.print("Failed to send Temp. Reason: ");
    Serial.println(firebaseData.errorReason());
  }
}
