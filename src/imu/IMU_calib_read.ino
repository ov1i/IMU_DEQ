#include "MPU9250.h"

MPU9250 imu; // Default I2C address

void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(2000);
  Serial.println("Initializing MPU9250...");
  imu.setup(0x68);
  delay(3000);

  // Calibrate accelerometer and gyroscope
  Serial.println("Calibration of GYRO and ACCELEROMETER started please DO NOT move the device...");
  imu.calibrateAccelGyro();
  Serial.println("Calibration of the GYRO and ACCELEROMETER complete!");

  delay(3000);

  // Calibrate magnetometer
  Serial.println("Calibration of the magnetometer started please move the sensor in all directions...");
  imu.calibrateMag();
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
  if(imu.update()) {
    float yaw = imu.getYaw();    // In degrees
    float pitch = imu.getPitch(); // In degrees
    float roll = imu.getRoll();   // In degrees

    Serial.print("Yaw: "); Serial.print(yaw);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Roll: "); Serial.println(roll);
  }
}
