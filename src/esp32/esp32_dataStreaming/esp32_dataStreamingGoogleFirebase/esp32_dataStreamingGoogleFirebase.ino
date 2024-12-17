#include <WiFi.h>
#include <Wire.h>
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <addons/RTDBHelper.h>

const char* ssid = "DIGI_eb21f8";
const char* password = "ddcc14de";

#define FIREBASE_HOST "https://projectiot-ee562-default-rtdb.europe-west1.firebasedatabase.app/"
#define FIREBASE_SECRET "UnJ5jFpZhqC2gRhkxKPBcZyhokwk5JFZmWeJ7kn3"

FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

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
  firebaseData.setBSSLBufferSize(4096 /* Rx buffer size: [ 512 - 16384 ] bytes */, 1024 /* Tx buffer size: [ 512 - 16384 ] bytes */);
  Firebase.begin(&config, &auth);
  // Firebase.begin(DATABASE_URL, DATABASE_SECRET); // LEGACY Connection

  delay(3000);
}

void loop() {
  float yaw, pitch, roll, temp;

  yaw = random(1, 10);
  pitch = random(10, 20);
  roll = random(20, 30);
  temp = random(30, 40);

  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Roll: ");
  Serial.println(roll);
  Serial.print(" | Temp: "); Serial.println(temp);

  if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Yaw", yaw)) {
    Serial.println("Yaw data sent to Firebase!");
  } else {
    Serial.print("Failed to send Yaw. Reason: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Pitch", pitch)) {
    Serial.println("Pitch data sent to Firebase!");
  } else {
    Serial.print("Failed to send Pitch. Reason: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Roll", roll)) {
    Serial.println("Roll data sent to Firebase!");
  } else {
    Serial.print("Failed to send Roll. Reason: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.RTDB.setFloat(&firebaseData, "/SensorData/Temp", temp)) {
    Serial.println("Temp data sent to Firebase!");
  } else {
    Serial.print("Failed to send Temp. Reason: ");
    Serial.println(firebaseData.errorReason());
  }


  delay(1000);
}
