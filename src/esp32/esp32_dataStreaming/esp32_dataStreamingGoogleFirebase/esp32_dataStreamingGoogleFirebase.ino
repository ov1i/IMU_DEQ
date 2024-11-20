#include <WiFi.h>
#include <FirebaseESP32.h>

// Wi-Fi credentials
const char* ssid = "DIGI_eb21f8";
const char* password = "ddcc14de";

// Firebase credentials
#define FIREBASE_HOST "https://projectiot-ee562-default-rtdb.europe-west1.firebasedatabase.app"  // Replace with your Firebase Realtime Database URL
#define FIREBASE_AUTH "b	UnJ5jFpZhqC2gRhkxKPBcZyhokwk5JFZmWeJ7kn3"    // Replace with your Firebase Database Secret or API Key

FirebaseData firebaseData;  // Firebase Data Object

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");

  // Initialize Firebase
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
}

void loop() {
  // Replace this with your sensor data
  float temperature = random(20, 30);
  float humidity = random(60, 100);

  // Send data to Firebase
  if (Firebase.pushFloat(firebaseData, "/SensorData/Temperature", temperature)) {
    Serial.println("Temperature data sent to Firebase!");
  } else {
    Serial.print("Failed to send temperature. Reason: ");
    Serial.println(firebaseData.errorReason());
  }

  if (Firebase.pushFloat(firebaseData, "/SensorData/Humidity", humidity)) {
    Serial.println("Humidity data sent to Firebase!");
  } else {
    Serial.print("Failed to send humidity. Reason: ");
    Serial.println(firebaseData.errorReason());
  }

  delay(5000);  // Update every 5 seconds
}
