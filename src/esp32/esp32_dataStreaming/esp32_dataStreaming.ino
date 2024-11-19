#include <WiFi.h>
#include "ThingSpeak.h"

// Wi-Fi credentials
const char* ssid = "SSID";
const char* password = "PSWD";

// ThingSpeak credentials
WiFiClient client;
unsigned long CH_no = CH_NO; // Replace with your channel number
const char* API_key = "API_KEY";    // Replace with your Write API Key

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing ESP32-CAM...");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

void loop() {
  // Simulate a sensor reading (e.g., temperature)
  float test = random(20, 30); // Replace with actual sensor reading

  // Send data to ThingSpeak
  int statusCode = ThingSpeak.writeField(CH_no, 1, test, API_key);
  if (statusCode == 200) {
    Serial.println("Data sent successfully to ThingSpeak!");
  } else {
    Serial.print("Failed to send data. HTTP error code: ");
    Serial.println(statusCode);
  }

  delay(20000); // Wait 20 seconds (minimum ThingSpeak update interval is 15 seconds)
}
