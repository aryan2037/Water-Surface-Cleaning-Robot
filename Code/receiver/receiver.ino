#include <SPI.h>
#include <LoRa.h>

// Pin definitions for LoRa
#define SS D8
#define RST D0
#define DIO0 D1

// Variables to store received data
float temperature = 0.0;
float pressure = 0.0;
float distance = 0.0;
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;

void setup() {
  Serial.begin(115200);

  // Initialize LoRa
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed.");
    while (1);
  }
  Serial.println("LoRa Receiver Initialized");
}

void loop() {
  // Try to parse a received LoRa packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedData = "";

    // Read the packet
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }

    Serial.println("Received: " + receivedData);

    // Parse the data
    parseData(receivedData);
  }
}

void parseData(String data) {
  int index = 0;
  String value = "";
  float parsedValues[6]; // Array to store parsed values

  // Parse the comma-separated data
  for (int i = 0; i < data.length(); i++) {
    if (data[i] == ',') {
      parsedValues[index++] = value.toFloat();
      value = "";
    } else {
      value += data[i];
    }
  }
  parsedValues[index] = value.toFloat(); // Add the last value

  // Assign parsed values to variables
  temperature = parsedValues[0];
  pressure = parsedValues[1];
  distance = parsedValues[2];
  latitude = parsedValues[3];
  longitude = parsedValues[4];
  altitude = parsedValues[5];

  // Print parsed values to Serial Monitor
  Serial.println("Temperature: " + String(temperature) + " °C");
  Serial.println("Pressure: " + String(pressure) + " hPa");
  Serial.println("Distance: " + String(distance) + " cm");
  Serial.println("Latitude: " + String(latitude));
  Serial.println("Longitude: " + String(longitude));
  Serial.println("Altitude: " + String(altitude) + " m");
}
