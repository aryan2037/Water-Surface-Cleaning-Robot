#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <Wire.h>

// Pin definitions for LoRa
#define SS 53
#define RST 48
#define DIO0 49

// Pin definitions for Ultrasonic Sensor
const int trigPin = 6;
const int echoPin = 7;

// Timing controls for each sensor
unsigned long lastTempRead = 0;
unsigned long lastPressureRead = 0;
unsigned long lastDistanceRead = 0;
unsigned long lastTransmission = 0;
unsigned long lastGPSRead = 0;

// Intervals for sensor readings (in milliseconds)
const unsigned long TEMP_INTERVAL = 1000;     // Temperature every 1 second
const unsigned long PRESSURE_INTERVAL = 1000; // Pressure every 1 second
const unsigned long DISTANCE_INTERVAL = 500;  // Distance every 0.5 seconds
const unsigned long GPS_INTERVAL = 1000;      // GPS every 1 second
const unsigned long TRANSMISSION_INTERVAL = 2000; // Transmit every 2 seconds

// Variables to store sensor data
float temperature = 0.0;
float pressure = 0.0;
float distance = 0.0;
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;

// Flags to track if new data is available
bool newTempData = false;
bool newPressureData = false;
bool newDistanceData = false;
bool newGPSData = false;

// Sensor objects
Adafruit_BMP280 bmp;
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // Use Serial1 for GPS communication

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize LoRa
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed.");
    while (1);
  }
  Serial.println("LoRa Transmitter Initialized");

  // Initialize BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 sensor not found.");
    while (1);
  }
  Serial.println("BMP280 initialized");
}

void loop() {
  unsigned long currentMillis = millis();

  // Check and read temperature sensor
  if (currentMillis - lastTempRead >= TEMP_INTERVAL) {
    readTemperature();
    lastTempRead = currentMillis;
  }

  // Check and read pressure sensor
  if (currentMillis - lastPressureRead >= PRESSURE_INTERVAL) {
    readPressure();
    lastPressureRead = currentMillis;
  }

  // Check and read distance sensor
  if (currentMillis - lastDistanceRead >= DISTANCE_INTERVAL) {
    readDistance();
    lastDistanceRead = currentMillis;
  }

  // Check and read GPS data
  if (currentMillis - lastGPSRead >= GPS_INTERVAL) {
    readGPS();
    lastGPSRead = currentMillis;
  }

  // Transmit data if interval has passed and we have new data
  if (currentMillis - lastTransmission >= TRANSMISSION_INTERVAL) {
    if (newTempData || newPressureData || newDistanceData || newGPSData) {
      transmitData();
      lastTransmission = currentMillis;

      // Reset new data flags
      newTempData = false;
      newPressureData = false;
      newDistanceData = false;
      newGPSData = false;
    }
  }
}

void readTemperature() {
  temperature = bmp.readTemperature();
  newTempData = true;
  Serial.println("Temperature: " + String(temperature) + " °C");
}

void readPressure() {
  pressure = bmp.readPressure() / 100.0;
  newPressureData = true;
  Serial.println("Pressure: " + String(pressure) + " hPa");
}

void readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.0343) / 2;
  newDistanceData = true;
  Serial.println("Distance: " + String(distance) + " cm");
}

void readGPS() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        altitude = gps.altitude.meters();
        newGPSData = true;
        Serial.println("GPS Data: Lat = " + String(latitude) + ", Lon = " + String(longitude) + ", Alt = " + String(altitude));
      } else {
        Serial.println("Invalid GPS data");
      }
    }
  }
}

void transmitData() {
  String dataPacket = String(temperature) + "," + String(pressure) + "," + 
                      String(distance) + "," + String(latitude) + "," + 
                      String(longitude) + "," + String(altitude);

  LoRa.beginPacket();
  LoRa.print(dataPacket);
  LoRa.endPacket();

  Serial.println("Transmitted: " + dataPacket);
}
