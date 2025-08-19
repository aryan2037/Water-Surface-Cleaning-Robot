#include <ESP32Servo.h>  // Include ESP32Servo library for controlling motors

// Motor pins
Servo ESC1;  // Right motor
Servo ESC2;  // Left motor

int count = 0;
// Pin definitions
const int inputPin1 = 34;   // Pin for right direction
const int inputPin2 = 35;   // Pin for left direction
const int inputPin3 = 32;   // Pin for stop condition
const int ledPin = 13;     // Pin for LED
const int trigPinLeft = 14; // Trigger pin for left ultrasonic sensor
const int echoPinLeft = 27; // Echo pin for left ultrasonic sensor
const int trigPinRight = 12;// Trigger pin for right ultrasonic sensor
const int echoPinRight = 26;// Echo pin for right ultrasonic sensor
const int trigPinFront = 25;// Trigger pin for Front ultrasonic sensor
const int echoPinFront = 33;// Echo pin for Front ultrasonic sensor
// Ultrasonic sensor variables
long durationLeft, distanceLeft;
long durationRight, distanceRight;
long durationFront, distanceFront;

// Motor speed variables
int motor1Speed = 100; // Default to stopped
int motor2Speed = 100; // Default to stopped
int targetMotor1Speed = 100; // Target speed for smooth adjustments
int targetMotor2Speed = 100; // Target speed for smooth adjustments

void setup() {
  // Setup motor pins
Serial.begin(9600);

  ESC1.attach(18, 1000, 2000); // Attach the ESC to the specified pin with min and max pulse widths
  ESC1.write(0); // Send 0 signal to stop the motor
  delay(2000);   // Left motor is attached to pin 27
  ESC2.attach(19, 1000, 2000); // Attach the ESC to the specified pin with min and max pulse widths
  ESC2.write(0); // Send 0 signal to stop the motor
  delay(2000); 
  // Setup input pins
  pinMode(inputPin1, INPUT);
  pinMode(inputPin2, INPUT);
  pinMode(inputPin3, INPUT);

  // Setup LED pin
  pinMode(ledPin, OUTPUT);

  // Setup ultrasonic sensor pins
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
}

void loop() {
  // Step 1: Check ultrasonic sensors
  distanceLeft = getDistance(trigPinLeft, echoPinLeft);
  distanceRight = getDistance(trigPinRight, echoPinRight);
  distanceFront = getDistance(trigPinFront, echoPinFront);
  Serial.print("distanceLeft  ");
  Serial.println(distanceLeft);
  Serial.print("distanceRight  ");
  Serial.println(distanceRight);
  Serial.print("distancefront  ");
  Serial.println(distanceFront);

  
    // Adjust for obstacles
    if (distanceLeft < 20) {
      blinkLED(distanceLeft);
      if (distanceFront < 50) {
      // If obstacle is too close on the left, move right
      targetMotor1Speed = 100; // Full speed for right motor
      targetMotor2Speed = max(0, targetMotor2Speed - 5);   // Gradually reduce speed for left motor
      
    } }
    else if (distanceRight < 20) {
      blinkLED(distanceRight);
      if (distanceFront < 50) {
      // If obstacle is too close on the right, move left
      targetMotor1Speed = max(0, targetMotor1Speed - 5);   // Gradually reduce speed for right motor
      targetMotor2Speed = 100; // Full speed for left motor
      
    } }
    else {
    if (digitalRead(inputPin1) == HIGH) {
      // Move right
      if (distanceRight > 20){
      targetMotor1Speed = 100; // Full speed for right motor
      targetMotor2Speed = max(0, targetMotor2Speed - 5);   // Gradually reduce speed for left motor
      blinkLED(distanceRight);
    } }
    else if (digitalRead(inputPin2) == HIGH) {
      // Move left
      if (distanceLeft > 20){
      targetMotor1Speed = max(0, targetMotor1Speed - 5);   // Gradually reduce speed for right motor
      targetMotor2Speed = 100; // Full speed for left motor
      blinkLED(distanceLeft);
    }}
     else if (digitalRead(inputPin3) == HIGH) {
      // Stop condition
       if (distanceFront > 50){
      targetMotor1Speed = 100;
      targetMotor2Speed = 100;
      blinkLED(distanceFront);
    }}
  }

  // Gradually adjust motor speeds
  adjustMotorSpeed(); // Short delay for stability
}

// Function to get the distance from an ultrasonic sensor
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration / 2) * 0.0344;  // Convert time to distance (in cm)
  return distance;
}

// Function to blink the LED based on the distance
void blinkLED(long distance) {
  if (distance < 50) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(20);
  } 
  else if (distance < 150) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(30);
  }
  else {
    digitalWrite(ledPin, HIGH);
  }
}

// Function to gradually adjust motor speeds
void adjustMotorSpeed() {
  if (motor1Speed < targetMotor1Speed) {
    motor1Speed = min(motor1Speed + 5, targetMotor1Speed);
  } else if (motor1Speed > targetMotor1Speed) {
    motor1Speed = max(motor1Speed - 5, targetMotor1Speed);
  }

  if (motor2Speed < targetMotor2Speed) {
    motor2Speed = min(motor2Speed + 5, targetMotor2Speed);
  } else if (motor2Speed > targetMotor2Speed) {
    motor2Speed = max(motor2Speed - 5, targetMotor2Speed);
  }

  ESC1.write(motor1Speed);
  ESC2.write(motor2Speed);
}
