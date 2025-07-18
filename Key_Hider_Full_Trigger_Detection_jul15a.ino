
//ok this offically works great!!! 
// Made by Claude Sonnet 4 fixed by Deepseek R1

#include <ESP32Servo.h>

// Pin definitions
#define TRIG_PIN 5      // Ultrasonic trigger pin
#define ECHO_PIN 18     // Ultrasonic echo pin
#define SERVO_PIN 4     // Servo signal pin

// Create servo object
Servo myServo;

// Settings you can easily change
int MIN_DISTANCE = 5;     // Start moving servo at this distance (cm)
int MAX_DISTANCE = 50;    // Stop moving servo at this distance (cm)
int MIN_ANGLE = 0;        // Servo angle when object is closest
int MAX_ANGLE = 180;      // Servo angle when object is farthest

// Detection mode variables
bool detectionMode = false;
unsigned long lastDetectionTime = 0;
const unsigned long NO_DETECTION_TIMEOUT = 0; // 2 seconds of no detection to exit detection mode 2000 original

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Attach servo
  myServo.attach(SERVO_PIN);
  
  // Start servo at middle position
  myServo.write(180); // change this original 90
  
  Serial.println("ESP32 Ultrasonic Servo Control Ready!");
}

void loop() {
  // Get distance from ultrasonic sensor
  long distance = getDistance();
  
  // Check if any object is detected within the full sensor range (up to MAX_DISTANCE)
  bool objectDetected = (distance > 0 && distance <= MAX_DISTANCE);
  
  if (objectDetected) {
    // Object detected - enter or stay in detection mode
    if (!detectionMode) {
      detectionMode = true;
      myServo.write(MIN_ANGLE);
      Serial.println("Detection mode activated - servo moved to minimum angle");
    }
    lastDetectionTime = millis();
    
    Serial.print("Detection mode - Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Servo at minimum angle: ");
    Serial.println(MIN_ANGLE);
    
  } else {
    // No object detected
    if (detectionMode) {
      // Check if enough time has passed without detection
      if (millis() - lastDetectionTime > NO_DETECTION_TIMEOUT) {
        detectionMode = false;
        myServo.write(180); // Reset servo to 180° DeepSeek R1
        Serial.println("Detection mode deactivated - returning to normal operation");
      } else {
        // Still in detection mode, keep servo at minimum angle
        Serial.println("Detection mode - waiting for timeout, servo at minimum angle");
      }
    }
  }
  
  // Small delay for stability
  delay(50);
}

long getDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance (speed of sound = 343 m/s)
  long distance = duration * 0.034 / 2;
  
  return distance;
}
