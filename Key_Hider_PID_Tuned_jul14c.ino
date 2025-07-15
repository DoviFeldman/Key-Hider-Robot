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

// PID tuning parameters - adjust these to reduce jitter
float Kp = 0.3;          // Proportional gain (higher = more responsive)  2.0 original P 0.5 this is starting to work 0.3 works even better
float Ki = 0.08;          // Integral gain (helps eliminate steady-state error) 0.1 I  0.05 this is starting to work  0.08 is good too
float Kd = 0.1;          // Derivative gain (reduces overshoot and oscillation) 0.5 D  0.5 this is starting to work 0.1 works amazing!!! increadible!!!! works super great!!!

// PID variables
float previousError = 0;
float integral = 0;
float currentAngle = 90; // Current servo position

unsigned long noDetectionStartTime = 0; // THIS IS FOR THE RETURN TO 180 DEGREES TIMER
bool noDetectionTimerActive = false; // THIS IS FOR THE RETURN TIMER

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Attach servo
  myServo.attach(SERVO_PIN);
  
  // Start servo at middle position
  myServo.write(180); // was 90 
  
  Serial.println("ESP32 Ultrasonic Servo Control Ready!");
}

void loop() {
  // Get distance from ultrasonic sensor
  long distance = getDistance();
  
  // Only move servo if object is within our range
  if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
    
    noDetectionTimerActive = false; // THIS IS FOR THE RETURN TIMER
    
    // Calculate target angle based on distance
    int targetAngle = map(distance, MIN_DISTANCE, MAX_DISTANCE, MIN_ANGLE, MAX_ANGLE);
    
    // Apply PID control for smooth movement
    float smoothAngle = pidControl(targetAngle);
    
    // Move servo to smooth position
    myServo.write((int)smoothAngle);
    currentAngle = smoothAngle;
    
    // Print info to serial monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Target: ");
    Serial.print(targetAngle);
    Serial.print("°, Smooth: ");
    Serial.println(smoothAngle);
  }

  
  /////////////////////////  Hontly this codes not as smooth as it was bbefore but it should be the same and its good enough
else { 
  // Start timer if not already active
  if (!noDetectionTimerActive) {
    noDetectionStartTime = millis();
    noDetectionTimerActive = true;
  }
  
  // Only move to 180 after 1 second of no detection
  if (millis() - noDetectionStartTime >= 1000) {
    float smoothAngle = pidControl(180);
    myServo.write((int)smoothAngle);
    currentAngle = smoothAngle;
  }
} // Comment out this whole else block if you don't want auto-return
 // Comment out this whole else block if you don't want auto-return // HONSTLY IT DOESNT WORK AS WELL WITH THIS WE SHOULD JUST USE THE TRIGGERED ONE IF WE WANT IT TO MOVE BACK, OR MAYBE I COULD COMBINE THEM AND ADD A TIMER SO
  /////////////////////////////  

  
  // Small delay for stability
  delay(20);
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

float pidControl(float targetAngle) {
  // Calculate error
  float error = targetAngle - currentAngle;
  
  // Proportional term
  float proportional = Kp * error;
  
  // Integral term (accumulated error over time)
  integral += error;
  float integralTerm = Ki * integral;
  
  // Derivative term (rate of change of error)
  float derivative = error - previousError;
  float derivativeTerm = Kd * derivative;
  
  // Calculate PID output
  float output = proportional + integralTerm + derivativeTerm;
  
  // Calculate new angle
  float newAngle = currentAngle + output;
  
  // Constrain to servo limits
  newAngle = constrain(newAngle, MIN_ANGLE, MAX_ANGLE);
  
  // Update previous error for next iteration
  previousError = error;
  
  return newAngle;
}
//
//
//Perfect! I've added PID control to smooth out the servo movement and reduce jitter. Here's what's new:
//PID Tuning Parameters (Easy to Adjust)
//At the top of the code, you can tune these values:
//
//Kp = 2.0 - Proportional gain (higher = more responsive, but can cause overshoot)
//Ki = 0.1 - Integral gain (eliminates steady-state error, prevents drift)
//Kd = 0.5 - Derivative gain (reduces overshoot and oscillation)
//
//How to Tune for Smooth Movement
//Start with these values and adjust:
//
//Too jittery/oscillating? → Reduce Kp and/or increase Kd
//Too slow to respond? → Increase Kp
//Doesn't reach target position? → Increase Ki slightly
//Overshoots and bounces back? → Increase Kd
//
//Tuning Tips
//
//For smooth, slow movement: Try Kp=1.0, Ki=0.05, Kd=0.8 (tried too jittery, very fast)
//For responsive movement: Try Kp=3.0, Ki=0.2, Kd=0.3
//For very stable movement: Try Kp=1.5, Ki=0.1, Kd=1.0  (tried, too jittery)
//
//The PID controller now calculates smooth movements instead of jumping directly to the target angle. The servo will gradually move to the desired position based on the distance reading, eliminating most jitter and making the movement much smoother.
//You can monitor the Serial output to see the difference between the target angle and the smooth angle being applied.
//
//



