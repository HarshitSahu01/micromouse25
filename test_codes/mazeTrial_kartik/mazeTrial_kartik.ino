#include <Wire.h>
#include <SparkFun_TB6612.h>
#include <VL53L1X.h>

// ---------------- Motor driver pins ----------------
#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21
const int offsetA = 1;
const int offsetB = 1;
Motor motorLeft  = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// ---------------- Encoder setup ----------------
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Encoder configuration - adjust these values for your setup
const int encoderTicksPerRotation = 360;  // Adjust based on your encoder
const float tyreCircumference = 17.072;     // Wheel circumference in cm

// ESP32 interrupt-capable pins - adjust these to your actual connections
const int leftEncoderPin = 2;             // GPIO 2 for left encoder
const int rightEncoderPin = 15;           // GPIO 15 for right encoder (change from 3)

// Alternative pins you can try: 4, 5, 12, 13, 14, 15, 25, 26, 27

// ---------------- ToF sensor setup (keep front sensor for obstacle detection) ----------------
const uint8_t sensorCount = 1;
const uint8_t xshutPins[sensorCount] = { 27 }; // FRONT sensor only
VL53L1X sensors[sensorCount];

// ---------------- PID variables for straight-line driving ----------------
float straightKp = 2.3;   // Proportional gain - adjust based on testing
float straightKi = 0.0;   // Integral gain
float straightKd = 0.1;   // Derivative gain
float straightError = 0;
float straightPrev = 0;
float straightInt = 0;
float straightPIDValue = 0;

// ---------------- Robot settings ----------------
int baseSpeed = 150; // Base PWM speed

// ---------------- Navigation variables ----------------
unsigned long lastPIDTime = 0;
const int pidInterval = 50;         // PID calculation interval in ms

// ---------------- Encoder interrupt functions ----------------
void IRAM_ATTR leftEncoderISR() {
  leftEncoderCount++;
}

void IRAM_ATTR rightEncoderISR() {
  rightEncoderCount++;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(33, 32);   // Custom I2C pins for ESP32
  Wire.setClock(400000); // 400 kHz I2C speed
  
  // Setup encoder interrupts with more robust configuration
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  
  // Test encoder pins before setting interrupts
  Serial.println("Testing encoder pins...");
  Serial.print("Left encoder pin state: "); Serial.println(digitalRead(leftEncoderPin));
  Serial.print("Right encoder pin state: "); Serial.println(digitalRead(rightEncoderPin));
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, CHANGE);
  
  Serial.print("Left encoder interrupt number: "); Serial.println(digitalPinToInterrupt(leftEncoderPin));
  Serial.print("Right encoder interrupt number: "); Serial.println(digitalPinToInterrupt(rightEncoderPin));
  
  // Initialize front sensor for obstacle detection
  pinMode(xshutPins[0], OUTPUT);
  digitalWrite(xshutPins[0], LOW);
  delay(10);
  
  pinMode(xshutPins[0], INPUT); // Release XSHUT
  delay(10);
  sensors[0].setTimeout(500);
  if (!sensors[0].init())
  {
    Serial.println("Failed to detect and initialize front sensor");
    while (1);
  }
  sensors[0].setAddress(0x2A);
  sensors[0].startContinuous(50);
  
  Serial.println("Encoder-based PID system initialized");
  Serial.println("Move the robot manually to test encoders...");
  
  // Test encoders for 5 seconds
  unsigned long testStart = millis();
  while (millis() - testStart < 5000) {
    Serial.print("Left: "); Serial.print(leftEncoderCount);
    Serial.print("\tRight: "); Serial.println(rightEncoderCount);
    delay(500);
  }
  
  lastPIDTime = millis();
}



void stopMotors()
{
  motorLeft.drive(0);
  motorRight.drive(0);
}

// Function to move straight for a specific distance with PID correction
void moveDistance(int distance, int speed) {
  // Calculate target encoder counts for the distance
  int targetDistanceCount = (int)(((float)encoderTicksPerRotation / tyreCircumference) * distance);
  
  // Store starting encoder positions
  long startLeftCount = leftEncoderCount;
  long startRightCount = rightEncoderCount;
  
  // Reset PID variables for this movement
  straightError = 0;
  straightPrev = 0;
  straightInt = 0;
  
  Serial.print("Moving ");
  Serial.print(distance);
  Serial.println(" cm with PID correction");
  
  while (true) {
    // Calculate current distance traveled by each wheel
    long leftDistance = leftEncoderCount - startLeftCount;
    long rightDistance = rightEncoderCount - startRightCount;
    
    // Check if target distance reached
    if (abs(leftDistance) >= targetDistanceCount && abs(rightDistance) >= targetDistanceCount) {
      break;
    }
    
    // PID calculation for straight-line correction
    straightError = leftDistance - rightDistance; // Difference in encoder counts
    straightInt += straightError;
    float straightDeriv = straightError - straightPrev;
    straightPrev = straightError;
    straightPIDValue = straightKp * straightError + straightKi * straightInt + straightKd * straightDeriv;
    
    // Apply PID correction to motor speeds
    int leftSpeed = speed - straightPIDValue;
    int rightSpeed = speed + straightPIDValue;
    
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Drive motors
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
    
    // Debug output
    Serial.print("L_dist:"); Serial.print(leftDistance);
    Serial.print("\tR_dist:"); Serial.print(rightDistance);
    Serial.print("\tErr:"); Serial.print(straightError);
    Serial.print("\tPID:"); Serial.print(straightPIDValue);
    Serial.print("\tL_spd:"); Serial.print(leftSpeed);
    Serial.print("\tR_spd:"); Serial.println(rightSpeed);
    
    delay(10); // Small delay for stability
  }
  
  // Stop motors
  motorLeft.brake();
  motorRight.brake();
  Serial.println("Distance complete!");
}

// Function for continuous PID-corrected forward movement
void moveForwardWithPID() {
  // Only calculate PID at specified intervals
  if (millis() - lastPIDTime >= pidInterval) {
    // Calculate encoder difference for straight-line correction
    static long lastLeftCount = 0;
    static long lastRightCount = 0;
    
    long leftDelta = leftEncoderCount - lastLeftCount;
    long rightDelta = rightEncoderCount - lastRightCount;
    
    // PID calculation based on encoder difference
    straightError = leftDelta - rightDelta;
    straightInt += straightError;
    
    // Limit integral windup
    straightInt = constrain(straightInt, -1000, 1000);
    
    float straightDeriv = straightError - straightPrev;
    straightPrev = straightError;
    straightPIDValue = straightKp * straightError + straightKi * straightInt + straightKd * straightDeriv;
    
    // Apply PID correction to motor speeds
    int leftSpeed = baseSpeed - straightPIDValue;
    int rightSpeed = baseSpeed + straightPIDValue;
    
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Drive motors
    motorLeft.drive(leftSpeed);
    motorRight.drive(rightSpeed);
    
    // Debug output
    Serial.print("L_count:"); Serial.print(leftEncoderCount);
    Serial.print("\tR_count:"); Serial.print(rightEncoderCount);
    Serial.print("\tL_delta:"); Serial.print(leftDelta);
    Serial.print("\tR_delta:"); Serial.print(rightDelta);
    Serial.print("\tErr:"); Serial.print(straightError);
    Serial.print("\tPID:"); Serial.print(straightPIDValue);
    Serial.print("\tL_spd:"); Serial.print(leftSpeed);
    Serial.print("\tR_spd:"); Serial.println(rightSpeed);
    
    // Update for next calculation
    lastLeftCount = leftEncoderCount;
    lastRightCount = rightEncoderCount;
    lastPIDTime = millis();
  }
}

void loop()
{
  // Read front sensor for obstacle detection (optional - for monitoring only)
  //int distFront = sensors[0].read();
  sensors[0].read();
int distFront = sensors[0].ranging_data.range_mm;

  // Display front distance for debugging
  Serial.print("Front: "); Serial.print(distFront); Serial.print("mm\t");
  
  // Continuous forward movement with PID correction
  moveForwardWithPID();
}