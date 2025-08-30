/* Wall Follower (no encoders) with smooth-front-arc behavior
   - BTN1 = left wall follow
   - BTN2 = right wall follow
   - baseSpeed = 240
   - Smooth "search toward missing wall" logic
   - Front-obstacle -> smooth arc (direction depends on follow mode)
   - Uses VL53L1X left/front/right
*/

#include <Wire.h>
#include <VL53L1X.h>

// ---------------- Pins ----------------
#define BTN1 34
#define BTN2 35

#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21

#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// XSHUT pins for sensors
#define XSHUT_L 13
#define XSHUT_F 27
#define XSHUT_R 25

// I2C addresses
#define ADDR_L 0x30
#define ADDR_F 0x31
#define ADDR_R 0x32

void IRAM_ATTR leftEncoderISR() {
  bool a = digitalRead(LEFT_ENC_A);
  bool b = digitalRead(LEFT_ENC_B);
  leftTicks += (a == b) ? 1 : -1;
}

void IRAM_ATTR rightEncoderISR() {
  bool a = digitalRead(RIGHT_ENC_A);
  bool b = digitalRead(RIGHT_ENC_B);
  rightTicks += (a == b) ? 1 : -1;
}

// ---------------- Sensors & globals ----------------
VL53L1X sensorL, sensorF, sensorR;
int timingBudget = 20; // ms
int curtime = 0; // for timing budget

int distLeft = 0, distFront = 0, distRight = 0;

const int baseSpeed = 230;        // Further reduced for better control
const int targetDist = 80;        // mm desired wall distance
const int frontThresh = 150;      // Reduced threshold for earlier detection
const int wallThresh = 180;       // Increased for better wall detection
const int outerSpeed = 255;       // Reduced for smoother turns
const int innerSpeed = 70;        // Increased minimum for better movement
const int rotationSpeed = 180;    // Further reduced for better accuracy

// ---------------- Wall PID ----------------
float Kp = 1.40;                   // Increased for better response
float Ki = 0.0002;                 // Increased for steady-state accuracy
float Kd = 0.87;                   // Increased for stability
float pidPrev = 0, pidInt = 0;

// ---------------- Mode ----------------
bool followLeft = false; // set at startup

// ---------------- Motor helper ----------------
void mspeed(int a, int b) {
    if (abs(a) <= 255) {
        digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
        analogWrite(PWMA, abs(a));
    }

    if (abs(b) <= 255) {
        digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
        analogWrite(PWMB, abs(b));
    }
}

// ---------------- Sensors ----------------
void setupSensors() {
  Wire.begin(33, 32);

  // Reset all sensors
  pinMode(XSHUT_L, OUTPUT); 
  pinMode(XSHUT_F, OUTPUT); 
  pinMode(XSHUT_R, OUTPUT);
  digitalWrite(XSHUT_L, LOW); 
  digitalWrite(XSHUT_F, LOW); 
  digitalWrite(XSHUT_R, LOW);
  delay(100);

  // Bring left up
  pinMode(XSHUT_L, INPUT); 
  delay(10);
  if (!sensorL.init()) { 
    Serial.println("SensorL init fail"); 
    while(1); 
  }
  sensorL.setAddress(ADDR_L);

  // front
  pinMode(XSHUT_F, INPUT); 
  delay(10);
  if (!sensorF.init()) { 
    Serial.println("SensorF init fail"); 
    while(1); 
  }
  sensorF.setAddress(ADDR_F);

  // right
  pinMode(XSHUT_R, INPUT); 
  delay(10);
  if (!sensorR.init()) { 
    Serial.println("SensorR init fail"); 
    while(1); 
  }
  sensorR.setAddress(ADDR_R);

  sensorL.setDistanceMode(VL53L1X::Medium);
  sensorF.setDistanceMode(VL53L1X::Medium);
  sensorR.setDistanceMode(VL53L1X::Medium);

  sensorL.setMeasurementTimingBudget(timingBudget * 1000);
  sensorF.setMeasurementTimingBudget(timingBudget * 1000);
  sensorR.setMeasurementTimingBudget(timingBudget * 1000);

  sensorL.startContinuous(timingBudget);
  sensorF.startContinuous(timingBudget);
  sensorR.startContinuous(timingBudget);

  Serial.println("Sensors up");
}

void updateSensors() {
  distLeft  = sensorL.read();
  distFront = sensorF.read();
  distRight = sensorR.read();
  
  // Add sensor error handling
  if (distLeft == 0 || distLeft > 4000) distLeft = 4000;
  if (distFront == 0 || distFront > 4000) distFront = 4000;
  if (distRight == 0 || distRight > 4000) distRight = 4000;
}

// ---------------- PID routines ----------------
void runWallPIDSingle(int measured, int desired, bool invertMirror) {
  // invertMirror == true -> mirror error (for right-wall case) so code below can be identical
  float e = (float)measured - (float)desired;
  if (invertMirror) e = -e;

  pidInt += e;
  pidInt = constrain(pidInt, -5000, 5000);  // Increased integral limits
  float deriv = e - pidPrev;
  pidPrev = e;

  float pid = Kp * e + Ki * pidInt + Kd * deriv;

  // pid positive -> steering correction
  int leftSp = constrain(baseSpeed - pid, 30, 255);   // Added minimum speed
  int rightSp = constrain(baseSpeed + pid, 30, 255);  // Added minimum speed

  mspeed(leftSp, rightSp);
}

float encoderCountToDegrees = 0.945;
void rotate(int degree) {
    int target = encoderCountToDegrees * abs(degree);
    int dir = degree > 0 ? 1 : -1;
    bool rotatingLeft = true, rotatingRight = true;
    leftTicks = rightTicks = 0;
    int rspeed = baseSpeed;

    mspeed(dir*rspeed, -dir*rspeed);
    while (rotatingLeft or rotatingRight) {
        if (abs(leftTicks) > target) {
            mspeed(0, 300);
            rotatingLeft = false;
        }
        if (abs(rightTicks) > target) {
            mspeed(300, 0);
            rotatingRight = false;
        }
        if (abs(leftTicks) > target - 50 or abs(rightTicks) > target - 50) {
          rspeed = 120;
        }
        delay(5);
    }
    mspeed(0, 0);
    pidInt = 0;
    pidPrev = 0; 
}

// ---------------- Main behavior (symmetric) ----------------
void behaviorStep() {
  updateSensors();
  
  // Debug output
  // Serial.printf("L:%d F:%d R:%d Mode:%s\n", 
  //               distLeft, distFront, distRight, 
  //               followLeft ? "LEFT" : "RIGHT");

  // 1) Handle missing wall first - search behavior (higher priority than front obstacle)
  if (followLeft) {
    if (distLeft > wallThresh) {
      // Serial.println("Left wall missing - turning left");
      mspeed(innerSpeed, outerSpeed);  // Turn left to find wall
      return;
    }
  } else {
    if (distRight > wallThresh) {
      // Serial.println("Right wall missing - turning right");
      mspeed(outerSpeed, innerSpeed);  // Turn right to find wall
      return;
    }
  }

  // 2) Check for front obstacle - but only when we have a wall to follow
  if (distFront < frontThresh) {
    // Serial.println("Front obstacle detected - executing turn");
    
    // Don't stop completely, just slow down briefly
    mspeed(-100, -100);
    delay(20);
    
    if (followLeft) {
      // Left wall following: turn right away from wall
      // Serial.println("Left follow mode - turning right 90 degrees");
      rotate(90);
    } else {
      // Right wall following: turn left away from wall
      // Serial.println("Right follow mode - turning left 90 degrees");
      rotate(-90);
    }
    return;
  }

  // 3) Wall following with PID
  if (followLeft) {
    runWallPIDSingle(distLeft, targetDist, false);
  } else {
    runWallPIDSingle(distRight, targetDist, true);
  }
}

// ---------------- Setup & loop ----------------
void setup() {
  Serial.begin(115200);
  delay(10);

  // Motor pins
  pinMode(PWMA, OUTPUT); 
  pinMode(AIN1, OUTPUT); 
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); 
  pinMode(BIN1, OUTPUT); 
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); 
  digitalWrite(STBY, HIGH);

  // Buttons
  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);

  // Sensors
  setupSensors();
  
  // Encoder setup
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  Serial.println("Press BTN1 (left) or BTN2 (right) to choose wall-follow mode...");
  
  // wait for a button press at start
  while (true) {
    if (digitalRead(BTN1) == HIGH) { 
      followLeft = true; 
      break; 
    }
    if (digitalRead(BTN2) == HIGH) { 
      followLeft = false; 
      break; 
    }
    delay(10);
  }
  delay(300);
  Serial.printf("Mode chosen: %s-wall follow\n", followLeft ? "LEFT" : "RIGHT");
  curtime = millis();
}

void loop() {
  behaviorStep();
  while(millis() - curtime < timingBudget) {}
  curtime = millis();
}