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

// XSHUT pins for sensors
#define XSHUT_L 13
#define XSHUT_F 27
#define XSHUT_R 25

// I2C addresses
#define ADDR_L 0x30
#define ADDR_F 0x31
#define ADDR_R 0x32

// ---------------- Sensors & globals ----------------
VL53L1X sensorL, sensorF, sensorR;
int timingBudget = 33; // ms

int distLeft = 0, distFront = 0, distRight = 0;

const int baseSpeed = 240;
const int targetDist = 80; // mm desired wall distance
const int wallDetectThreshold = 120; // mm consider "wall present"
const int outerSpeed = 220;
const int innerSpeed = 120;

// ---------------- Wall PID ----------------
float Kp = 1.45;
float Ki = 0.0;
float Kd = 0.84;
float pidPrev = 0, pidInt = 0;

// ---------------- Mode ----------------
bool followLeft = false; // set at startup

// ---------------- Motor helper ----------------
void mspeed(int a, int b) {
  // clamp to -255..255
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);

  digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
  analogWrite(PWMA, abs(a));

  digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
  analogWrite(PWMB, abs(b));
}

// ---------------- Sensors ----------------
void setupSensors() {
  Wire.begin(33, 32);

  // Reset all sensors
  pinMode(XSHUT_L, OUTPUT); pinMode(XSHUT_F, OUTPUT); pinMode(XSHUT_R, OUTPUT);
  digitalWrite(XSHUT_L, LOW); digitalWrite(XSHUT_F, LOW); digitalWrite(XSHUT_R, LOW);
  delay(100);

  // Bring left up
  pinMode(XSHUT_L, INPUT); delay(10);
  if (!sensorL.init()) { Serial.println("SensorL init fail"); while(1); }
  sensorL.setAddress(ADDR_L);

  // front
  pinMode(XSHUT_F, INPUT); delay(10);
  if (!sensorF.init()) { Serial.println("SensorF init fail"); while(1); }
  sensorF.setAddress(ADDR_F);

  // right
  pinMode(XSHUT_R, INPUT); delay(10);
  if (!sensorR.init()) { Serial.println("SensorR init fail"); while(1); }
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
}

// ---------------- PID routines ----------------
void runWallPIDSingle(int measured, int desired, bool invertMirror) {
  // invertMirror == true -> mirror error (for right-wall case) so code below can be identical
  float e = (float)measured - (float)desired;
  if (invertMirror) e = -e;

  pidInt += e;
  pidInt = constrain(pidInt, -1000, 1000);
  float deriv = e - pidPrev;
  pidPrev = e;

  float pid = Kp * e + Ki * pidInt + Kd * deriv;

  // pid positive -> steering correction
  int leftSp = constrain(baseSpeed - pid, 0, 255);
  int rightSp = constrain(baseSpeed + pid, 0, 255);

  mspeed(leftSp, rightSp);
}

// ---------------- Main behavior (symmetric) ----------------
void behaviorStep() {
  updateSensors();

  // 1) If front obstacle, smooth arc depending on mode (Option B)
  if (distFront < wallDetectThreshold) {
    if (followLeft) {
      mspeed(outerSpeed, innerSpeed); // turn right
    } else {
      mspeed(innerSpeed, outerSpeed); // turn left
    }
    return;
  }
  if (followLeft) {
    if (distLeft > wallDetectThreshold) {
      mspeed(innerSpeed, outerSpeed);
      return;
    }
    runWallPIDSingle(distLeft, targetDist, false);
  } else {
    if (distRight > wallDetectThreshold) {
      mspeed(outerSpeed, innerSpeed);
      return;
    }
    runWallPIDSingle(distRight, targetDist, true);
  }

}

// ---------------- Setup & loop ----------------
void setup() {
  Serial.begin(115200);
  delay(10);

  // Motor pins
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  // Buttons
  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);

  // Sensors
  setupSensors();

  Serial.println("Press BTN1 (left) or BTN2 (right) to choose wall-follow mode...");
  // wait for a button press at start
  while (true) {
    if (digitalRead(BTN1) == HIGH) { followLeft = true; break; }
    if (digitalRead(BTN2) == HIGH) { followLeft = false; break; }
    delay(10);
  }
  delay(300);
  Serial.printf("Mode chosen: %s-wall follow\n", followLeft ? "LEFT" : "RIGHT");
}

void loop() {
  behaviorStep();
  delay(timingBudget); // cadence matched to sensor timing
}