// Not working correctly
#include <Wire.h>
#include <VL53L1X.h>
#include <queue>
#include <utility>
using namespace std;

#define BTN1 34
#define BTN2 35

#define LED1 0
#define LED2 12

#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21

// Encoder pins
#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// XSHUT pins
#define XSHUT1 13
#define XSHUT2 27
#define XSHUT3 25

// I2C addresses we’ll assign
#define ADDR1 0x30
#define ADDR2 0x31
#define ADDR3 0x32

// Sensor objects
VL53L1X sensor1;
VL53L1X sensor2;
VL53L1X sensor3;

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

int timingBudget = 20; // in mm
int frontThresh = 60;
int frontSlowThresh = 100;
int frontWallCorrection = 150;
int baseSpeed = 220;
int rotSpeed = 80;
int minSpeed = 80;

const int MAZESIZE = 5;
int targetX = MAZESIZE/2, targetY = MAZESIZE/2;
const int unitBlock = 18;
int blockSize = unitBlock;
const int rotationAxisCorrection= 3;

int maze[MAZESIZE][MAZESIZE];
int flood[MAZESIZE][MAZESIZE];
int posX = 0, posY = 0; // Current position in the maze
int orientation = 2;

// ---------------- PID variables ----------------
float wallKp = 2.5;   // start small
float wallKi = 0.02; // start near zero
float wallKd = 10;    // start small

unsigned long lastMillis = 0;

void setdelay() {
  lastMillis = millis();
}

float rotation_kp = 2.4;     // proportional gain
float rotation_ki = 0.002;    // integral gain (small, avoids bias drift)
float rotation_kd = 1.5;     // derivative gain

float encoderCountToDegrees = 0.80;  //0.85   // calibration factor
float encoderCountToDegrees180 = 0.84;  //0.85   // calibration factor

int rotBaseSpeed = 120;                // base rotation speed
int rotMaxSpeed = 200;  


void setup() {
  // put your setup code here, to run once:
     Serial.begin(115200);
    // Use custom I2C pins: SDA = 33, SCL = 32

    Wire.begin(33, 32);
    Wire.setClock(400000);

    setupMotorEncodersMore();

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(1000);
  rotate(90);
  delay(2000); 
  rotate(180);


}

void rotate(int degree) {
    long targetTicks;
    if (degree == 180) {
        blockSize -= rotationAxisCorrection;
        targetTicks = encoderCountToDegrees180 * abs(degree);
    }
    else{
        targetTicks = encoderCountToDegrees * abs(degree);
    }
    int dir = (degree > 0) ? 1 : -1;

    // Reset encoder counts
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();

    //setYaw();

    // Reset PID state
    float error = 0, prevError = 0, integral = 0;
    bool rotatingLeft = true, rotatingRight = true;

    while (rotatingLeft || rotatingRight) {
        // Compute progress of each wheel
        long leftAbs = abs(leftTicks);
        long rightAbs = abs(rightTicks);

        //Stop each motor independently when its target reached
        if (leftAbs >= targetTicks) {
            mspeed(-40, 300);
            rotatingLeft = false;
        }
        if (rightAbs >= targetTicks) {
            mspeed(300, -40);
            rotatingRight = false;
        }
        if (!(rotatingLeft || rotatingRight)) break;

        // PID on difference in encoder counts
        error = (leftAbs - rightAbs);         // balance error
        integral += error;
        integral = constrain(integral, -500, 500);  // anti-windup
        float derivative = error - prevError;
        prevError = error;

        float correction = rotation_kp * error + rotation_ki * integral + rotation_kd * derivative;

        // Apply correction symmetrically
        int leftSpeed  = dir * (rotBaseSpeed - correction);
        int rightSpeed = -dir * (rotBaseSpeed + correction);

        // Constrain speeds
        leftSpeed  = constrain(leftSpeed, -rotMaxSpeed, rotMaxSpeed);
        rightSpeed = constrain(rightSpeed, -rotMaxSpeed, rotMaxSpeed);

        mspeed(leftSpeed, rightSpeed);
    }

    // Small counter-brake to rotation_kill inertia
    mspeed(-dir * 80, dir * 80);
    delay(5);
    mspeed(0, 0);
    Serial.printf("Target was %d, it traveled %d %d \n", targetTicks, leftTicks, rightTicks);
}
void setupMotorEncodersMore() {
    // Motor setup
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    // Encoder setup
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
    Serial.println("Motor Encoders initialised");

    pinMode(BTN1, INPUT);
    pinMode(BTN2, INPUT);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
}
void mspeed(int a, int b) {
    if (abs(a) <= 255) {
        digitalWrite(AIN1, a >= 0);
        digitalWrite(AIN2, a < 0);
        analogWrite(PWMA, abs(a));
    }

    if (abs(b) <= 255) {
        digitalWrite(BIN1, b >= 0);
        digitalWrite(BIN2, b < 0);
        analogWrite(PWMB, abs(b));
    }
}