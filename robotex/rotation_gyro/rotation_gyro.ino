// Required libraries for MPU6050 and I2C communication
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// ================================================================
// ===                      PIN DEFINITIONS                     ===
// ================================================================
// Motor Driver Pins
#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21

// Encoder Pins
#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

// ================================================================
// ===                GLOBAL VARIABLES & CONSTANTS              ===
// ================================================================
// Volatile variables for Interrupt Service Routines (ISRs)
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// --- Physical/Calibration Constants ---
// This factor converts a target degree into a target number of encoder ticks.
// You will need to tune this for your specific robot's wheels and gear ratio.
// A good starting point: (TicksPerRevolution / 360) * (WheelTrack_cm / WheelDiameter_cm)
float encoderCountToDegrees = 0.89;
const int rotationAxisCorrection = 3; // Correction for 180-degree turns.

// --- Speed Constants ---
int rotBaseSpeed = 120; // Base speed for rotation
int rotMaxSpeed = 200;  // Maximum allowable speed during rotation

// --- PID Constants ---
// For Encoder Balancing (keeping the turn tight)
float rotation_kp = 2.0;
float rotation_ki = 0.002;
float rotation_kd = 1.5;

// For Gyro Correction (ensuring angular accuracy)
float yaw_kp = 3.0;
float yaw_ki = 0.02;
float yaw_kd = 2.5;

// ================================================================
// ===                  GYRO HANDLER CODE                       ===
// ================================================================
// (This is the required code from your gyro_handler.ino)

MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool GYRO_CALIBRATED = false;
volatile float currentYaw = 0.0;
float yawOffset = 0.0;
TaskHandle_t gyroTaskHandle;

void gyroTask(void *pvParameters) {
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println(F("MPU6050 connection failed on Core 0!"));
        vTaskDelete(NULL);
        return;
    }
    Serial.println(F("Starting Gyro/Accel calibration on Core 0..."));
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println(F("\nCalibration complete!"));
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        GYRO_CALIBRATED = true;
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        vTaskDelete(NULL);
        return;
    }
    while (true) {
        fifoCount = mpu.getFIFOCount();
        if (fifoCount < packetSize) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (fifoCount >= 1024) {
            mpu.resetFIFO();
            continue;
        }
        while (fifoCount >= packetSize) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        currentYaw = ypr[0] * 180.0 / M_PI;
    }
}

void gyroInit() {
    xTaskCreatePinnedToCore(gyroTask, "GyroTask", 10000, NULL, 1, &gyroTaskHandle, 0);
}

void setYaw(float targetAngle) {
    if (GYRO_CALIBRATED) {
        yawOffset = targetAngle - currentYaw;
    }
}

float readYaw() {
    float adjustedYaw = currentYaw + yawOffset;
    while (adjustedYaw > 180.0) adjustedYaw -= 360.0;
    while (adjustedYaw < -180.0) adjustedYaw += 360.0;
    return adjustedYaw;
}


// ================================================================
// ===             INTERRUPT SERVICE ROUTINES (ISRs)            ===
// ================================================================
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

// ================================================================
// ===                   MOTOR CONTROL FUNCTION                 ===
// ================================================================
void mspeed(int leftSpeed, int rightSpeed) {
    // Control Left Motor
    if (leftSpeed >= 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    analogWrite(PWMA, abs(leftSpeed));

    // Control Right Motor
    if (rightSpeed >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }
    analogWrite(PWMB, abs(rightSpeed));
}


// ================================================================
// ===               HYBRID ROTATION FUNCTION (to be tested)    ===
// ================================================================
void rotate(int degree) {
    if (degree == 0) return; // No rotation needed
    
    // This is a placeholder for a variable that was in your maze code.
    // For this test, it doesn't need to do anything.
    int blockSize = 18; 
    if (abs(degree) == 180) blockSize -= rotationAxisCorrection;

    // 1. Set up targets for both encoders and the gyro
    long targetTicks = encoderCountToDegrees * abs(degree);
    int dir = (degree > 0) ? 1 : -1;

    float startYaw = readYaw();
    float targetYaw = startYaw + degree;

    // Normalize targetYaw to be within -180 to +180
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    // Reset encoders safely
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();

    // Reset PID state variables
    float yawError = 0, yawPrevError = 0, yawIntegral = 0;
    float encError = 0, encPrevError = 0, encIntegral = 0;

    // 2. Main loop runs until the encoder target is met
    while ((abs(leftTicks) + abs(rightTicks)) / 2 < targetTicks) {

        // --- 3. Calculate Gyro PID for angular accuracy ---
        float currentYaw = readYaw();
        
        // Calculate the shortest angle difference to handle wrapping (-180/180)
        yawError = targetYaw - currentYaw;
        if (yawError > 180.0f) yawError -= 360.0f;
        if (yawError < -180.0f) yawError += 360.0f;

        yawIntegral += yawError;
        yawIntegral = constrain(yawIntegral, -3000, 3000); // Anti-windup
        float yawDerivative = yawError - yawPrevError;
        yawPrevError = yawError;

        float yawCorrection = yaw_kp * yawError + yaw_ki * yawIntegral + yaw_kd * yawDerivative;

        // --- 4. Calculate Encoder Balance PID for tight turns (optional but recommended) ---
        encError = abs(leftTicks) - abs(rightTicks);
        encIntegral += encError;
        encIntegral = constrain(encIntegral, -500, 500);
        float encDerivative = encError - encPrevError;
        encPrevError = encError;

        float encBalanceCorrection = rotation_kp * encError + rotation_ki * encIntegral + rotation_kd * encDerivative;

        // --- 5. Combine corrections and apply to motors ---
        float totalCorrection = yawCorrection + encBalanceCorrection;

        int leftSpeed  = dir * (rotBaseSpeed - totalCorrection);
        int rightSpeed = -dir * (rotBaseSpeed + totalCorrection);

        leftSpeed  = constrain(leftSpeed, -rotMaxSpeed, rotMaxSpeed);
        rightSpeed = constrain(rightSpeed, -rotMaxSpeed, rotMaxSpeed);

        mspeed(leftSpeed, rightSpeed);
        delay(2); // Small delay for stability
    }

    // Small counter-brake to kill inertia and prevent overshoot
    mspeed(-dir * 80, dir * 80);
    delay(10);
    mspeed(0, 0);
    Serial.printf("Target Yaw: %.2f, Final Yaw: %.2f\n", targetYaw, readYaw());
}


// ================================================================
// ===                         SETUP                            ===
// ================================================================
void setup() {
    Serial.begin(115200);
    
    // Initialize I2C for the gyro using your custom pins
    Wire.begin(33, 32);
    Wire.setClock(400000);

    // Start the gyro initialization and calibration on Core 0
    gyroInit();
    
    // Setup motor driver pins
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
    
    // Setup encoder pins
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

    Serial.println("Waiting for gyro calibration...");
    while (!GYRO_CALIBRATED) {
        delay(100);
    }
    Serial.println("Gyro calibrated!");
    
    // Set the initial "forward" direction to 0 degrees.
    // NOTE: The DMP yaw often starts around 90 or -90. setYaw(0) corrects this.
    setYaw(0);
    delay(100); // Give it a moment to settle

    Serial.println("\n--- Rotation Test Ready ---");
    Serial.println("Enter an angle (e.g., 90, -90, 180) in the Serial Monitor and press Enter.");
}

// ================================================================
// ===                    MAIN TEST LOOP                        ===
// ================================================================
void loop() {
    // Check if there is data available to read from the Serial Monitor
   rotate(90);
    delay(2000);
   rotate(-90);
    delay(2000);
}