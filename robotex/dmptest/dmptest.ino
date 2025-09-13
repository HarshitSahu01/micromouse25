#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define INTERRUPT_PIN 14  // MPU6050 INT pin to ESP32 GPIO4

MPU6050 mpu;

volatile bool mpuInterrupt = false;   // Flag set by ISR
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

unsigned long lastPrint = 0;
const uint16_t PRINT_INTERVAL = 100;  // ms

// Latest read data (updated inside ISR handler)
float latestYaw = 0, latestPitch = 0, latestRoll = 0;

void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(33, 32);
  Wire.setClock(400000);

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  uint8_t devStatus = mpu.dmpInitialize();

  // Optional: Set your gyro/accel offsets here (calibration)
  // mpu.setXAccelOffset(...);
  // mpu.setYAccelOffset(...);
  // mpu.setZAccelOffset(...);
  // mpu.setXGyroOffset(...);
  // mpu.setYGyroOffset(...);
  // mpu.setZGyroOffset(...);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready and interrupt attached.");
  } else {
    Serial.print("DMP Initialization failed with code: ");
    Serial.println(devStatus);
    while (1);
  }
}

void loop() {
  if (mpuInterrupt) {
    mpuInterrupt = false;

    uint16_t fifoCount = mpu.getFIFOCount();

    // Handle overflow
    if (fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println("FIFO overflow!");
      return;
    }

    // Read all packets in FIFO to catch up
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // Process quaternion, gravity, YPR
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      latestYaw = ypr[0] * 180.0 / M_PI;
      latestPitch = ypr[1] * 180.0 / M_PI;
      latestRoll = ypr[2] * 180.0 / M_PI;

      if (latestYaw < 0) latestYaw += 360;

      fifoCount = mpu.getFIFOCount();  // Update count after reading
    }
  }

  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL) {
    lastPrint = now;

    Serial.print("Yaw: ");
    Serial.print(latestYaw, 2);
    Serial.print(" | Pitch: ");
    Serial.print(latestPitch, 2);
    Serial.print(" | Roll: ");
    Serial.println(latestRoll, 2);
  }
}
