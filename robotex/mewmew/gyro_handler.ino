// Required libraries for MPU6050 and I2C communication
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// MPU6050 object
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (based on DMP features enabled)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Global flag to indicate calibration status
volatile bool GYRO_CALIBRATED = false;

// Yaw variables
volatile float currentYaw = 0.0;
float yawOffset = 0.0;

// Task handle for the gyro task
TaskHandle_t gyroTaskHandle;

// ================================================================
// ===                      GYRO CORE 0 TASK                    ===
// ================================================================
// This function runs on Core 0. It now handles calibration first,
// then proceeds to continuously read the DMP.
void gyroTask(void *pvParameters) {
  // --- Step 1: Initialize the MPU6050 on this core ---
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed on Core 0!"));
    vTaskDelete(NULL); // End this task
    return;
  }

  // --- Step 2: Run calibration in the background ---
  Serial.println(F("Starting Gyro/Accel calibration on Core 0..."));
  // Place the sensor on a flat, stable surface before this runs
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println(F("\nCalibration complete!"));
  
  // --- Step 3: Initialize the DMP ---
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    // Set the global flag to true now that everything is ready
    GYRO_CALIBRATED = true; 
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    vTaskDelete(NULL); // End this task
    return;
  }

  // --- Step 4: Main Loop for Core 0 ---
  while (true) {
    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize) {
      vTaskDelay(pdMS_TO_TICKS(2)); // Non-blocking delay
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


// ================================================================
// ===                      INITIALIZATION                      ===
// ================================================================
// Call this function from your main setup() to start the process.
void gyroInit() {
  // Create the task that will run on Core 0. This task now handles
  // everything from initialization to calibration and reading.
  xTaskCreatePinnedToCore(
      gyroTask,         // Function to implement the task
      "GyroTask",       // Name of the task
      10000,            // Stack size in words
      NULL,             // Task input parameter
      1,                // Priority of the task
      &gyroTaskHandle,  // Task handle to keep track of created task
      0);               // Pin to core 0
}

// ================================================================
// ===                         YAW SET                          ===
// ================================================================
// Sets the current orientation to a desired angle.
void setYaw(float targetAngle) {
  if (GYRO_CALIBRATED) {
    yawOffset = targetAngle - currentYaw;
  }
}

// ================================================================
// ===                       YAW ACCESSOR                       ===
// ================================================================
// Call this function from your main loop() to get the latest yaw.
float readYaw() {
  // Apply the software offset
  float adjustedYaw = currentYaw + yawOffset;

  // Normalize the angle to be within -180 to +180 degrees
  while (adjustedYaw > 180.0) adjustedYaw -= 360.0;
  while (adjustedYaw < -180.0) adjustedYaw += 360.0;
  
  return adjustedYaw;
}