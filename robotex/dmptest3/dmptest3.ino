#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// DMP variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
int epoch = 30;

// Orientation
Quaternion q;
VectorFloat gravity;
float ypr[3]; // yaw, pitch, roll

// Shared data (volatile because used across tasks)
volatile float sharedYPR[3] = {0, 0, 0};

// Task handles
TaskHandle_t TaskMPU, TaskSerial;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Init I2C
  Wire.begin(33, 32);
  Wire.setClock(400000);

  // Init MPU
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Load DMP firmware
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    Serial.printf("DMP init failed (code %d)\n", devStatus);
    while (1);
  }

  mpu.CalibrateAccel(epoch);
  mpu.CalibrateGyro(epoch);
  mpu.PrintActiveOffsets();

  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;

  packetSize = mpu.dmpGetFIFOPacketSize();

  // Create tasks
  xTaskCreatePinnedToCore(TaskMPUcode, "TaskMPU", 4096, NULL, 2, &TaskMPU, 0);   // Core 0
  xTaskCreatePinnedToCore(TaskSerialcode, "TaskSerial", 4096, NULL, 1, &TaskSerial, 1); // Core 1
}

void loop() {
  // Nothing here, tasks do the work
}

// Task: read MPU6050 DMP packets
void TaskMPUcode(void *pvParameters) {
  while (true) {
    if (!dmpReady) continue;

    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize) continue;

    // Check for overflow
    if (fifoCount >= 1024) {
      mpu.resetFIFO();
      continue;
    }

    // Read latest packet
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    // Compute YPR
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Update shared data
    sharedYPR[0] = ypr[0] * 180 / M_PI; // yaw
    sharedYPR[1] = ypr[1] * 180 / M_PI; // pitch
    sharedYPR[2] = ypr[2] * 180 / M_PI; // roll
  }
}

// Task: print YPR every 100ms
void TaskSerialcode(void *pvParameters) {
  while (true) {
    Serial.printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\n",
                  sharedYPR[0], sharedYPR[1], sharedYPR[2]);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
