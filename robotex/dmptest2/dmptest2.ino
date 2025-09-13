#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

// orientation/motion vars
Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

// calibration offsets
int16_t ax, ay, az, gx, gy, gz;

unsigned long lastPrint = 0;
const unsigned long printInterval = 500; // 33s

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Init I2C on custom pins
  Wire.begin(33, 32); 
  Wire.setClock(400000);

  Serial.println("Initializing MPU6050...");

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connected");

  // Run calibration
  Serial.println("Calibrating MPU6050... keep device still");
  delay(200);
  mpu.CalibrateAccel(20);
  mpu.CalibrateGyro(20);
  mpu.PrintActiveOffsets();

  // Load DMP firmware
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready!");
  } else {
    Serial.print("DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

void loop() {
  if (!dmpReady) return;

  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    mpu.resetFIFO(); // overflow
    Serial.println("FIFO overflow!");
    return;
  }

  if (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Get quaternion and YPR
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    unsigned long now = millis();
    if (now - lastPrint >= printInterval) {
      lastPrint = now;
      Serial.printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\n",
                    ypr[0] * 180 / M_PI,
                    ypr[1] * 180 / M_PI,
                    ypr[2] * 180 / M_PI);
    }
  }
}
