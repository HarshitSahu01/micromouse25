#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

// Variables to hold average sensor readings
long accelX_sum = 0, accelY_sum = 0, accelZ_sum = 0;
long gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(33, 32);
  mpu.initialize();

  // Verify connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  delay(1000);
  Serial.println("Calibrating... Please wait.");
  
  // Take a large number of readings to average out noise
  int num_readings = 2000;
  for (int i = 0; i < num_readings; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accelX_sum += ax;
    accelY_sum += ay;
    accelZ_sum += az;
    gyroX_sum += gx;
    gyroY_sum += gy;
    gyroZ_sum += gz;
    delay(2); // Small delay between readings
  }

  // Calculate the average readings
  int16_t accelX_offset = accelX_sum / num_readings;
  int16_t accelY_offset = accelY_sum / num_readings;
  int16_t accelZ_offset = accelZ_sum / num_readings;
  int16_t gyroX_offset = gyroX_sum / num_readings;
  int16_t gyroY_offset = gyroY_sum / num_readings;
  int16_t gyroZ_offset = gyroZ_sum / num_readings;

  // The Z accelerometer reading should be 1G (16384), so we need to adjust for that
  // This is a simplified approach. For perfect calibration, the sensor would be oriented on all 6 axes.
  accelZ_offset = accelZ_offset - 16384;

  // Print the results in a copy-paste format
  Serial.println("\nCalibration complete!");
  Serial.println("Paste the following lines into your gyroTask() function:");
  Serial.println("-----------------------------------------------------");
  Serial.print("mpu.setXAccelOffset("); Serial.print(accelX_offset); Serial.println(");");
  Serial.print("mpu.setYAccelOffset("); Serial.print(accelY_offset); Serial.println(");");
  Serial.print("mpu.setZAccelOffset("); Serial.print(accelZ_offset); Serial.println(");");
  Serial.print("mpu.setXGyroOffset("); Serial.print(gyroX_offset); Serial.println(");");
  Serial.print("mpu.setYGyroOffset("); Serial.print(gyroY_offset); Serial.println(");");
  Serial.print("mpu.setZGyroOffset("); Serial.print(gyroZ_offset); Serial.println(");");
  Serial.println("-----------------------------------------------------");
}

void loop() {
  // Nothing to do here
}