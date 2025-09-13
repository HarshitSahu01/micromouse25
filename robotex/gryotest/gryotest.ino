#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin(33, 32);  // SDA=33, SCL=32 for your ESP32

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);  // calibrates; keep IMU still
  Serial.println("MPU6050 ready!");
}

void loop() {
  mpu6050.update();

  // Read raw values
  float ax = mpu6050.getAccX();
  float ay = mpu6050.getAccY();
  float az = mpu6050.getAccZ();

  float gx = mpu6050.getGyroX();
  float gy = mpu6050.getGyroY();
  float gz = mpu6050.getGyroZ();

  float temp = mpu6050.getTemp();

  // Print using printf for compact debugging
  Serial.printf("ACC: X=%.2f\t Y=%.2f\t Z=%.2f\t | GYRO: X=%.2f \t Y=%.2f \t Z=%.2f\t | TEMP=%.2f C\n",
                ax, ay, az, gx, gy, gz, temp);
  // Serial.printf("GYRO: X=%.2f\t Y=%.2f  \t Z=%.2f\t | TEMP=%.2f C\n",
  //               gx, gy, gz, temp);

  delay(100); // adjust update rate
}
