#include <Wire.h>

#define MPU9250_ADDR 0x68
#define AK8963_ADDR  0x0C

// MPU9250 registers
#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B
#define INT_PIN_CFG    0x37

// AK8963 registers
#define AK8963_ST1     0x02
#define AK8963_XOUT_L  0x03
#define AK8963_CNTL1   0x0A

void setup() {
  Serial.begin(115200);
  Wire.begin(33, 32);  // SDA, SCL

  // Wake up MPU9250
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);

  // Enable I2C bypass to access magnetometer directly
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission();

  // Configure AK8963: 16-bit output, continuous measurement mode 2
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(AK8963_CNTL1);
  Wire.write(0x16);
  Wire.endTransmission();

  Serial.println("MPU9250 (Accel + Gyro + Mag) Raw Data");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz, tempRaw;
  int16_t mx, my, mz;

  // ----- Read Accel + Temp + Gyro -----
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  tempRaw = (Wire.read() << 8) | Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  // ----- Read Magnetometer -----
  // Wait for data ready
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(AK8963_ST1);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, 1);
  if (Wire.available() && (Wire.read() & 0x01)) {
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(AK8963_XOUT_L);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8963_ADDR, 7);

    if (Wire.available() >= 7) {
      mx = (Wire.read() | (Wire.read() << 8));
      my = (Wire.read() | (Wire.read() << 8));
      mz = (Wire.read() | (Wire.read() << 8));
      Wire.read(); // ST2 register (must be read)
    }
  }

  // ----- Print Raw Data -----
  Serial.printf("Accel: %d %d %d | Gyro: %d %d %d | Mag: %d %d %d | Temp: %d\n",
                ax, ay, az, gx, gy, gz, mx, my, mz, tempRaw);

  delay(200);
}
