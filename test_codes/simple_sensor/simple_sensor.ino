#include <Wire.h>
#include <VL53L1X.h>

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

void setup() {
  Serial.begin(115200);

  // Use custom I2C pins: SDA = 33, SCL = 32
  Wire.begin(33, 32);

  // --- Step 1: Reset all sensors ---
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(100);

  // --- Step 2: Boot sensor1 only ---
  pinMode(XSHUT1, INPUT); // release -> pulled HIGH
  delay(10);
  if (!sensor1.init()) {
    Serial.println("Sensor1 init failed!");
    while (1);
  }
  sensor1.setAddress(ADDR1);

  // --- Step 3: Boot sensor2 ---
  pinMode(XSHUT2, INPUT);
  delay(10);
  if (!sensor2.init()) {
    Serial.println("Sensor2 init failed!");
    while (1);
  }
  sensor2.setAddress(ADDR2);

  // --- Step 4: Boot sensor3 ---
  pinMode(XSHUT3, INPUT);
  delay(10);
  if (!sensor3.init()) {
    Serial.println("Sensor3 init failed!");
    while (1);
  }
  sensor3.setAddress(ADDR3);

  // --- Step 5: Configure all sensors ---
  sensor1.setDistanceMode(VL53L1X::Medium);
  sensor1.setMeasurementTimingBudget(33000); // 33 ms
  sensor1.startContinuous(33);

  sensor2.setDistanceMode(VL53L1X::Medium);
  sensor2.setMeasurementTimingBudget(33000);
  sensor2.startContinuous(33);

  sensor3.setDistanceMode(VL53L1X::Medium);
  sensor3.setMeasurementTimingBudget(33000);
  sensor3.startContinuous(33);

  Serial.println("All sensors initialized.");
}

void loop() {
  int d1 = sensor1.read();
  int d2 = sensor2.read();
  int d3 = sensor3.read();

  Serial.print("S1: ");
  Serial.print(d1);
  Serial.print(" mm | S2: ");
  Serial.print(d2);
  Serial.print(" mm | S3: ");
  Serial.print(d3);
  Serial.println(" mm");

  delay(200);
}
