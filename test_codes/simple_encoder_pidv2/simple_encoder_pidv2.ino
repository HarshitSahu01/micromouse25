#include <Arduino.h>

// ===== Motor Driver Pins =====
#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21

// ===== Encoder Pins =====
#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

// ===== Globals =====
volatile long leftTicks = 0;
volatile long rightTicks = 0;

float Kp = 0, Ki = 0, Kd = 0;
int basePWM = 150;

unsigned long lastSampleTime = 0;
int sampleInterval = 50; // ms

// For auto-tuning
float Ku = 0;
float Tu = 0;
bool oscillating = false;
unsigned long lastZeroCross = 0;
int zeroCrossCount = 0;

// ===== Encoder ISRs =====
void IRAM_ATTR leftISR() {
  int b = digitalRead(LEFT_ENC_B);
  if (b > 0) leftTicks++; else leftTicks--;
}
void IRAM_ATTR rightISR() {
  int b = digitalRead(RIGHT_ENC_B);
  if (b > 0) rightTicks++; else rightTicks--;
}

// ===== Motor Control =====
void setMotor(int pwmLeft, int pwmRight) {
  digitalWrite(STBY, HIGH);

  // Left motor
  if (pwmLeft >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    pwmLeft = -pwmLeft;
  }
  analogWrite(PWMA, constrain(pwmLeft, 0, 255));

  // Right motor
  if (pwmRight >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    pwmRight = -pwmRight;
  }
  analogWrite(PWMB, constrain(pwmRight, 0, 255));
}

// ===== ZNF Auto-Tune =====
void znfAutoTune() {
  Serial.println("Starting ZNF auto-tune...");
  delay(2000);

  for (float trialKp = 0.5; trialKp < 20; trialKp += 0.5) {
    Serial.print("Testing Kp = ");
    Serial.println(trialKp);

    // Reset
    leftTicks = rightTicks = 0;
    zeroCrossCount = 0;
    lastZeroCross = 0;
    oscillating = false;

    unsigned long start = millis();
    while (millis() - start < 5000) { // 5 sec trial
      // Simple proportional control
      long error = leftTicks - rightTicks;
      int correction = trialKp * error;

      int pwmL = basePWM - correction;
      int pwmR = basePWM + correction;

      setMotor(pwmL, pwmR);

      // Detect oscillation (sign change of error)
      static long lastError = 0;
      if ((error > 0 && lastError <= 0) || (error < 0 && lastError >= 0)) {
        if (!oscillating) {
          oscillating = true;
          zeroCrossCount = 0;
        } else {
          if (zeroCrossCount == 0) lastZeroCross = millis();
          else if (zeroCrossCount == 5) { // 5 crossings → stable oscillation
            Tu = (millis() - lastZeroCross) / 5.0 / 1000.0; // sec
            Ku = trialKp;
            goto done; // Found Ku, Tu
          }
          zeroCrossCount++;
        }
      }
      lastError = error;
      delay(20);
    }
  }
done:
  setMotor(0, 0);
  if (Ku > 0 && Tu > 0) {
    Serial.print("Ku = "); Serial.println(Ku);
    Serial.print("Tu = "); Serial.println(Tu);

    // Ziegler–Nichols PID
    Kp = 0.6 * Ku;
    Ki = 1.2 * Ku / Tu;
    Kd = 0.075 * Ku * Tu;

    Serial.println("== Tuned PID ==");
    Serial.print("Kp = "); Serial.println(Kp);
    Serial.print("Ki = "); Serial.println(Ki);
    Serial.print("Kd = "); Serial.println(Kd);
  } else {
    Serial.println("Failed to find Ku/Tu");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR, RISING);

  delay(2000);
  znfAutoTune();
}

void loop() {
  // After tuning, bot is idle. You can copy Kp, Ki, Kd into your main PID code.
}
