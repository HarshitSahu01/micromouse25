// ------------------- Pin Definitions -------------------
#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

#define BUTTON_PIN 34  // Start button

// ------------------- Global Variables -------------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

float Kp = 1.678, Ki = 0.02, Kd = 0.146;
float errorSum = 0;
float lastError = 0;
unsigned long lastPIDTime = 0;

bool tuningStarted = false;

// ------------------- Interrupts -------------------
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

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  // Encoder pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // Button
  pinMode(BUTTON_PIN, INPUT);

  Serial.println("Ready. Press button (GPIO 34) to start PID tuning.");
}

// ------------------- Main Loop -------------------
void loop() {
  if (!tuningStarted && digitalRead(BUTTON_PIN) == HIGH) {
    tuningStarted = true;
    Serial.println("Starting Twiddle PID tuning...");
    twiddlePID(Kp, Ki, Kd);
    Serial.printf("Best PID: Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
  }

  if (tuningStarted) {
    static long lastLeft = 0, lastRight = 0;
    unsigned long now = millis();
    if (now - lastPIDTime >= 50) {
      long leftNow = leftTicks;
      long rightNow = rightTicks;

      long dLeft = leftNow - lastLeft;
      long dRight = rightNow - lastRight;
      long error = dLeft - dRight;

      errorSum += error;
      float dError = error - lastError;

      float correction = Kp * error + Ki * errorSum + Kd * dError;

      int baseSpeed = 200;
      int leftSpeed = constrain(baseSpeed - correction, 0, 255);
      int rightSpeed = constrain(baseSpeed + correction, 0, 255);

      mspeed(leftSpeed, rightSpeed);

      lastLeft = leftNow;
      lastRight = rightNow;
      lastError = error;
      lastPIDTime = now;
    }
  }
}

// ------------------- Motor Control -------------------
void mspeed(int a, int b) {
  // Left motor
  if (a >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, a);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -a);
  }

  // Right motor
  if (b >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, b);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -b);
  }
}

// ------------------- PID Helpers -------------------
void setPID(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  errorSum = 0;
  lastError = 0;
}

float runBalanceTest() {
  leftTicks = 0;
  rightTicks = 0;
  unsigned long start = millis();

  while (millis() - start < 2500) {
    loop();
    delay(10);
  }

  return abs(leftTicks - rightTicks);
}

void twiddlePID(float &Kp, float &Ki, float &Kd) {
  float p[3] = {Kp, Ki, Kd};
  float dp[3] = {0.2, 0.01, 0.1};
  float bestErr = runBalanceTest();

  for (int i = 0; i < 15; i++) {
    for (int j = 0; j < 3; j++) {
      p[j] += dp[j];
      setPID(p[0], p[1], p[2]);
      float err = runBalanceTest();

      if (err < bestErr) {
        bestErr = err;
        dp[j] *= 1.1;
      } else {
        p[j] -= 2 * dp[j];
        setPID(p[0], p[1], p[2]);
        err = runBalanceTest();

        if (err < bestErr) {
          bestErr = err;
          dp[j] *= 1.1;
        } else {
          p[j] += dp[j];
          dp[j] *= 0.9;
        }
      }
    }

    Serial.printf("Iter %d: Kp=%.3f Ki=%.3f Kd=%.3f Err=%.2f\n", i, p[0], p[1], p[2], bestErr);
  }

  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
}
