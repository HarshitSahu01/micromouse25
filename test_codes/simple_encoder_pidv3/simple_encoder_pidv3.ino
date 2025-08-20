// ========== ENCODER PID WITH TWIDDLE AUTO TUNER ==========

// --- Motor driver pins (replace with yours) ---
#define AIN1 17
#define AIN2 16
#define PWMA 4
#define BIN1 19
#define BIN2 18
#define PWMB 21
#define STBY 5

// --- Encoder pins (replace with yours) ---
#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

// --- PID params ---
float Kp = 1.0, Ki = 0.0, Kd = 0.1;
float dp[3] = {0.5, 0.01, 0.1};
float best_err = 1e9;

// --- Encoder counters ---
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// --- Target speed (ticks per interval) ---
int targetTicks = 50;

// --- PID state ---
float integralL = 0, lastErrL = 0;
float integralR = 0, lastErrR = 0;

// ================== ENCODER ISR ==================
void IRAM_ATTR leftEncISR() {
  int b = digitalRead(LEFT_ENC_B);
  if (b > 0) leftTicks++; else leftTicks--;
}

void IRAM_ATTR rightEncISR() {
  int b = digitalRead(RIGHT_ENC_B);
  if (b > 0) rightTicks++; else rightTicks--;
}

// ================== MOTOR DRIVE ==================
void driveMotorA(int speed) {
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    speed = -speed;
  }
  analogWrite(PWMA, constrain(speed, 0, 255));
}

void driveMotorB(int speed) {
  if (speed >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    speed = -speed;
  }
  analogWrite(PWMB, constrain(speed, 0, 255));
}

// ================== PID CONTROL ==================
float pidControl(float Kp, float Ki, float Kd,
                 float &integral, float &lastErr,
                 int target, int actual) {
  float error = target - actual;
  integral += error;
  float derivative = error - lastErr;
  lastErr = error;
  return Kp * error + Ki * integral + Kd * derivative;
}

// ================== RUN ONE TEST ==================
float runTest(float Kp, float Ki, float Kd) {
  integralL = integralR = 0;
  lastErrL = lastErrR = 0;
  long lastLeft = leftTicks;
  long lastRight = rightTicks;
  float totalError = 0;

  for (int i = 0; i < 50; i++) { // ~50 cycles
    delay(100); // 100ms interval

    long curLeft = leftTicks;
    long curRight = rightTicks;

    int speedL = curLeft - lastLeft;
    int speedR = curRight - lastRight;

    lastLeft = curLeft;
    lastRight = curRight;

    float outL = pidControl(Kp, Ki, Kd, integralL, lastErrL, targetTicks, speedL);
    float outR = pidControl(Kp, Ki, Kd, integralR, lastErrR, targetTicks, speedR);

    driveMotorA((int)outL);
    driveMotorB((int)outR);

    totalError += abs(targetTicks - speedL) + abs(targetTicks - speedR);
  }
  return totalError;
}

// ================== TWIDDLE ==================
void twiddle() {
  float params[3] = {Kp, Ki, Kd};
  float tolerance = dp[0] + dp[1] + dp[2];
  if (tolerance < 0.001) return;

  for (int i = 0; i < 3; i++) {
    params[i] += dp[i];
    float err = runTest(params[0], params[1], params[2]);

    if (err < best_err) {
      best_err = err;
      dp[i] *= 1.1;
    } else {
      params[i] -= 2 * dp[i];
      err = runTest(params[0], params[1], params[2]);
      if (err < best_err) {
        best_err = err;
        dp[i] *= 1.1;
      } else {
        params[i] += dp[i];
        dp[i] *= 0.9;
      }
    }
  }
  Kp = params[0]; Ki = params[1]; Kd = params[2];
  Serial.printf("New PID: Kp=%.4f Ki=%.4f Kd=%.4f | Best Err=%.4f\n", Kp, Ki, Kd, best_err);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncISR, RISING);

  Serial.println("Starting Twiddle tuner...");
  best_err = runTest(Kp, Ki, Kd);
}

// ================== LOOP ==================
void loop() {
  twiddle();
  delay(500);
}
