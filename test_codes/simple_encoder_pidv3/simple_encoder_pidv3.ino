// ===== CONFIG =====
const int targetTicksPerSec = 400;   // Target encoder ticks/sec
const float Kp = 1.678;
const float Ki = -0.020;
const float Kd = 0.146;
const int basePWM = 100;  // Base speed duty
const int maxPWM  = 255;  // Max duty

// ===== MOTOR PINS =====
const int motorL_PWM = 25;
const int motorL_DIR = 26;
const int motorR_PWM = 27;
const int motorR_DIR = 14;

// ===== ENCODER PINS =====
const int encL_A = 34;
const int encR_A = 35;

// ===== ENCODER VARIABLES =====
volatile long ticksL = 0;
volatile long ticksR = 0;

// ===== PID VARIABLES =====
float error_prev = 0;
float integral = 0;

// ===== TIMER =====
unsigned long lastTime = 0;
unsigned long pidInterval = 100; // ms

void IRAM_ATTR encL_ISR() {
  ticksL++;
}

void IRAM_ATTR encR_ISR() {
  ticksR++;
}

void setup() {
  Serial.begin(115200);

  pinMode(motorL_PWM, OUTPUT);
  pinMode(motorL_DIR, OUTPUT);
  pinMode(motorR_PWM, OUTPUT);
  pinMode(motorR_DIR, OUTPUT);

  pinMode(encL_A, INPUT_PULLUP);
  pinMode(encR_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encL_A), encL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encR_A), encR_ISR, RISING);

  lastTime = millis();

  Serial.println("PID Motor Control Starting...");
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= pidInterval) {
    lastTime = now;

    // Calculate ticks in last interval
    static long prevTicksL = 0;
    static long prevTicksR = 0;

    long currTicksL = ticksL;
    long currTicksR = ticksR;

    int deltaL = currTicksL - prevTicksL;
    int deltaR = currTicksR - prevTicksR;

    prevTicksL = currTicksL;
    prevTicksR = currTicksR;

    // Convert to ticks/sec
    float ticksPerSecL = (deltaL * 1000.0) / pidInterval;
    float ticksPerSecR = (deltaR * 1000.0) / pidInterval;

    // --- PID based on difference ---
    float speedError = (ticksPerSecL - ticksPerSecR); // balance
    integral += speedError * (pidInterval / 1000.0);
    float derivative = (speedError - error_prev) / (pidInterval / 1000.0);
    float correction = Kp * speedError + Ki * integral + Kd * derivative;
    error_prev = speedError;

    // --- Speed control to reach targetTicksPerSec ---
    float errorTargetL = targetTicksPerSec - ticksPerSecL;
    float errorTargetR = targetTicksPerSec - ticksPerSecR;

    int pwmL = basePWM + errorTargetL + correction;
    int pwmR = basePWM + errorTargetR - correction;

    // Constrain
    pwmL = constrain(pwmL, 0, maxPWM);
    pwmR = constrain(pwmR, 0, maxPWM);

    setMotor(motorL_PWM, motorL_DIR, pwmL, true);
    setMotor(motorR_PWM, motorR_DIR, pwmR, true);

    // Print info
    Serial.print("L_ticks/sec: "); Serial.print(ticksPerSecL);
    Serial.print(" | R_ticks/sec: "); Serial.print(ticksPerSecR);
    Serial.print(" | PWM_L: "); Serial.print(pwmL);
    Serial.print(" | PWM_R: "); Serial.println(pwmR);
  }
}

void setMotor(int pwmPin, int dirPin, int pwmVal, bool forward) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, pwmVal);
}
