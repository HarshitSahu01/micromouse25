// reduced base
#define AIN1 17
#define AIN2 16
#define PWMA 4

#define BIN1 18
#define BIN2 19
#define PWMB 21

#define STBY 5

#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

#define BTN 35

volatile long leftTicksRaw = 0;
volatile long rightTicksRaw = 0;

long leftTicks = 0;
long rightTicks = 0;

float leftSpeed = 0;
float rightSpeed = 0;

// Default base speed (reduced from 200.0 to 120.0)
float targetSpeed = 120.0;  
float driftPenalty = 0.18;

// Slightly reduced PID gains for smoother control
float Kp = 35, Ki = 110, Kd = 3.0;

float leftErrSum = 0, rightErrSum = 0;
float lastLeftErr = 0, lastRightErr = 0;

unsigned long lastUpdate = 0;

// Flag for detecting turns (replace with actual maze logic)
bool isTurning = false;

// Encoder ISR
void IRAM_ATTR leftEncoderISR() {
  bool a = digitalRead(LEFT_ENC_A);
  bool b = digitalRead(LEFT_ENC_B);
  leftTicksRaw += (a == b) ? 1 : -1;
}
void IRAM_ATTR rightEncoderISR() {
  bool a = digitalRead(RIGHT_ENC_A);
  bool b = digitalRead(RIGHT_ENC_B);
  rightTicksRaw += (a == b) ? 1 : -1;
}

void setupMotorPins() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void setupEncoders() {
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);

  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
}

void setLeftMotor(int speed) {
  if (speed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, min(abs(speed), 255));
}

void setRightMotor(int speed) {
  if (speed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, min(abs(speed), 255));
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  setupEncoders();
  Serial.println("PID Motor Control Started");
  pinMode(BTN, INPUT); // Button 1
  while (digitalRead(BTN) == LOW) {
    // Wait for button press to start
  }
  Serial.println("Button pressed, starting control loop...");
  delay(2000);
}

void loop() {
  unsigned long now = millis();
  leftTicks = leftTicksRaw * driftPenalty;
  rightTicks = rightTicksRaw;

  // Stop condition
  if (rightTicks > 2000 || leftTicks > 2000) {
    Serial.println("Stopping bot...");
    setLeftMotor(0);
    setRightMotor(0);
    while (1) {
      Serial.print("Ticks L: "); Serial.print(leftTicks);
      Serial.print(" | R: "); Serial.println(rightTicks);
      delay(500);

      if (digitalRead(BTN) == 1) {
        leftTicksRaw = 0;
        rightTicksRaw = 0;
        delay(2000);
        break;
      }
    }
  }

  // Dynamic speed control
  if (isTurning) {
    targetSpeed = 100.0;  // Slow down for turns
  } else {
    targetSpeed = 120.0;  // Base speed for straights
  }

  if (now - lastUpdate >= 100) {
    lastUpdate = now;

    static long prevLeftTicks = 0;
    static long prevRightTicks = 0;

    long deltaLeft = leftTicks - prevLeftTicks;
    long deltaRight = rightTicks - prevRightTicks;

    prevLeftTicks = leftTicks;
    prevRightTicks = rightTicks;

    // PID for left motor
    float leftErr = targetSpeed - deltaLeft;
    leftErrSum += leftErr;
    float leftDeriv = leftErr - lastLeftErr;
    lastLeftErr = leftErr;

    float leftPWM = Kp * leftErr + Ki * leftErrSum + Kd * leftDeriv;

    // PID for right motor
    float rightErr = targetSpeed - deltaRight;
    rightErrSum += rightErr;
    float rightDeriv = rightErr - lastRightErr;
    lastRightErr = rightErr;

    float rightPWM = Kp * rightErr + Ki * rightErrSum + Kd * rightDeriv;

    // Apply motor speeds
    setLeftMotor((int)leftPWM);
    setRightMotor((int)rightPWM);

    // Debug
    Serial.print("Ticks L: "); Serial.print(deltaLeft);
    Serial.print(" | R: "); Serial.print(deltaRight);
    Serial.print(" | PWM L: "); Serial.print(leftPWM);
    Serial.print(" | PWM R: "); Serial.print(rightPWM);
    Serial.print(" | LeftTicks: "); Serial.print(leftTicks);
    Serial.print(" | RightTicks: "); Serial.println(rightTicks);
  }
}
