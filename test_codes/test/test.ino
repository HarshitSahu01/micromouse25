#define AIN1 17
#define AIN2 16
#define PWMA 4
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

// Encoder pins
#define LEFT_ENC_A 2
#define LEFT_ENC_B 15
#define RIGHT_ENC_A 22
#define RIGHT_ENC_B 23

volatile long leftTicks = 0;
volatile long rightTicks = 0;

float setpoint = 300; 
float leftInput = 0, rightInput = 0;
float leftError, rightError;
float leftOutput = 0, rightOutput = 0;

float leftLastError = 0, rightLastError = 0;
float leftIntegral = 0, rightIntegral = 0;
float leftDerivative = 0, rightDerivative = 0;

float Kp = 0, Ki = 0, Kd = 0;
float Ku = 70;  // <- YOU: Manually find this by observing oscillation
float Tu = 0.6; // <- YOU: Measure period in seconds

bool tuned = false;
unsigned long lastTime = 0;

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

void setup() {
  Serial.begin(115200);

  // Motor setup
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  // Encoder setup
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  Serial.println("Begin Ziegler-Nichols Tuning (Watch Serial Plotter)");
  delay(2000);

  // Start with Ki and Kd = 0
  Ki = 0;
  Kd = 0;

  // Increase Kp manually in loop() until oscillation starts
  Kp = Ku;  // Start with your guess
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  if (dt >= 0.1) {
    noInterrupts();
    long lt = leftTicks;
    long rt = rightTicks;
    leftTicks = 0;
    rightTicks = 0;
    interrupts();

    leftInput = lt / dt;
    rightInput = rt / dt;

    leftError = setpoint - leftInput;
    rightError = setpoint - rightInput;

    leftIntegral += leftError * dt;
    rightIntegral += rightError * dt;

    leftDerivative = (leftError - leftLastError) / dt;
    rightDerivative = (rightError - rightLastError) / dt;

    leftOutput = Kp * leftError + Ki * leftIntegral + Kd * leftDerivative;
    rightOutput = Kp * rightError + Ki * rightIntegral + Kd * rightDerivative;

    leftLastError = leftError;
    rightLastError = rightError;

    if (!tuned) {
      mspeed(leftOutput, rightOutput);
    } else {
      mspeed(0, 0);
    }

    // Print speeds for observation
    Serial.print("L_Spd: "); Serial.print(leftInput);
    Serial.print(" | R_Spd: "); Serial.print(rightInput);
    Serial.print(" | L_Out: "); Serial.print(leftOutput);
    Serial.print(" | R_Out: "); Serial.println(rightOutput);

    // Auto-tune after 15s of manual oscillation
    if (!tuned && millis() > 15000) {
      // After observing Ku and Tu, plug them in above
      // Compute final PID values
      Kp = 0.6 * Ku;
      Ki = 2 * Kp / Tu;
      Kd = Kp * Tu / 8;

      Serial.println("\n--- PID TUNING COMPLETE ---");
      Serial.print("Kp: "); Serial.println(Kp);
      Serial.print("Ki: "); Serial.println(Ki);
      Serial.print("Kd: "); Serial.println(Kd);
      Serial.println("Bot stopped. Use these values in your main control code.");

      tuned = true;
    }

    lastTime = now;
  }
}

void mspeed(int a, int b) {
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);

  // Left Motor (A)
  if (a >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, a);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -a);
  }

  // Right Motor (B)
  if (b >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, b);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -b);
  }
}