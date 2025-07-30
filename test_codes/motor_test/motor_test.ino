#define AIN1 16
#define AIN2 17
#define PWMA 4
#define STBY 5
#define BIN1 19
#define BIN2 18
#define PWMB 21

// Encoder pins
#define LEFT_ENC_A 23
#define LEFT_ENC_B 22
#define RIGHT_ENC_A 15 
#define RIGHT_ENC_B 2

volatile long leftTicks = 0;
volatile long rightTicks = 0;

int speedA = 0;
int speedB = 0;

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

  Serial.println("Ready. Use 'a+200', 'b-150' to control motors.");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 1) {
      char motor = input.charAt(0);
      int val = input.substring(1).toInt();
      if (motor == 'a') {
        speedA = constrain(val, -255, 255);
      } else if (motor == 'b') {
        speedB = constrain(val, -255, 255);
      }
    }
  }

  mspeed(speedA, speedB);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print("Left Ticks: ");
    Serial.print(leftTicks);
    Serial.print("  Right Ticks: ");
    Serial.println(rightTicks);
  }
}

void mspeed(int a, int b) {
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
