#include "driver/pcnt.h"

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

// ===== PCNT Units =====
#define PCNT_UNIT_LEFT  PCNT_UNIT_0
#define PCNT_UNIT_RIGHT PCNT_UNIT_1

volatile long leftTicks = 0;
volatile long rightTicks = 0;

int speedA = 255;
int speedB = 255;

// ==== Encoder setup with PCNT ====
void setupEncoder(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t pcntConfig = {};
  pcntConfig.pulse_gpio_num = pinA;   // Channel A
  pcntConfig.ctrl_gpio_num = pinB;    // Channel B
  pcntConfig.lctrl_mode = PCNT_MODE_REVERSE; // Reverse count if B = 0
  pcntConfig.hctrl_mode = PCNT_MODE_KEEP;    // Keep direction if B = 1
  pcntConfig.pos_mode = PCNT_COUNT_INC;      // Count up on rising A
  pcntConfig.neg_mode = PCNT_COUNT_DEC;      // Count down on falling A
  pcntConfig.counter_h_lim = 32767;
  pcntConfig.counter_l_lim = -32768;
  pcntConfig.unit = unit;
  pcntConfig.channel = PCNT_CHANNEL_0;

  pcnt_unit_config(&pcntConfig);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void setup() {
  Serial.begin(115200);

  // Motor setup
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  // Encoder setup with PCNT
  setupEncoder(PCNT_UNIT_LEFT, LEFT_ENC_A, LEFT_ENC_B);
  setupEncoder(PCNT_UNIT_RIGHT, RIGHT_ENC_A, RIGHT_ENC_B);

  Serial.println("Ready. Use 'a+200', 'b-150' to control motors.");
}

void loop() {
  // ===== Motor speed control from Serial =====
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

  // ===== Read encoder counts every 200ms =====
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    int16_t leftVal, rightVal;
    pcnt_get_counter_value(PCNT_UNIT_LEFT, &leftVal);
    pcnt_get_counter_value(PCNT_UNIT_RIGHT, &rightVal);

    leftTicks = leftVal;
    rightTicks = rightVal;

    Serial.print("Left Ticks: ");
    Serial.print(leftTicks);
    Serial.print("  Right Ticks: ");
    Serial.println(rightTicks);
  }
}

// ===== Motor speed control =====
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
