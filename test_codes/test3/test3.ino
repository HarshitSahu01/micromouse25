// #define BTN 35

// void setup() {
//   Serial.begin(115200);
//   pinMode(BTN, INPUT);
// }

// void loop () {
//   Serial.println(digitalRead(BTN));
// }

#define LED 0

void setup() {
  pinMode(LED, OUTPUT);
}

void loop () {
  digitalWrite(LED, 1);
}