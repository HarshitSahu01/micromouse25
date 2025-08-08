void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("Flash size: %d bytes\n", ESP.getFlashChipSize());
}

void loop() {
}
