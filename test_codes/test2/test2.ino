#define BTN1 34
#define BTN2 35

void setup() {
    Serial.begin(115200);
    pinMode(BTN1, INPUT);
    pinMode(BTN2, INPUT);
}

void loop() {
    Serial.print("BTN1: ");
    Serial.print(digitalRead(BTN1));
    Serial.print(" | BTN2: ");
    Serial.println(digitalRead(BTN2));
    
    delay(500); // Debounce delay
}