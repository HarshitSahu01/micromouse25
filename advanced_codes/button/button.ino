#define BTN1 34
#define BTN2 35

void setup() {
    Serial.begin(115200);
    Serial.println("Started");
    pinMode(BTN1, INPUT);
    pinMode(BTN2, INPUT);
}

void loop () {
    Serial.printf("%d %d \n", digitalRead(BTN1), digitalRead(BTN2));
}