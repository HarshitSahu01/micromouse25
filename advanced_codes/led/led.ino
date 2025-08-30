#define LED1 0
#define LED2 12


void setup() {
    Serial.begin(115200);
    Serial.println("Started");
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
}

void loop () {
    digitalWrite(LED1, 1); 
    digitalWrite(LED2, 1);
    delay(500); 
    digitalWrite(LED1, 0); 
    digitalWrite(LED2, 0); 
    delay(500); 
}