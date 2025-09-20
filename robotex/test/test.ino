// Define the pins for the encoder you want to test
#define ENCODER_A_PIN 22
#define ENCODER_B_PIN 23

// This variable stores the encoder's position.
// 'volatile' is crucial because it's changed by an interrupt.
volatile long encoderTicks = 0;

// This is the Interrupt Service Routine (ISR).
// It's a special function that runs instantly when the interrupt pin changes state.
// On ESP32, 'IRAM_ATTR' makes the ISR run faster.
void IRAM_ATTR readEncoder() {
  // Check the B pin to determine the direction of rotation.
  if (digitalRead(ENCODER_A_PIN) == LOW) {
    // If B is LOW, the encoder is turning one way (e.g., clockwise)
    encoderTicks++;
  } else {
    // If B is HIGH, it's turning the other way (e.g., counter-clockwise)
    encoderTicks--;
  }
}

void setup() {
  // Start serial communication to print the results
  Serial.begin(115200);

  // Set the encoder pins as inputs with internal pull-up resistors.
  // This means you don't need external resistors if your encoder is open-collector.
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  // Attach the interrupt to the A pin.
  // It will call the 'readEncoder' function every time the pin goes from LOW to HIGH (RISING).
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), readEncoder, RISING);

  Serial.println("Encoder test started. Turn the encoder...");
}

void loop() {
  // Print the current encoder tick count every 200 milliseconds.
  Serial.print("Encoder Ticks: ");
  Serial.println(encoderTicks);
  delay(200);
}