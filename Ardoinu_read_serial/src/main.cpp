#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    
    String RxedString = Serial.readString();  // Read the entire string
    if (RxedString == "4") {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

int myFunction(int x, int y) {
  return x + y;
}
