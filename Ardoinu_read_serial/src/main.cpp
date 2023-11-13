#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  pinMode(12, OUTPUT);
  // digitalWrite(12, LOW);
}

void loop() {
  if (Serial.available()) {
    // digitalWrite(12, HIGH);
    String RxedString = Serial.readString();  // Read the entire string
    if (RxedString == "180") {
      digitalWrite(12, HIGH);
      delay(1000);
      digitalWrite(12, LOW);
    }  
    
  }
}


