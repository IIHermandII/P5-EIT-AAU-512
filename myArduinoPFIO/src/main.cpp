#include <Arduino.h>

const int analogPin = A3; // potentiometer wiper (middle terminal) connected to analog pin 3
const int overShootPin = 11;
const int underShootPin = 9;
int time_t = 0;
bool state = false; //true starte = overshoot | false state = undershoot
int lastSystemError = 1;


int getInput() {
  int input;
  input = map(analogRead(analogPin), 0, 1023, 0, 255);
  Serial.print("analog input value ----> ");
  Serial.println(input);
  return input;
}


float getRequestAngle(int angleRequest) {
  float requestValue;
  requestValue = (angleRequest * 360 / 255);
  Serial.print("requested value ----> ");
  Serial.println(requestValue);
  return requestValue;
}


int getError(int inputValue, float requestValue) {
  int error;
  error = requestValue - inputValue;
  Serial.print("error value ----> ");
  Serial.println(error);
  return error;
}


int getP(int K_p, int systemError) {
  int P;
  P = K_p * systemError;
  Serial.print("P value ----> ");
  Serial.println(P);
  return P;
}


int getI(int K_i, int K_c, int systemError) {
  int I;

  I = K_c * (systemError + (K_i * lastSystemError));
  Serial.print("I value ----> ");
  Serial.println(I);
  return I;
}


int getD(int K_d, int K_c, int systemError) {
  int D;
  D = K_c * (systemError - (K_d * lastSystemError));
  Serial.print("D value ----> ");
  Serial.println(D);
  lastSystemError = systemError;
  return D; // nice
}


void systemOutPID(int P, int I, int D){
  int PID;
  PID = P + I + D;
  if (PID >= 255) PID = 255;
  if (PID <= -255) PID = -255; 

  if (PID >=0 ) {
    analogWrite(overShootPin, PID);
    pinMode(underShootPin, LOW);
    }
  if (PID < 0 ) {
    analogWrite(underShootPin, PID);
    pinMode(overShootPin, LOW);
    } 
  }

  
void setup() {
  Serial.begin(9600);
  pinMode(overShootPin, OUTPUT); 
  pinMode(underShootPin, OUTPUT); 
}


void loop() {
  int inputValue;
  int requestValue;
  int systemError;
  int angleRequest = 90;
  int K_p = 0.4; // PID hvor P = K_p * e(t)
  int K_i = 0.6; // PID hvor I = K_i * intg0->t{ e(t) dt }
  int K_d = 0.2; // PID hvor D = K_d * diff{ e(t) }
  int K_c = 0.3; // P D controler parameter 
  int P;
  int I;
  int D;


  inputValue = getInput();  // gets the value from potensiometer and returns 0..255
  requestValue = getRequestAngle(angleRequest); // gets the angle from request and returns 0..255
  systemError = getError(inputValue, requestValue); // gets the system error
  P = getP(K_p, systemError); // gets P in PID
  I = getI(K_i, K_c, systemError); // gets I in PID
  D = getD(K_d, K_c, systemError); // gets D in PID
  systemOutPID(P, I, D);



  delay(500);
}