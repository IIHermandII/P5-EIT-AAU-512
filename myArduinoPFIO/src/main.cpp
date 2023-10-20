#include <Arduino.h>

// Define PID constants
float Kp = 1;
float Ki = 0.05;
float Kd = 0.05;

// Define variables
float setpoint = 90;  // Desired temperature
float input, output, error;
float last_error = 0;
float integral = 0;
int encoder_count = 0;
int encoder_threshold = 1000;
bool encoder_state_change = 0;
int counts_per_rotation = 178;
bool print_one = true;

// Define pins
const int encoderPin = 2;
const int Black = 11;
const int Red = 9;

// pwm global variables
bool clock_wise;

struct PID_values {
  float P;
  float I;
  float D;
};

// Interupt function
void encoder_counter (){

  if (clock_wise){
    if (encoder_count == 178) {
      encoder_count = 0;
    }
    encoder_count ++;
  }
  else{
    if (encoder_count == 0) {
      encoder_count = 178;
    }
    encoder_count --;
  }
}

float readSensor() {
  float encoder_read;
  encoder_read = analogRead(encoderPin); //
  Serial.print("Encoder count : ");
  float angle = map((encoder_count), 0, counts_per_rotation, 0, 359);
  return angle;
}

void controlActuator(float output) {
  int PID = output;
  int max_pwm = 255;
  if (PID >= max_pwm) PID = max_pwm;
  if (PID <= -max_pwm) PID = -max_pwm; 

  if (PID >=0 ) {
    clock_wise=true;
    analogWrite(Black, PID);
    pinMode(Red, LOW);
    }
  else {
    clock_wise=false;
    analogWrite(Red, -PID);
    pinMode(Black, LOW);
    } 
  Serial.print("PID : ");
  Serial.println(PID);
}

PID_values anti_overflow(float P, float I, float D){
  PID_values holder;
float arr[3] ={P,I,D};
for (int i=0; i<3; i++){
  if (arr[i] > 255){
    arr[i] = 255;
  }
  if (arr[i] < -255){
    arr[i] = -255;
  }
}
  holder.P = arr[0];
  holder.I = arr[1];
  holder.D = arr[2];
return holder;
}

void setup() {
  Serial.begin(115200);
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoder_counter, CHANGE);

  pinMode(Red, OUTPUT);
  pinMode(Black, OUTPUT);
}

void loop() {
  input = readSensor();  //
  //Serial.print("angle : ");
  //Serial.println(input);
  error = setpoint - input;
  float P = Kp * error;

  integral += Ki * error;  // fix over flov !!!!!!


  float derivative = Kd * (error - last_error);    
  output = P + integral + derivative;
   PID_values result = anti_overflow(P, integral, derivative);
   P = result.P;
   integral = result.I;
   derivative = result.D;
  // Serial.print("P, I, D : ");
  // Serial.print(P);
  // Serial.print(" ");
  // Serial.print(integral);
  // Serial.print(" ");
  // Serial.println(derivative);
  controlActuator(output);
  last_error = error;
}
