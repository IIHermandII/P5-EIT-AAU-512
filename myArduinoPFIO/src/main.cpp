#include <Arduino.h>

// Define PID constants
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.1;

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
const float pwm_period = 50; // (10kHz)20 kHz PWM period in microseconds
unsigned long pre_micros = 0;
int pwmValue = 0;
int i, j = 0;
bool clock_wise;


// Interupt function
void encoder_counter (){
  if (clock_wise){
    encoder_count ++;
  }
  else{
    encoder_count --;
  }
}



void pwm(int pin, float duty){//duty [%] from 0-100 
  unsigned long current_micros = micros();
  
  if (current_micros - pre_micros >= ((pwm_period/100)*duty)){
    digitalWrite(pin, LOW);
    }
  if (current_micros - pre_micros >= pwm_period && duty != 0) {
    pre_micros = current_micros;
    digitalWrite(pin, HIGH);
  }
  else{
    digitalWrite(pin, LOW);
  }
}

float readSensor() {
  float encoder_read;
  encoder_read = analogRead(encoderPin); //
  if (encoder_read<encoder_threshold){
    if (encoder_state_change){
      encoder_state_change = 0;
    }
  }
  else{
    encoder_state_change = 1;
  }
  float angle = map((encoder_count%counts_per_rotation), 0, counts_per_rotation-1, 0, 359);
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
}

void setup() {
  Serial.begin(115200);
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoder_counter, CHANGE);

  pinMode(Red, OUTPUT);
  pinMode(Black, OUTPUT);
}

void loop() {
  input = readSensor();
  error = setpoint - input;
  float P = Kp * error;
  integral += Ki * error;
  float derivative = Kd * (error - last_error);
  output = P + integral + derivative;
  output = -125;
  controlActuator(output);
  last_error = error;
}
