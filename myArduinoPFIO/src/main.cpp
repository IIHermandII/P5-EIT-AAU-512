#include <Arduino.h>

// Define PID constants
float Kp = 10;
float Ki = 0.7;
float Kd = 0;

// Define variables
float target = 0;  
float input, output, error;
float last_error = 0;
float integral = 0;
int encoder_count = 0;
const int counts_per_rotation = 178;
bool clock_wise;

// timing
int time;
int last_time = 0;
const int periode = 30;
int step=0;


// Define pins
const int encoderPin = 2;
const int Black = 11;
const int Red = 9;

struct PID_values {
  float P;
  float I;
  float D;
};

// Interupt function
void encoder_counter (){
  if (clock_wise){
    if (encoder_count == counts_per_rotation) {
      encoder_count = 0;
    }
    encoder_count ++;
  }
  else{
    if (encoder_count == 0) {
      encoder_count = counts_per_rotation;
    }
    encoder_count --;
  }
}

float readSensor() {
  float encoder_read;
  encoder_read = analogRead(encoderPin); //
  // Serial.print("Encoder count : ");
  float angle = map((encoder_count), 0, counts_per_rotation, 0, 359);
  return angle;
}

void controlActuator(float output) {
  int PID = output;
  int max_pwm = 255;
  if (PID >= max_pwm) PID = max_pwm;
  if (PID <= -max_pwm) PID = -max_pwm; 

  if (PID >=0 ) {
    clock_wise=true; //!problem med at tælle forkert ved retningsskift
    analogWrite(Black, PID);
    pinMode(Red, LOW);
    }
  else {
    clock_wise=false;
    analogWrite(Red, -PID);
    pinMode(Black, LOW);
    } 
  // Serial.print("PID : ");
  // Serial.println(PID);
}


float anti_windup(float i, float max){
  float value;
  if (i > max){
    value = max;
    Serial.println("hello");
  }
  else if (i < -max){
    value = -max;
  }
  else{
    value = i;
  }
  return value;
}

void error_calc(float input, float target){
  if (input > target){
    if (input - target < counts_per_rotation/2){
        error = -(input - target);
      
    }
    else{
        error = counts_per_rotation - input + target;

    }
  }
  else {
      if(target-input < counts_per_rotation/2){
        error = target-input;
      }
      else{
        error = -(counts_per_rotation - target +  input);
      }
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
  time = millis();
  if (time - last_time > periode) {
    // target += 10;
    
    // if (target > 178){
    //   target = 0;
    // }
    step ++;
    if (step==100){ 
      target=10;
      Serial.print("encoder count: ");
      Serial.println(input);
      Serial.print("target: ");
      Serial.println(target);
    }
    if(step==101){
      Serial.print("error: ");
      Serial.println(error);
    }
    if (step==200){ 
      target=counts_per_rotation/2;
      Serial.print("encoder count: ");
      Serial.println(input);
      Serial.print("target: ");
      Serial.println(target);
      }
    if(step==201){
      Serial.print("error: ");
      Serial.println(error);
    }  
    if (step==300){ 
      target=170;
      Serial.print("encoder count: ");
      Serial.println(input);
      Serial.print("target: ");
      Serial.println(target);
      }
    if(step==301){
      Serial.print("error: ");
      Serial.println(error);
    }
    if (step==400){ 
      target=counts_per_rotation/2;
      Serial.print("encoder count: ");
      Serial.println(input);
      Serial.print("target: ");
      Serial.println(target);
      }
    if(step==401){
      Serial.print("error: ");
      Serial.println(error);
    }
    if (step > 410){
      step = 0;
    }
    
    // input = readSensor();  //
    input = encoder_count;
    // Serial.print("encoder count: ");
    // Serial.println(input);
    // error = target - input;
    error_calc(input,target);
    float P = Kp * error;
    integral += Ki * error;

    integral = anti_windup(integral,10);

    float derivative = Kd * (error - last_error);    
    
    
    output = P + integral + derivative;
    Serial.print("P, I, D : ");
    Serial.print(P);
    Serial.print(" ");
    Serial.print(integral);
    Serial.print(" ");
    Serial.println(derivative);
    controlActuator(output);
    last_error = error;
    last_time=time;
  }
}
