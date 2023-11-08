#include <Arduino.h>

// Define PID constants
float Kp = 10;
float Ki = 0.7;
float Kd = 0;

// Define variables
int target = 0;  
int input, output, error;
int last_error = 0;
float I = 0;
int encoder_count = 0;
const int counts_per_rotation = 178;
bool clock_wise;

// timing
int time;
int last_time = 0;
const int periode = 30;
int step=0;

// Define pins
const int encoder_Pin = 2;
const int encoder_2_Pin = 3;
const int Black = 11;
const int Red = 9;

// Interupt function which counts encoder steps
void encoder_counter (){
  static bool A;
  static bool B;

  A = digitalRead(encoder_Pin);
  B = digitalRead(encoder_2_Pin);

  //XOR OF A, B
  if(A != B){
    //Clockwise
    if (encoder_count <= 0) {
      encoder_count = counts_per_rotation-1;
    }
    encoder_count --;
  }
  else{
    //Counter clockwise
    if (encoder_count >= counts_per_rotation-1) {
      encoder_count = 0;
    }
    encoder_count ++;
  }  
}

// Controls the PWM signal to the motor
void control_actuator(float output) {
  int PID = output;
  int max_pwm = 255;
  if (PID >= max_pwm) PID = max_pwm;
  if (PID <= -max_pwm) PID = -max_pwm; 

  if (PID >=0) {
    //clock_wise=true;
    analogWrite(Black, PID);
    pinMode(Red, LOW);
    }
  else {
    //clock_wise=false;
    analogWrite(Red, -PID);
    pinMode(Black, LOW);
    } 
}

// Ensures propper szie limitation of PID values
float anti_windup(float i, float max){
  float value;
  if (i > max){
    value = max;
  }
  else if (i < -max){
    value = -max;
  }
  else{
    value = i;
  }
  return value;
}

//Calculates error and determis the shortest path
void error_calc(int input, int target){
  error = target - input;
  if (error > counts_per_rotation/2) {
    error -= counts_per_rotation; // If error is greater than 89, subtract 178 to get the shortest path.
  } else if (error < -counts_per_rotation/2) {
    error += counts_per_rotation; // If error is less than -89, add 178 to get the shortest path.
  }
} 

void setup() {
  Serial.begin(115200);
  pinMode(encoder_Pin, INPUT);
  pinMode(encoder_2_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_Pin), encoder_counter, CHANGE);

  pinMode(Red, OUTPUT);
  pinMode(Black, OUTPUT);
}

void loop() {
  time = millis();
  if (time - last_time > periode) {
    step ++;
    

    if (step==100){ 
      target=70;
      Serial.print("encoder count: ");
      Serial.println(input);
      Serial.print("target: ");
      Serial.println(target);
    }
    if (step > 200){
      target = 0;
      step = 0;
    }

    input = encoder_count;
    error_calc(input,target);
    
    static float P;
    static float D;

    P = Kp * error;
    I += Ki * error;
    I = anti_windup(I,10);
    D = Kd * (error - last_error);    
    
    output = P + I + D;
    // Serial.print("P, I, D : ");
    // Serial.print(P);
    // Serial.print(" ");
    // Serial.print(I);
    // Serial.print(" ");
    // Serial.println(D);
    control_actuator(output);
    last_error = error;
    last_time=time;
  }
}
