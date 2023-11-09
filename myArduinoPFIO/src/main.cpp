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
const int encoder_2_Pin = 8;
const int optical_sensor = 3; 
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
    clock_wise=true;
    if (encoder_count <= 0) {
      encoder_count = counts_per_rotation;
    }
    encoder_count --;
  }
  else{
    //Counter clockwise
    clock_wise=false;
    if (encoder_count >= counts_per_rotation) {
      encoder_count = 0;
    }
    encoder_count ++;
  }  
}

// Interrupt
void angle_reset(){
  //Serial.println("Factory reset");
  static int slot_width = 2;

  if(clock_wise){
    encoder_count = 0;
  }
  else{
    encoder_count = 0 + slot_width;
  }
  
}

// Controls the PWM signal to the motor
void control_actuator(float output) {
  int PID = output;
  int max_pwm = 255;
  if (PID >= max_pwm) PID = max_pwm;
  if (PID <= -max_pwm) PID = -max_pwm; 

  if (PID >=0) {
    analogWrite(Black, PID);
    pinMode(Red, LOW);
    }
  else {
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

void serial_read(){
  if(Serial.available()){
    target = map(Serial.readString().toInt(),0,359,0,177);
    Serial.print("New target set to: "); 
    Serial.println(target);
  }
}

void calibrate(){
  Serial.println("Calibrating");
  analogWrite(Black, 150);
  pinMode(Red, LOW);
  delay(1000);
  pinMode(Black, LOW);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello world");
  pinMode(encoder_Pin, INPUT);
  pinMode(encoder_2_Pin, INPUT);
  pinMode(optical_sensor, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(optical_sensor), angle_reset, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_Pin), encoder_counter, CHANGE);

  pinMode(Red, OUTPUT);
  pinMode(Black, OUTPUT);

  calibrate();
}

void loop() {
  time = millis();
  serial_read();
  if (time - last_time > periode) {
    step ++;
    //target += 10;

    // if (step==100){ 
    //   target=70;
    //   Serial.print("encoder count: ");
    //   Serial.println(input);
    //   Serial.print("target: ");
    //   Serial.println(target);
    // }
    if (step > 177){
      //target = 0;
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
