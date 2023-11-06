#include <Arduino.h>

// Define PID constants
float Kp = 10;
float Ki = 0.7;
float Kd = 0;

// Define variables
int target = 0;  
int input, output, error;
int last_error = 0;
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
const int encoder_Pin = 2;
const int encoder_2_Pin = 3;
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
   // int temp = analogRead(encoder_Pin); 
   // int temp2 = analogRead(encoder_2_Pin);
   // Serial.print("1:");
   // Serial.print(temp);
   // Serial.print(" 2:");
   // Serial.println(analogRead(temp2));
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

// float readSensor(int encoder) {
//   float encoder_read;
//   encoder_read = analogRead(encoder); //
//   // Serial.print("Encoder count : ");
//   float angle = map((encoder_count), 0, counts_per_rotation, 0, 359);
//   return angle;
// }

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

// void error_calc(float input, float target){
//   if (input > target){
//     if (input - target < counts_per_rotation/2){
//         error = -(input - target);
//     }
//     else{
//         error = counts_per_rotation - input + target;
//     }
//   }
//   else {
//       if(target-input < counts_per_rotation/2){
//         error = target-input;
//       }
//       else{
//         error = -(counts_per_rotation - target +  input);
//       }
//     }
// }

int mod( int x, int y ){
  int res;
  if(x<0){
    res = ((x+1)%y)+y-1;
  }
  else{
    res = x%y;
  }
  return res;
}


void error_calc(int input, int target){
  int diffrance = target - input;

  if (diffrance > counts_per_rotation/2){
    Serial.println("i was a big boi");
    diffrance = (-1)*diffrance; 
    error = -mod(diffrance, counts_per_rotation); 
  }
  else if(diffrance < -(counts_per_rotation/2)){
    error = mod(diffrance, counts_per_rotation); 
  }
  else{
    error = diffrance;
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
    
    if(step > 100){
      target = random(0,counts_per_rotation-1);
      step = 0;
    }
    
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
