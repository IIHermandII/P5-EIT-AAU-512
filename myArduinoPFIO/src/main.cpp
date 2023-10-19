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
  // Serial.print("count: ");
  // Serial.println(encoder_count);
  // Serial.print("Read: ");
  // Serial.println(analogRead(analogPin));
}



void pwm(int pin, float duty){//duty [%] from 0-100 
  unsigned long current_micros = micros();
  
  if (current_micros - pre_micros >= ((pwm_period/100)*duty)){
    digitalWrite(pin, LOW);
    }

  // Check if it's time to update the PWM signal
  if (current_micros - pre_micros >= pwm_period && duty != 0) {
    pre_micros = current_micros;
    digitalWrite(pin, HIGH);
  }
  else{
    digitalWrite(pin, LOW);
  }
  // Serial.println(((pwm_period/100)*duty));
}



float readSensor() {
  float encoder_read;
  encoder_read = analogRead(encoderPin); //
  // Serial.print("Encoder sensor input value ----> ");
  // Serial.println(encoder_read);
  if (encoder_read<encoder_threshold){
    if (encoder_state_change){
      // encoder_counter ();
      encoder_state_change = 0;
    }
  }
  else{
    encoder_state_change = 1;
  }
  // Serial.print("State: ");
  // Serial.println(encoder_state_change);
  // Serial.print("count: ");
  // Serial.println(encoder_count);
  // Serial.print("Read: ");
  // Serial.println(analogRead(analogPin));
  float angle = map((encoder_count%counts_per_rotation), 0, counts_per_rotation-1, 0, 360);
  // Serial.println(angle);
  return angle;
}

void controlActuator(float output) {
  int PID = output;
  int max_pwm = 100;
  // Serial.print("PWM: ");
  // Serial.println(output);
  if (PID >= max_pwm) PID = max_pwm;
  if (PID <= -max_pwm) PID = -max_pwm; 

  if (PID >=0 ) {
    clock_wise=true;
    pwm(Black, PID);
    pinMode(Red, LOW);
    }
  else {
    clock_wise=false;
    pwm(Red, -PID);
    pinMode(Black, LOW);
    } 
}

void setup() {
  // Initialize your hardware, e.g., temperature sensor and actuator
  // Set up serial communication for debugging
  Serial.begin(115200);
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoder_counter, CHANGE);

  pinMode(Red, OUTPUT);
  pinMode(Black, OUTPUT);
  
}



void loop() {
readSensor(); 

// Serial.println(encoder_count);
if (abs(encoder_count)<(40*counts_per_rotation)){
  analogWrite(Red,150);
}
else{
  if(print_one){
    analogWrite(Red,0);
    Serial.println(encoder_count);
    print_one=false;
    }
}

// pwm(Red, 80);
// analogWrite(Red,50);

// if (i == 10000){
//   i = 0;
//   j ++ ;
//   Serial.println(j);
//   if (j > 255){
//     Serial.println("Speed reset!");
//     j = 0;
    
//   }
//   }
// i++;
  /*
  // Read the current temperature from a sensor (replace with your actual sensor code)
  input = readSensor();

  // Calculate the error
  error = setpoint - input;

  // Calculate the proportional term
  float P = Kp * error;

  // Calculate the integral term
  integral += Ki * error;

  // Calculate the derivative term
  float derivative = Kd * (error - last_error);

  // Calculate the control output
  output = P + integral + derivative;
  output = -125;
  // Apply the control output to your actuator (e.g., a heater or cooler)
  controlActuator(output);

  // Store the current error for the next iteration
  last_error = error;

  // Print the values for debugging
  // Serial.print("Input: ");
  // Serial.print(input);
  // Serial.print("  Output: ");
  // Serial.print(output);
  // Serial.print("  Error: ");
  // Serial.println(error);

  // Add a delay to control the loop update rate
  // delay(1000);  // Adjust as needed
  */
}
