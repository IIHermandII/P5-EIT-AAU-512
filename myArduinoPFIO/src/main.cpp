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
int counts_per_rotation = 100;
float position=0;


// Define pins
const int encoderPin = A3;
const int Black = 11;
const int Red = 9;

// pwm global variables
const int pwm_period = 50; // 20 kHz PWM period in microseconds
unsigned long pre_micros = 0;
int pwmValue = 0;
int i, j = 0;


// Interupt function
void encoder_counter (){
  encoder_count ++;
  // Serial.print("count: ");
  // Serial.println(encoder_count);
  // Serial.print("Read: ");
  // Serial.println(analogRead(analogPin));
}



void pwm(int pin, int duty){
  unsigned long current_micros = micros();
  
  if (current_micros - pre_micros >= duty){
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
}

float readSensor() {
  float input;
  input = analogRead(encoderPin); //
  // Serial.print("Encoder sensor input value ----> ");
  // Serial.println(input);
  if (input<encoder_threshold){
    if (encoder_state_change){
      encoder_counter ();
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
  
  return position;
}

void controlActuator(float output) {
  int PID = output;
  int max_pwm = 50;
  // Serial.print("PWM: ");
  // Serial.println(output);
  if (PID >= max_pwm) PID = max_pwm;
  if (PID <= -max_pwm) PID = -max_pwm; 

  if (PID >=0 ) {
    pwm(Black, PID);
    pinMode(Red, LOW);
    }
  else {
    pwm(Red, -PID);
    pinMode(Black, LOW);
    } 
}

void setup() {
  // Initialize your hardware, e.g., temperature sensor and actuator
  // Set up serial communication for debugging
  Serial.begin(9600);
  pinMode(encoderPin, INPUT);

  pinMode(Red, OUTPUT);
  pinMode(Black, OUTPUT);
  
}



void loop() {
readSensor(); 
if (encoder_count<1000){
  pwm(Red, 10);
}
else{
  pwm(Red, 0);
  Serial.println(encoder_count);
}

// pwm(Red, 0);
// if (i == 30000){
//   i = 0;
//   j ++ ;
//   Serial.println(j);
//   if (j == 50){
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
