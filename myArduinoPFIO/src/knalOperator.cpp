#include <Arduino.h>

unsigned long global_time = 0;
int encode_array_possion = 0;
bool encoder_array_possion_direction_count = true; // true = med ur,   false = mod ur 
const int posision_arr[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 
21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 
41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 
61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 
81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 
101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 
160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177};

unsigned long encoder_counter_time_arr[100]={};

// Interupt function

struct possin_data {
  int curent_possion = 0;   // stores the curent possion of wher the motor are
  int desired_pission = 0;  // stores the desired possion of the motor 
  int steps = 0;            // stores the number of steps requred to go from curent to desired possion
  bool direction = true;    // stores the direction requred to go from curent to desired possion
  int time_messure = 0;
};

void encoder_counter (){
  static int encoder_possion_ticks = 0;
  if (encode_array_possion > 99){
    encode_array_possion = 0;
  }
  if (encoder_possion_ticks > 177){
    encoder_possion_ticks = 0;
  }
  if (encoder_possion_ticks < 0){
    encoder_possion_ticks = 177;
  }
  encoder_counter_time_arr[encode_array_possion]=encoder_possion_ticks;
  encode_array_possion++;
  if (encoder_array_possion_direction_count){
    encoder_possion_ticks++;
  }else{
    encoder_possion_ticks--;
  }
}

void get_direction (struct possin_data* motor_data){
  int positive_step = 0, nigative_step = 0;
  int posision_count = motor_data->curent_possion;
  while(1){
    delay(1);
      if (posision_arr[posision_count] == motor_data->desired_pission){
      break;
    }else if (posision_arr[posision_count] == 177){ // the opration acts on posision_count like a step
      positive_step++;                            // therfore we do the same for nigative_step
      posision_count = 0;
    }
    if (posision_arr[posision_count] == motor_data->desired_pission){
      break;
    }
    positive_step++;
    posision_count++;
  }
  posision_count = motor_data->curent_possion;
  while(1){
    delay(1);
     if (posision_arr[posision_count] == motor_data->desired_pission){
      break;
    }else if (posision_arr[posision_count] == 0){ // the opration acts on posision_count like a step
      nigative_step++;                            // therfore we do the same for nigative_step
      posision_count = 177;
    }
    if (posision_arr[posision_count] == motor_data->desired_pission){
      break;
    }
    nigative_step++;
    posision_count--;
  }

  if (positive_step <= nigative_step) {
    motor_data->steps = positive_step;
    motor_data->direction = true;
  } else{
    motor_data->steps = nigative_step;
    motor_data->direction = false;
  }
}

void serial_read_desired_pission(struct possin_data* motor_data){
  if (Serial.available() > 0) {
    String word = Serial.readStringUntil('\n');
    word.trim();
    Serial.print("ANGLE REQUEST---> ");
    Serial.print(word);
    Serial.println(" deg ");
    motor_data->desired_pission = word.toInt();
  }
  if (motor_data->desired_pission > 360){
    motor_data->desired_pission = 360;
  }
  if (motor_data->desired_pission < 0){
    motor_data->desired_pission = 0;
  }
  motor_data->desired_pission = map(motor_data->desired_pission, 0, 360, 0, 177);
}

void interrupt_processing_curent_possion(struct possin_data* motor_data){
  motor_data->time_messure = millis();
  static unsigned long last_time_mesure = 0;
  if (motor_data->time_messure - last_time_mesure > 30){ // only inters after 30 ms this will // make sure we only acces data at given time intervals 
  last_time_mesure = millis();
  // Serial.print("encoder_counter_time_arr[");
  // Serial.print(encode_array_possion-1);
  // Serial.print("]=");
  // Serial.println(encoder_counter_time_arr[encode_array_possion-1]);
  motor_data->curent_possion = encoder_counter_time_arr[encode_array_possion-1];
  }
}

void setup() {
  Serial.begin(115200);
  //attachInterrupt(digitalPinToInterrupt(encoderPin), encoder_counter, CHANGE);

  for (int i=0; i<100; i++){
    encoder_counter_time_arr[i]=0;
  }
}

void loop() {
  global_time = millis(); // overflowes after 49.7 dayes
  possin_data motor_data;

  motor_data.curent_possion = 177;
  encoder_counter();
  interrupt_processing_curent_possion(&motor_data);
  serial_read_desired_pission(&motor_data); // find out where SDR want you to go OBS NO MINIMAL SAFTY
  get_direction(&motor_data); // get_direction takes motor_data with all it contains

  Serial.print(" global time : ");
  Serial.println(global_time); // debucking

  Serial.print(" curent possion : ");
  Serial.println(motor_data.curent_possion);
  Serial.print(" desired possion : ");
  Serial.println(motor_data.desired_pission);
  Serial.print(" steps : ");
  Serial.println(motor_data.steps);
  Serial.print(" dir : ");
  Serial.println(motor_data.direction);
  delay(100);
}
