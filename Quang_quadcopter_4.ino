#include <Wire.h>
#include <EEPROM.h>

#define gyro_address  0x68
#define MS5611_address 0x77
#define compass_address 0x1E

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cài đặt mức tăng và giới hạn PID
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;
float pid_i_gain_roll = 0.04;
float pid_d_gain_roll = 17.0;
int pid_max_roll = 400;

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;

float pid_p_gain_yaw = 3.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400;

float pid_p_gain_altitude = 1.4;
float pid_i_gain_altitude = 0.2;
float pid_d_gain_altitude = 0.85;
int pid_max_altitude = 400;

int16_t manual_takeoff_throttle = 1500;    // Nhập điểm cất cánh thủ công khi không phát hiện cất cánh tự động (giữa 1400 và 1600).
int16_t motor_idle_speed = 1100;           // Nhập xung ga tối thiếu của động cơ khi chúng không tải (giữa 1000 và 1200).

float declination = 0.0;                   // Đặt độ xích vĩ giữa từ trường và phía bắc địa lý.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Khai báo các biến toàn cục
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;
volatile int pid_roll_setpoint_base, pid_pitch_setpoint_base;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, counter_channel_5, counter_channel_6, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int  start, cal_int;
int temperature, count_var;
int acc_axis[4], gyro_axis[4];
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float roll_level_adjust, pitch_level_adjust;

int32_t acc_x, acc_y, acc_z;
int32_t acc_total_vector, acc_total_vector_at_start;
int16_t acc_x_cal, acc_y_cal;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, current_time;

double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float angle_roll_error, angle_pitch_error;
boolean gyro_angles_set;

uint32_t loop_timer, error_timer, flight_mode_timer;
uint8_t level_calibration_on;
uint8_t check_byte, flip32;
uint8_t error, error_counter, error_led;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint8_t takeoff_detected, manual_altitude_change;
int16_t manual_throttle;
int16_t takeoff_throttle;

int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;
int16_t acc_z_average_short[26], acc_z_average_long[51];
uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;
int32_t acc_alt_integrated;

//Compass variablesvariables
uint8_t compass_calibration_on, heading_lock;
int16_t compass_x, compass_y, compass_z;
int16_t compass_cal_values[6];
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_x, compass_scale_y, compass_scale_z;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;

//Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
unsigned int C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t D1, D2, raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int64_t dT, dT_C5;
//Altitude PID variables
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  delay(3000);

  TWBR = 12;

  DDRD |= B11110000;
  DDRB |= B00000000;
  DDRC |= B00001100;

  green_led(LOW);
  red_led(HIGH);

  sensor_response(); // Kiểm tra xem các cảm biến có phản hồi không.
  
  gyro_setup();
  setup_compass();
  read_compass();
  angle_yaw = actual_compass_heading;

  // Tạo độ trễ 5s trước khi hiệu chỉnh.
  for (count_var = 0; count_var < 1250; count_var++) {
    if (count_var % 125 == 0) {
      digitalWrite(A2, !digitalRead(A2));
    }
    delay(4);
  }
  count_var = 0;

  MS5611_calibration();
  calibrate_gyro();

  PCICR |= (1 << PCIE0); //Set PCIE0 chấp nhận PCMSK0 quét.
  PCMSK0 |= (1 << PCINT0); //Set PCINT0 (digital input 8)
  PCMSK0 |= (1 << PCINT1); //Set PCINT1 (digital input 9)
  PCMSK0 |= (1 << PCINT2); //Set PCINT2 (digital input 10)
  PCMSK0 |= (1 << PCINT3); //Set PCINT3 (digital input 11)
  PCMSK0 |= (1 << PCINT4); //Set PCINT4 (digital input 12)
  PCMSK0 |= (1 << PCINT5); //Set PCINT5 (digital input 13)
  delay(100);

  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    error = 4;
    error_signal();
    delay(3);
    PORTD |= B11110000; //Set chân 4, 5, 6 and 7 high.
    delayMicroseconds(1000); //chờ 1000us.
    PORTD &= B00001111; //Set chân 4, 5, 6 and 7 low.
  }
  error = 0;

  red_led(LOW);

  // Nạp điện áp pin vào biến battery_voltage.
  // 65 là điện áp bù cho diode.
  // 12.6V = ~5V @ Analog 0.
  // 12.6V = 1023 analogRead(0).
  // 1260 / 1023 = 1.2317.
  // Biến battery_voltage là 1050 nếu điện áp pin là 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.2317;
  Serial.print(battery_voltage); Serial.println("\t");

  // Trước khi bắt đầu, giá trị của gia tốc kế trung bình được nạp trươc vào các biến.
  for (start = 0; start <= 24; start++)acc_z_average_short[start] = acc_z;
  for (start = 0; start <= 49; start++)acc_z_average_long[start] = acc_z;
  acc_z_average_short_total = acc_z * 25;
  acc_z_average_long_total = acc_z * 50;
  start = 0;

  if (motor_idle_speed < 1000)motor_idle_speed = 1000;  // Giới hạn tốc độ động cơ không tải tối thiểu ở mức 1000us.
  if (motor_idle_speed > 1200)motor_idle_speed = 1200;  // Giới hạn tốc độ động cơ không tải tối đa ở mức 1200us.

  loop_timer = micros();
}

void loop() {
  if(start == 0){
    // Cần điều khiển tất cả phía trên bên phải: hiệu chỉnh cân bằng
    if(receiver_input_channel_1 > 1900 && receiver_input_channel_2 < 1100 && receiver_input_channel_3 > 1900 && receiver_input_channel_4 > 1900) calibrate_level();
    // Cần điều khiển tất cả phía trên bên trái: hiệu chỉnh la bàn
    if(receiver_input_channel_1 < 1100 && receiver_input_channel_2 < 1100 && receiver_input_channel_3 > 1900 && receiver_input_channel_4 > 1100) calibrate_compass();
  }

  heading_lock = 0;
  if (receiver_input_channel_6 > 1200)heading_lock = 1;

  flight_mode = 1;
  if (receiver_input_channel_5 >= 1200 && receiver_input_channel_5 < 1600)flight_mode = 2;

  flight_mode_signal();
  error_signal();
  gyro_signalen();
  read_barometer();
  read_compass();

  gyro_roll_input = (gyro_roll_input * 0.8) + (((float)gyro_roll / 65.5) * 0.2);
  gyro_pitch_input = (gyro_pitch_input * 0.8) + (((float)gyro_pitch / 65.5) * 0.2);
  gyro_yaw_input = (gyro_yaw_input * 0.8) + (((float)gyro_yaw / 65.5) * 0.2);

  // Serial.print(gyro_roll_input); Serial.print("\t");
  // Serial.print(gyro_pitch_input); Serial.print("\t");
  // Serial.print(gyro_yaw_input); Serial.println("\t");

  angle_pitch += (float)gyro_pitch * 0.0000611;
  angle_roll += (float)gyro_roll * 0.0000611;
  angle_yaw += (float)gyro_yaw * 0.0000611;
  if (angle_yaw < 0) angle_yaw += 360;                           // Nếu angle_yaw < 0, 360 sẽ được thêm vào để giữ nó trong phạm vi từ 0 đến 360.
  else if (angle_yaw >= 360) angle_yaw -= 360;                   // Nếu angle_yaw > 360, 360 sẽ được bớt đi để giữ nó trong phạm vi từ 0 đến 360.

  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);

  // Tính toán góc gia tốc.
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  //57.296 = 1 / (3.142 / 180)
  if (abs(acc_x) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_x/acc_total_vector)* 57.296;
  }
  if (abs(acc_y) < acc_total_vector) { 
    angle_roll_acc = asin((float)acc_y/acc_total_vector)* 57.296;
  }

  // Serial.print((float)acc_x/acc_total_vector); Serial.print("\t");
  // Serial.print((float)acc_y/acc_total_vector); Serial.println("\t");

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  // Serial.print(angle_pitch); Serial.print(",");
  // Serial.print(angle_roll); Serial.print(",");
  // Serial.print(angle_yaw); Serial.println(",");

  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;

  vertical_acceleration_calculations();

  pid_roll_setpoint_base = receiver_input_channel_1;
  pid_pitch_setpoint_base = receiver_input_channel_2;

  if(heading_lock == 1){
    heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
    pid_roll_setpoint_base = 1500 + ((float)(receiver_input_channel_1 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(receiver_input_channel_2 - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
    pid_pitch_setpoint_base = 1500 + ((float)(receiver_input_channel_2 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(receiver_input_channel_1 - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
  }

  if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
  if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
  if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
  if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;

  calculate_pid();

  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * (0.08 * 1.2317);
  if(battery_voltage < 1000 && battery_voltage > 600 && error == 0) error = 1;

  // Serial.print(receiver_input_channel_1); Serial.print("\t");
  // Serial.print(receiver_input_channel_2); Serial.print("\t");
  // Serial.print(receiver_input_channel_3); Serial.print("\t");
  // Serial.print(receiver_input_channel_4); Serial.print("\t");
  // Serial.print(receiver_input_channel_5); Serial.print("\t");
  // Serial.print(receiver_input_channel_6); Serial.println("\t");

  start_stop_takeoff();

   if(takeoff_detected == 1 && start == 2) {
    throttle = receiver_input_channel_3 + takeoff_throttle;
    if (flight_mode >= 2){
      throttle = 1500 + takeoff_throttle + pid_output_altitude + manual_throttle;
    }
  }

  if (start == 2){
    if (throttle > 1800) throttle = 1800;
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

    if (battery_voltage < 1240 && battery_voltage > 800){
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);
    } 

    if (esc_1 < motor_idle_speed) esc_1 = motor_idle_speed;
    if (esc_2 < motor_idle_speed) esc_2 = motor_idle_speed;
    if (esc_3 < motor_idle_speed) esc_3 = motor_idle_speed;
    if (esc_4 < motor_idle_speed) esc_4 = motor_idle_speed;

    if(esc_1 > 2000)esc_1 = 2000;
    if(esc_2 > 2000)esc_2 = 2000;
    if(esc_3 > 2000)esc_3 = 2000;
    if(esc_4 > 2000)esc_4 = 2000;
  }
  else{
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  // Serial.print(esc_1);Serial.print(",");
  // Serial.print(esc_2);Serial.print(",");
  // Serial.print(esc_3);Serial.print(",");
  // Serial.print(esc_4);Serial.println(",");

  while(micros() - loop_timer < 4000);
  loop_timer = micros();

  PORTD |= B11110000;
  timer_channel_1 = esc_1 + loop_timer;
  timer_channel_2 = esc_2 + loop_timer;
  timer_channel_3 = esc_3 + loop_timer;
  timer_channel_4 = esc_4 + loop_timer;

  while(PORTD >= 16){
    esc_loop_timer = micros();
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;
  }
}

ISR(PCINT0_vect){ //Khai báo ngắt RX
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){ // Nếu chân 8 HIGH
    if(last_channel_1 == 0){ // Thay đổi chân 8 từ 0 thành 1
      last_channel_1 = 1; // Ghi nhớ trạng thái đầu vào hiện tại
      timer_1 = current_time; // Đặt timer_1 thành current_time
    }
  }
  else if(last_channel_1 == 1){ // Chân 8 không HIGH thì thay đổi từ 1 về 0
    last_channel_1 = 0; // Ghi nhớ trạng thái đầu vào hiện tại
    receiver_input_channel_1 = current_time - timer_1; 
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){ // Nếu chân 9 HIGH
    if(last_channel_2 == 0){ // Thay đổi chân 11 từ 0 thành 1
      last_channel_2 = 1; // Ghi nhớ trạng thái đầu vào hiện tại
      timer_2 = current_time; // Đặt timer_2 thành current_time
    }
  }
  else if(last_channel_2 == 1){ // Chân 9 không HIGH thì thay đổi từ 1 về 0
    last_channel_2 = 0; // Ghi nhớ trạng thái đầu vào hiện tại
    receiver_input_channel_2 = current_time - timer_2; 
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){ // Nếu chân 10 HIGH
    if(last_channel_3 == 0){ // Thay đổi chân 10 từ 0 thành 1
      last_channel_3 = 1; // Ghi nhớ trạng thái đầu vào hiện tại
      timer_3 = current_time; // Đặt timer_3 thành current_time
    }
  }
  else if(last_channel_3 == 1){ // Chân 10 không HIGH thì thay đổi từ 1 về 0
    last_channel_3 = 0; // Ghi nhớ trạng thái đầu vào hiện tại
    receiver_input_channel_3 = current_time - timer_3;
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){ // Nếu chân 11 HIGH
    if(last_channel_4 == 0){ // Thay đổi chân 11 từ 0 thành 1
      last_channel_4 = 1; // Ghi nhớ trạng thái đầu vào hiện tại
      timer_4 = current_time; // Đặt timer_4 thành current_time
    }
  }
  else if(last_channel_4 == 1){ // Chân 11 không HIGH thì thay đổi từ 1 về 0
    last_channel_4 = 0; // Ghi nhớ trạng thái đầu vào hiện tại
    receiver_input_channel_4 = current_time - timer_4; 
  }
  //Channel 5=========================================
  if(PINB & B00010000 ){ // Nếu chân 12 HIGH
    if(last_channel_5 == 0){ // Thay đổi chân 12 từ 0 thành 1
      last_channel_5 = 1; // Ghi nhớ trạng thái đầu vào hiện tại
      timer_5 = current_time; // Đặt timer_5 thành current_time
    }
  }
  else if(last_channel_5 == 1){ // Chân 12 không HIGH thì thay đổi từ 1 về 0
    last_channel_5 = 0; // Ghi nhớ trạng thái đầu vào hiện tại
    receiver_input_channel_5 = current_time - timer_5; 
  }
  //Channel 6=========================================
  if(PINB & B00100000 ){ // Nếu chân 13 HIGH
    if(last_channel_6 == 0){ // Thay đổi chân 13 từ 0 thành 1
      last_channel_6 = 1; // Ghi nhớ trạng thái đầu vào hiện tại
      timer_6 = current_time; // Đặt timer_3 thành current_time
    }
  }
  else if(last_channel_6 == 1){ // Chân 13 không HIGH thì thay đổi từ 1 về 0
    last_channel_6 = 0; // Ghi nhớ trạng thái đầu vào hiện tại
    receiver_input_channel_6 = current_time - timer_6; 
  }
}

void gyro_signalen(){
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address,14);

  while(Wire.available() < 14);
  acc_x = Wire.read()<<8|Wire.read();
  acc_y = Wire.read()<<8|Wire.read();
  acc_z = Wire.read()<<8|Wire.read();
  temperature = Wire.read()<<8|Wire.read();
  gyro_roll = Wire.read()<<8|Wire.read();
  gyro_pitch = Wire.read()<<8|Wire.read();
  gyro_yaw = Wire.read()<<8|Wire.read();
  gyro_pitch *= -1;
  gyro_yaw *= -1;

  if(cal_int >= 2000){
    gyro_roll -= gyro_roll_cal;
    gyro_pitch -= gyro_pitch_cal;
    gyro_yaw -= gyro_yaw_cal;
  }

  if (level_calibration_on == 0) {
    acc_x -= acc_x_cal;
    acc_y -= acc_y_cal;
  }

  // Serial.print(acc_x); Serial.print("\t");
  // Serial.print(acc_y); Serial.print("\t");
  // Serial.print(acc_z); Serial.println("\t");
}

void calibrate_level(void) {
  level_calibration_on = 1;
  red_led(HIGH);
  green_led(LOW);

  acc_x_cal = 0;
  acc_y_cal = 0;

  for (error = 0; error < 64; error ++) {
    gyro_signalen();
    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    if (acc_y > 500 || acc_y < -500)error = 80;
    if (acc_x > 500 || acc_x < -500)error = 80;
    delayMicroseconds(3700);
  }

  acc_x_cal /= 64;
  acc_y_cal /= 64;

  red_led(LOW);
  if (error < 80) {
    EEPROM.write(12, acc_x_cal & 0b11111111); EEPROM.write(13, acc_x_cal >> 8);
    EEPROM.write(14, acc_y_cal & 0b11111111); EEPROM.write(15, acc_y_cal >> 8);
    Serial.print(acc_x_cal); Serial.print("\t");
    Serial.print(acc_y_cal); Serial.println("\t");
    for (error = 0; error < 15; error ++) {
      green_led(HIGH);
      delay(50);
      green_led(LOW);
      delay(50);
    }
    error = 0;
  }
  else error = 3;
  level_calibration_on = 0;
  gyro_signalen();
  
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  if (abs(acc_x) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }
  if (abs(acc_y) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }
  angle_pitch = angle_pitch_acc;
  angle_roll = angle_roll_acc;
  loop_timer = micros();
}

void calibrate_gyro(void){
  Serial.print("Calibrating gyro");
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){
    if(cal_int % 15 == 0)digitalWrite(A2, !digitalRead(A2));
    if(cal_int % 125 == 0)Serial.print(".");
    gyro_signalen();
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;

    PORTD |= B11110000;
    delayMicroseconds(1000);
    PORTD &= B00001111;
    delay(3);
  }
  Serial.println("\t");
  red_led(HIGH);
  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
  gyro_yaw_cal /= 2000;
}

void gyro_setup(){
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B); // Thanh ghi PWR_MGMT_1
  Wire.write(0x00); // Đặt các bit Thanh ghi là 00000000 để kích hoạt con quay hồi chuyển.
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B); // Thanh ghi GYRO_CONFIG
  Wire.write(0x08); // Đặt các bit Thanh ghi là 00001000 (500dps full scale)
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C); // Thanh ghi ACCEL_CONFIG
  Wire.write(0x10); // Đặt các bit Thanh ghi là 00010000 (+/- 8g full scale range)
  Wire.endTransmission();

  // Thực hiện kiểm tra Thanh ghi ngẫu nhiên để xem các giá trị được viết đúng
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  while(Wire.available() < 1);
  if(Wire.read() != 0x08){
    digitalWrite(A2,HIGH);
    while(1)delay(10);
  }

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A); // Thanh ghi CONFIG
  Wire.write(0x03); // Đặt các bit Thanh ghi là 00000011 (Đặt Bộ lọc thông thấp kỹ thuật số thành ~43Hz)
  Wire.endTransmission();

  acc_x_cal  = EEPROM.read(13) << 8 | EEPROM.read(12);
  acc_y_cal  = EEPROM.read(15) << 8 | EEPROM.read(14);
  Serial.print(acc_x_cal); Serial.print("\t");
  Serial.print(acc_y_cal); Serial.println("\t");
}

void calculate_pid(){

  pid_roll_setpoint = 0;
  
  if(pid_roll_setpoint_base > 1512)pid_roll_setpoint = pid_roll_setpoint_base - 1512;
  else if(pid_roll_setpoint_base < 1492)pid_roll_setpoint = pid_roll_setpoint_base - 1492;

  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 4.0;

  pid_pitch_setpoint = 0;

  if(pid_pitch_setpoint_base > 1512)pid_pitch_setpoint = pid_pitch_setpoint_base - 1512;
  else if(pid_pitch_setpoint_base < 1492)pid_pitch_setpoint = pid_pitch_setpoint_base - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 4.0;

  pid_yaw_setpoint = 0;

  if(receiver_input_channel_3 > 1050){
    if(receiver_input_channel_4 > 1512)pid_yaw_setpoint = (receiver_input_channel_4 - 1512)/4.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/4.0;
  }

  // Serial.print(pid_roll_setpoint); Serial.print("\t");
  // Serial.print(pid_pitch_setpoint); Serial.println("\t");

  // Tính toán Roll
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  // Tính toán Pitch
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  // Tính toán Yaw
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void MS5611_calibration(void){
  Wire.beginTransmission(MS5611_address);
  Wire.write(0x1E);
  Wire.endTransmission();
  delay(3);

  for (start = 1; start <= 6; start++) {
    Wire.beginTransmission(MS5611_address);
    Wire.write(0xA0 + start * 2);
    Wire.endTransmission();

    Wire.requestFrom(MS5611_address, 2);
    C[start] = Wire.read();
    C[start] = (C[start] * (uint32_t)256) + Wire.read();
    // Serial.println(C[start]);
  }

  OFF_C2 = C[2] * pow(2, 16);
  SENS_C1 = C[1] * pow(2, 15);

  for (start = 0; start < 100; start++) {
    read_barometer();
    delay(4);
  }
  actual_pressure = 0;
}

void read_barometer(void) {
  // Serial.print(raw_pressure); Serial.print("\t");
  // Serial.print(raw_temperature); Serial.println("\t");
  // Serial.print(100810); Serial.print("\t");
  // Serial.print(actual_pressure); Serial.println("\t");
  barometer_counter ++;

  // Mỗi lần hàm này được gọi, biến barometer_counter sẽ tăng lên. Bằng cách này, một hành động cụ thể
  //được thực hiện vào đúng thời điểm. Điều này là cần thiết vì việc yêu cầu dữ liệu từ MS5611 mất khoảng 9 mili giây để hoàn thành.

  if (barometer_counter == 1) {
    if (temperature_counter == 0) {
      // Lấy dữ liệu nhiệt độ từ MS-5611
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(MS5611_address, 3);
      D2 = Wire.read();
      D2 = (D2 * (uint32_t)256) + Wire.read();
      D2 = (D2 * (uint32_t)256) + Wire.read();
      // raw_temperature = D2;
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = D2;
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;
    }
    else {
      // Lấy dữ liệu áp suất từ MS-5611
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(MS5611_address, 3);
      D1 = Wire.read();
      D1 = (D1 * (uint32_t)256) + Wire.read();
      D1 = (D1 * (uint32_t)256) + Wire.read();
      raw_pressure = D1;
    }

    temperature_counter ++;
    if (temperature_counter == 20) {
      temperature_counter = 0;
      // Yêu cầu dữ liệu nhiệt độ
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x58);
      Wire.endTransmission();
    }
    else {
      // Yêu cầu dữ liệu áp suất
      Wire.beginTransmission(MS5611_address);
      Wire.write(0x48);
      Wire.endTransmission();
    }
  }
  if (barometer_counter == 2) {
    // Tính toán áp suất như được giải thích trong datasheet của MS-5611.
    dT = raw_temperature - C[5] * pow(2, 8);
    OFF = OFF_C2 + (dT * C[4]) / pow(2, 7);
    SENS = SENS_C1 + (dT * C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];
    pressure_rotating_mem[pressure_rotating_mem_location] = P;
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];
    pressure_rotating_mem_location++;
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;

    
    actual_pressure_slow = actual_pressure_slow * (float)0.9 + actual_pressure_fast * (float)0.1;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;
    // Nếu chênh lệch lớn hơn 1 hoặc nhỏ hơn thì -1 thì mức trung bình chậm được điều chỉnh dựa trên sai số giữa mức trung bình nhanh và chậm.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;
  }

  if (barometer_counter == 3) {

    barometer_counter = 0;
    // Trong phần sau, bộ đệm quay được sử dụng để tính toán sự thay đổi dài hạn giữa các phép đo áp suất khác nhau..
    // Tổng giá trị này có thể được sử dụng để phát hiện hướng (lên/xuống) và tốc độ của quadcopter và chức năng như bộ điều khiển D của bộ điều khiển PID tổng.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10;
    parachute_rotating_mem_location++;
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;

    if (flight_mode >= 2 && takeoff_detected == 1) {
      if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;
      // Khi vị trí cần ga tăng hoặc giảm, chức năng giữ độ cao sẽ bị tắt một phần.
      // Biến manual_altitude_change sẽ cho biết liệu độ cao của quadcopter có bị thay đổi bởi người điều khiển hay không.
      manual_altitude_change = 0;
      manual_throttle = 0;
      if (receiver_input_channel_3 > 1600) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (receiver_input_channel_3 - 1600) / 3;
      }
      if (receiver_input_channel_3 < 1400) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (receiver_input_channel_3 - 1400) / 5;
      }

      // Tính toán đầu ra PID của giữ độ cao.
      pid_altitude_input = actual_pressure;
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;

      // Để kết quả tốt hơn thì P-gain sẽ được tăng khi sai số giữa setpoint và giá trị áp suất thực tế tăng.
      // Biến pid_error_gain_altitude sẽ được sử dụng để điều chỉnh P-gain của bộ điều khiển PID.
      pid_error_gain_altitude = 0;
      if (pid_error_temp > 10 || pid_error_temp < -10) {
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;
      }

      // Trong phần sau, đầu ra I được tính toán. Đó là sự tích lũy sai sót theo thời gian.
      // Yếu tố thời gian bị loại bỏ khi vòng lặp chương trình chạy ở tần số 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      // Trong dòng sau, đầu ra PID được tính toán.
      //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
      //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
      //D = pid_d_gain_altitude * parachute_throttle.
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
    }

    // Nếu chức năng giữ độ cao bị vô hiệu hóa, một số biến cần phải được đặt lại để đảm bảo khởi động không va chạm khi chức năng giữ độ cao được kích hoạt lại.
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {
      pid_altitude_setpoint = 0;
      pid_output_altitude = 0;
      pid_i_mem_altitude = 0;
      manual_throttle = 0;
      manual_altitude_change = 1;
    }
  }
}

float course_deviation(float course_b, float course_c) {
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}

void setup_compass(void){
  Wire.beginTransmission(compass_address);
  Wire.write(0x00); // Thanh ghi CONFIG
  Wire.write(0x78); // Đặt các bit thanh ghi là 01111000 để đặt tỷ lệ mẫu (trung bình là 8 ở 75Hz)
  Wire.write(0x20); // Đặt các bit thanh ghi là 00100000 để đặt mức tăng ở mức +/- 1,3Ga
  Wire.write(0x00);
  Wire.endTransmission();

  uint8_t addrEEPROM = 0;
  for (error = 0; error < 6; error ++){
    compass_cal_values[error] = EEPROM.read(addrEEPROM + 1) << 8 | EEPROM.read(addrEEPROM);
    addrEEPROM = addrEEPROM + 2;
    Serial.println(compass_cal_values[error]);
  }
  addrEEPROM = 0;
  error = 0;

  // Tính toán giá trị hiệu chỉnh và giá trị tỷ lệ.
  compass_scale_x = ((float)compass_cal_values[3] - compass_cal_values[2]) / (compass_cal_values[1] - compass_cal_values[0]);
  compass_scale_z = ((float)compass_cal_values[3] - compass_cal_values[2]) / (compass_cal_values[5] - compass_cal_values[4]);

  compass_offset_y = (compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3];
  compass_offset_x = (((float)compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1]) * compass_scale_x;
  compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}

void read_compass(void) {
  // Serial.print(compass_x); Serial.print("\t");
  // Serial.print(compass_y); Serial.print("\t");
  // Serial.print(compass_z); Serial.println("\t");
  Wire.beginTransmission(compass_address);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.requestFrom(compass_address, 6);
  compass_x = Wire.read() << 8 | Wire.read();
  compass_z = Wire.read() << 8 | Wire.read();
  compass_y = Wire.read() << 8 | Wire.read();
  compass_y *= -1;

  if (compass_calibration_on == 0) {
  compass_x += compass_offset_x;
  compass_x *= compass_scale_x;
  compass_z += compass_offset_z;
  compass_z *= compass_scale_z;
  compass_y += compass_offset_y;
  }

  // Các giá trị la bàn thay đổi khi góc cuộn và góc nghiêng của quadcopter thay đổi. Đó là lý do mà giá trị x, y cần tính toán cho vị trí nằm ngang ảo
  // 0.0174533 = pi/180 vì các hàm được tính bằng radian thay vì độ.
  compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
  compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

  // Bây giờ các giá trị theo chiều ngang đã được biết, hướng đầu có thể được tính toán. Với những dòng mã sau đây, hướng đầu được tính theo độ.
  // Hãy lưu ý rằng atan2 sử dụng radian thay vì độ đó là lý do tại sao 180/3.14.
  if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
  else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

  actual_compass_heading += declination;
  if (actual_compass_heading < 0) actual_compass_heading += 360;         // Nếu hướng đầu của la bàn trở về nhỏ hơn 0, 360 sẽ được thêm vào để giữ nó trong phạm vi từ 0 đến 360.
  else if (actual_compass_heading >= 360) actual_compass_heading -= 360; // Nếu hướng đầu của la bàn trở về lớn hơn 360, 360 sẽ được bớt đi để giữ nó trong phạm vi từ 0 đến 360.
}

void calibrate_compass(void){
  compass_calibration_on = 1;
  red_led(HIGH);
  green_led(LOW);
  while(receiver_input_channel_2 < 1900){
    delayMicroseconds(3700);
    read_compass();
    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
  }
  compass_calibration_on = 0;

  uint8_t addrEEPROM = 0;
  for (error = 0; error < 6; error ++){
    Serial.println(compass_cal_values[error]);
    EEPROM.write(addrEEPROM, compass_cal_values[error] & 0b11111111); EEPROM.write(addrEEPROM + 1, compass_cal_values[error] >> 8);
    addrEEPROM = addrEEPROM + 2;
  }
  addrEEPROM = 0;
  setup_compass();
  read_compass();
  angle_yaw = actual_compass_heading;

  red_led(LOW);
  for (error = 0; error < 15; error ++) {
    green_led(HIGH);
    delay(50);
    green_led(LOW);
    delay(50);
  }
  error = 0;

  loop_timer = micros();
}

void vertical_acceleration_calculations(void) {
  // Serial.print(acc_total_vector); Serial.print("\t");
  // Serial.print(acc_z_average_short_total/25); Serial.println("\t");
  acc_z_average_short_rotating_mem_location++;
  if (acc_z_average_short_rotating_mem_location == 25)acc_z_average_short_rotating_mem_location = 0;

  acc_z_average_short_total -= acc_z_average_short[acc_z_average_short_rotating_mem_location];
  acc_z_average_short[acc_z_average_short_rotating_mem_location] = acc_total_vector;
  acc_z_average_short_total += acc_z_average_short[acc_z_average_short_rotating_mem_location];

  if (acc_z_average_short_rotating_mem_location == 0) {
    acc_z_average_long_rotating_mem_location++;

    if (acc_z_average_long_rotating_mem_location == 50)acc_z_average_long_rotating_mem_location = 0;

    acc_z_average_long_total -= acc_z_average_long[acc_z_average_long_rotating_mem_location];
    acc_z_average_long[acc_z_average_long_rotating_mem_location] = acc_z_average_short_total / 25;
    acc_z_average_long_total += acc_z_average_long[acc_z_average_long_rotating_mem_location];
  }
  acc_z_average_total = acc_z_average_long_total / 50;


  acc_alt_integrated += acc_total_vector - acc_z_average_total;
  if (acc_total_vector - acc_z_average_total < 400 || acc_total_vector - acc_z_average_total > 400) {
    if (acc_z_average_short_total / 25 - acc_z_average_total < 500 && acc_z_average_short_total / 25 - acc_z_average_total > -500)
      if (acc_alt_integrated > 200)acc_alt_integrated -= 200;
      else if (acc_alt_integrated < -200)acc_alt_integrated += 200;
  }
}

void start_stop_takeoff(void) {
  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  // Khởi động động cơ: cần ga ở vị trí thấp và bên trái.
  if (start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450) {
    throttle = motor_idle_speed;
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    ground_pressure = actual_pressure;
    course_lock_heading = angle_yaw;
    acc_total_vector_at_start = acc_total_vector;
    start = 2;
    acc_alt_integrated = 0;
    if (manual_takeoff_throttle > 1400 && manual_takeoff_throttle < 1600) {
      takeoff_throttle = manual_takeoff_throttle - 1500;
      takeoff_detected = 1;
      // Đặt lại điều khiển PID để cất cánh suôn sẻ.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_output_roll = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_output_pitch = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
      pid_output_yaw = 0;
    }
    else if (manual_takeoff_throttle) {
      error = 5;
      takeoff_throttle = 0;
      start = 0;
    }
  }
  // Dừng động cơ: cần ga ở vị trí thấp và bên phải.
  if (start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950) {
    start = 0;
    takeoff_detected = 0;
  }

  if (takeoff_detected == 0 && start == 2) {
    if (receiver_input_channel_3 > 1480 && throttle < 1750) throttle++;
    if (throttle == 1750)error = 6;
    if (receiver_input_channel_3 <= 1480) {
      if (throttle > motor_idle_speed)throttle--;
      // Đặt lại điều khiển PID để cất cánh suôn sẻ.
      else {
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_output_roll = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_output_pitch = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        pid_output_yaw = 0;
      }
    }
    if (acc_z_average_short_total / 25 - acc_total_vector_at_start > 800) {
      takeoff_detected = 1;
      pid_altitude_setpoint = ground_pressure - 22;
      if (throttle > 1400 && throttle < 1700) takeoff_throttle = throttle - 1530;
      else {
        error = 7;
        takeoff_throttle = 0;
      }
    }
  }
}

void sensor_response(void){

  Wire.beginTransmission(gyro_address);
  error = Wire.endTransmission();
  while (error != 0) {
    Serial.println("The MPU-6050 isn't responding.");
    error = 1;
    error_signal();
    delay(4);
  }

  Wire.beginTransmission(compass_address);
  error = Wire.endTransmission();
  while (error != 0) {
    Serial.println("The compass isn't responding.");
    error = 2;
    error_signal();
    delay(4);
  }

  Wire.beginTransmission(MS5611_address);
  error = Wire.endTransmission();
  while (error != 0) {
    Serial.println("The MS5611 barometer isn't responding.");
    error = 3;
    error_signal();
    delay(4);
  }
}

void red_led(int8_t level) {
  if(flip32)digitalWrite(A2, !level);
  else digitalWrite(A2, level);
}
void green_led(int8_t level) {
  if(flip32)digitalWrite(A3, !level);
  else digitalWrite(A3, level);
}

void error_signal(void) {
  if (error >= 100) red_led(HIGH);
  else if (error_timer < millis()) {                                                       //Nếu giá trị error_timer nhỏ hơn hàm millis().
    error_timer = millis() + 250;                                                          //Đặt error_timer tiếp theo ở khoảng 250ms.
    if (error > 0 && error_counter > error + 3) error_counter = 0;                         //Nếu có lỗi để báo hiệu và error_counter > error +3 thì đặt lại error.
    if (error_counter < error && error_led == 0 && error > 0) {                            //Nếu chuỗi LED báo lỗi chưa được hoàn thành (error_counter < error) và đèn LED tắt.
      red_led(HIGH);                                                                       //Bật LED.
      error_led = 1;                                                                       //Đặt cờ LED để biết LED đang bật.
    }
    else {                                                                                 //Nếu chuỗi LED báo lỗi chưa được hoàn thành (error_counter < error) và đèn LED bật.
      red_led(LOW);                                                                        //Tắt LED.
      error_counter++;                                                                     //Tăng biến error_counter lên 1 để theo dõi các lần nhấp nháy.
      error_led = 0;                                                                       //Đặt cờ LED để biết LED đang tăt.
    }
  }
}

void flight_mode_signal(void) {
  if (flight_mode_timer < millis()) {                                                      //Nếu giá trị flight_mode_timer nhỏ hơn hàm millis().
    flight_mode_timer = millis() + 250;                                                    //Đặt Đặt error_timer tiếp theo ở khoảng 250ms. tiếp theo ở khoảng 250ms.
    if (flight_mode > 0 && flight_mode_counter > flight_mode + 3) flight_mode_counter = 0; //Nếu có lỗi để báo hiệu và flight_mode_counter > flight_mode +3 thì đặt lại flight_mode_counter
    if (flight_mode_counter < flight_mode && flight_mode_led == 0 && flight_mode > 0) {    //Nếu chuỗi LED báo lỗi chưa được hoàn thành (flight_mode_counter < flight_mode) và đèn LED tắt.
      green_led(HIGH);                                                                     //Bật LED.
      flight_mode_led = 1;                                                                 //Đặt cờ LED để biết LED đang bật.
    }
    else {                                                                                 //Nếu chuỗi LED báo lỗi chưa được hoàn thành (flight_mode_counter < flight_mode) và đèn LED bật.
      green_led(LOW);                                                                      //Tắt LED.
      flight_mode_counter++;                                                               //Tăng biến flight_mode_counter lên 1 để theo dõi các lần nhấp nháy.
      flight_mode_led = 0;                                                                 //Đặt cờ LED để biết LED đang tăt.
    }
  }
}