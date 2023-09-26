#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

//function declaration
void IMU_init();
void IMU_error();
void IMU_read();
void Madgwick();
void angleControl();
void outputMix();
bool checkPWM();
void looprate(int);
float invSqrt(float);
void print_pid();
void print_input();
void print_desired();

//time vars
unsigned long time_current, time_prev;
float dt;

//PWM Read Variables
unsigned long counter_1, counter_2, counter_3, counter_4, counter_5, counter_6, current_count;
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state, last_CH5_state, last_CH6_state;
int input_throttle, input_roll, input_pitch, input_yaw, input_aux1, input_aux2;

//IMU Vars
int MPU6050 = 0x68;
float Ax, Ay, Az, Gx, Gy, Gz;
float roll_measured, pitch_measured, yaw_measured;
float Ax_error, Ay_error, Az_error, Gx_error, Gy_error, Gz_error;
float Ax_prev, Ay_prev, Az_prev, Gx_prev, Gy_prev, Gz_prev; 

//Madgwick vars
const float rad2deg = 57.2958;
float beta = 0.04;
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Control vars
float roll_desired, pitch_desired, yaw_desired, roll_error, pitch_error, yaw_error, roll_pid, pitch_pid, yaw_pid;
float p_roll, i_roll, iPrev_roll, d_roll, p_pitch, i_pitch, iPrev_pitch, d_pitch, p_yaw, i_yaw, iPrev_yaw, d_yaw;
float kp_roll = 1.3, ki_roll = 0.04, kd_roll = 18.0;
float kp_pitch = 1.3, ki_pitch = 0.04, kd_pitch = 18.0;
float kp_yaw = 4.0, ki_yaw = 0.02, kd_yaw = 0.0;


float output_Throt, output_portAil, output_stbdAil, output_Elev;
float output_M1, output_M2, output_M3, output_M4;


bool failsafe;
bool connectstate = 1;
bool prevfailsafe = 1;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


void setup() {
  PCICR  |= B00000101;  //Enable PCMSK0 and PCMSK2 Scan
  PCMSK0 |= B00000001;
  PCMSK2 |= B11110100;

  

  Serial.begin(9600);

  pinMode(12, OUTPUT); //buzzer
  pinMode(13, OUTPUT); //light


  digitalWrite(13, HIGH);
  IMU_init();
  IMU_error();
  digitalWrite(13, LOW);

  servo1.attach(3, 900, 2100);
  servo1.writeMicroseconds(1000);
  servo2.attach(9, 900, 2100);
  servo2.writeMicroseconds(1000);
  servo3.attach(10, 900, 2100);
  servo3.writeMicroseconds(1000);
  servo4.attach(11, 900, 2100);
  servo4.writeMicroseconds(1000);

}

void loop() {
  failsafe = checkPWM();

  if((failsafe - prevfailsafe) < 0){
    connectstate = 1;
  }
  else if((failsafe - prevfailsafe) > 0){
    connectstate = 0;
  }
  prevfailsafe = failsafe;

  if (connectstate == 0){
    digitalWrite(12,HIGH);
  }
  else if(connectstate == 1){
    digitalWrite(12, LOW);
  }


  IMU_read();
  time_prev = time_current;
  time_current = micros();
  dt = (time_current - time_prev)/1000000.0;
  Madgwick();

  //deadband
  if((input_roll < 1516) & (input_roll > 1484)){
    input_roll = 1500;
  }
  if((input_pitch < 1516) & (input_pitch > 1484)){
    input_pitch = 1500;
  }
  if((input_yaw < 1520) & (input_yaw > 1480)){
    input_yaw = 1500;
  }

  roll_desired = map(input_roll, 1000, 2000, -10, 10);
  pitch_desired = map(input_pitch, 1000, 2000, -10, 10);
  yaw_desired = -1*map(input_yaw, 1000, 2000, -30, 30);



  angleControl();
  outputMix();
  if(input_throttle < 1080){
    output_M1 = 0;
    output_M2 = 0;
    output_M3 = 0;
    output_M4 = 0;

  }
  
  servo1.writeMicroseconds(output_M1);
  servo2.writeMicroseconds(output_M2);
  servo3.writeMicroseconds(output_M3);
  servo4.writeMicroseconds(output_M4);

  //print functions
  // print_pid();
  // print_input();
  // print_desired();




  looprate(250);
  //print_looptime();
}



void IMU_init(){
  // power mpu6050
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
  // acc config
  Wire.beginTransmission(MPU6050);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
  // gyro config
  Wire.beginTransmission(MPU6050);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
}

void IMU_error(){
  int iterations = 1000;
  //Acc error
  int i = 0;
  while(i < iterations){
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6, true);
    Ax = ( Wire.read() << 8| Wire.read());
    Ax = Ax/16384.0;
    Ay = ( Wire.read() << 8| Wire.read());
    Ay = Ay/16384.0;
    Az = ( Wire.read() << 8| Wire.read());
    Az = Az/16384.0; 
    Ax_error = Ax_error + Ax;
    Ay_error = Ay_error + Ay;
    Az_error = Az_error + Az;
    i++;
  }
  //Mean error
  Ax_error = Ax_error / iterations;
  Ay_error = Ay_error / iterations;
  Az_error = (Az_error / iterations) - 1; //gravity
  //Initialise lowpass filter
  Ax_prev = Ax_error;
  Ay_prev = Ay_error;
  Az_prev = Az_error + 1;

  //Gyro error
  i = 0;
  while(i < iterations){
    Wire.beginTransmission(MPU6050);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6, true);
    Gx = ( Wire.read() << 8| Wire.read());
    Gx = Gx/131.0;
    Gy = ( Wire.read() << 8| Wire.read());
    Gy = Gy/131.0;
    Gz = ( Wire.read() << 8| Wire.read());
    Gz = Gz/131.0;
    Gx_error = Gx_error + Gx;
    Gy_error = Gy_error + Gy;
    Gz_error = Gz_error + Gz;  
    i++;
  }
  //Mean error
  Gx_error = Gx_error / iterations;
  Gy_error = Gy_error / iterations;
  Gz_error = Gz_error / iterations;  
  //Initialise lowpass filter
  Gx_prev = Gx_error;
  Gy_prev = Gy_error;
  Gz_prev = Gz_error;
}

void IMU_read(){
  //acc data
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
    Ax = (Wire.read() << 8| Wire.read());
    Ax = Ax/16384.0;
    Ay = (Wire.read() << 8| Wire.read());
    Ay = Ay/16384.0;
    Az = (Wire.read() << 8| Wire.read());
    Az = Az/16384.0;
  //acc lowpass filter
  float LP_acc = 0.14;
  Ax = (1.0 - LP_acc)*Ax_prev + LP_acc*Ax;
  Ay = (1.0 - LP_acc)*Ay_prev + LP_acc*Ay;
  Az = (1.0 - LP_acc)*Az_prev + LP_acc*Az;
  Ax_prev = Ax;
  Ay_prev = Ay;
  Az_prev = Az;
  //error correction
  Ax = Ax - Ax_error;
  Ay = Ay - Ay_error;
  Az = Az - Az_error;

  //gyro data
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
    Gx = (Wire.read() << 8| Wire.read());
    Gx = Gx/131.0;
    Gy = (Wire.read() << 8| Wire.read());
    Gy = Gy/131.0;
    Gz = (Wire.read() << 8| Wire.read());
    Gz = Gz/131.0;
  //gyro lowpass filter
  float LP_gyro = 0.1;
  Gx = (1.0 - LP_gyro)*Gx_prev + LP_gyro*Gx;
  Gy = (1.0 - LP_gyro)*Gy_prev + LP_gyro*Gy;
  Gz = (1.0 - LP_gyro)*Gz_prev + LP_gyro*Gz;
  Gx_prev = Gx;
  Gy_prev = Gy;
  Gz_prev = Gz;
  //error correction
  Gx = Gx - Gx_error;
  Gy = Gy - Gy_error;
  Gz = Gz - Gz_error;
}

void Madgwick() {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  //deg/s to rad/s
  Gx = Gx / rad2deg;
  Gy = Gy / rad2deg;
  Gz = Gz / rad2deg;
  //quaternion rate of change from gyro
  qDot1 = 0.5f * (-q1 * Gx - q2 * Gy - q3 * Gz);
  qDot2 = 0.5f * (q0 * Gx + q2 * Gz - q3 * Gy);
  qDot3 = 0.5f * (q0 * Gy - q1 * Gz + q3 * Gx);
  qDot4 = 0.5f * (q0 * Gz + q1 * Gy - q2 * Gx);

  if(!((Ax == 0.0f) && (Ay == 0.0f) && (Az == 0.0f))) {
    //normalise acc
    recipNorm = invSqrt(Ax * Ax + Ay * Ay + Az * Az);
    Ax = Ax * recipNorm;
    Ay = Ay * recipNorm;
    Az = Az * recipNorm;
    //aux vars to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;
    //gradient descent algoritm corrective step
    s0 = _4q0 * q2q2 + _2q2 * Ax + _4q0 * q1q1 - _2q1 * Ay;
    s1 = _4q1 * q3q3 - _2q3 * Ax + 4.0f * q0q0 * q1 - _2q0 * Ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * Az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * Ax + _4q2 * q3q3 - _2q3 * Ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * Az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * Ax + 4.0f * q2q2 * q3 - _2q2 * Ay;
    //normalise magnitude step
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 = s0 * recipNorm;
    s1 = s1 * recipNorm;
    s2 = s2 * recipNorm;
    s3 = s3 * recipNorm;
    //apply feedback
    qDot1 = qDot1 - (beta * s0);
    qDot2 = qDot2 - (beta * s1);
    qDot3 = qDot3 - (beta * s2);
    qDot4 = qDot4 - (beta * s3);
  }
  //integrate quaternion rate of change
  q0 = q0 + (qDot1 * dt);
  q1 = q1 + (qDot2 * dt);
  q2 = q2 + (qDot3 * dt);
  q3 = q3 + (qDot4 * dt);
  //normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 * recipNorm;
  q1 = q1 * recipNorm;
  q2 = q2 * recipNorm;
  q3 = q3 * recipNorm;
  //compute angles
  roll_measured = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*rad2deg;
  pitch_measured = asin(-2.0f * (q1*q3 - q0*q2))*rad2deg;
  yaw_measured = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*rad2deg;
}

void angleControl(){
  //error
  roll_error = roll_desired - roll_measured;
  pitch_error = pitch_desired - pitch_measured;
  yaw_error = yaw_desired - Gz;
  //proportional control
  p_roll = kp_roll*roll_error;
  p_pitch = kp_pitch*pitch_error;
  p_yaw = kp_yaw*yaw_error;
  //integral control
  i_roll = iPrev_roll + roll_error*dt;
  i_roll = ki_roll*constrain(i_roll, -25, 25);
  i_pitch = iPrev_pitch + pitch_error*dt;
  i_pitch = ki_pitch*constrain(i_pitch, -25, 25);
  i_yaw = iPrev_yaw + yaw_error*dt;
  i_yaw = ki_yaw*constrain(i_yaw, -25, 25);
  if(input_throttle < 1080){
    i_roll = 0;
    i_pitch = 0;
    i_yaw = 0;
  }
  //derivative control
  d_roll = -kd_roll*Gx;
  d_pitch = -kd_pitch*Gy;
  d_yaw = -kd_yaw*Gz;

  //output
  roll_pid = constrain(p_roll + i_roll + d_roll,-400, 400);

  pitch_pid = constrain(p_pitch + i_pitch + d_pitch, -400, 400);

  yaw_pid = constrain(p_yaw + i_yaw + d_yaw, -400, 400);

  iPrev_roll = i_roll;
  iPrev_pitch = i_pitch;
  iPrev_yaw = i_yaw;

}

void outputMix(){
  // output_portAil = constrain(roll_pid,-500.0, 500.0) + 1500.0;
  // output_stbdAil = constrain(roll_pid,-500.0, 500.0) + 1500.0;
  // output_Elev = constrain(pitch_pid,-500.0, 500.0) + 1500.0;
  // output_Throt = input_throttle;


  output_M1 = input_throttle - pitch_pid + roll_pid + yaw_pid; //Front Left
  output_M2 = input_throttle - pitch_pid - roll_pid - yaw_pid; //Front Right
  output_M3 = input_throttle + pitch_pid - roll_pid + yaw_pid; //Back Right
  output_M4 = input_throttle + pitch_pid + roll_pid - yaw_pid; //Back Left

  output_M1 = constrain(output_M1, 1000, 2000);
  output_M2 = constrain(output_M2, 1000, 2000);
  output_M3 = constrain(output_M3, 1000, 2000);
  output_M4 = constrain(output_M4, 1000, 2000);


}


void looprate(int rate){
  unsigned long time_check = micros();
  float looptime = (1.0/rate)*1000000;
  while (looptime > (time_check - time_current)){
    time_check = micros();
  }
}

float invSqrt(float x){
  float y;
  y = 1.0/sqrt(x);
  return(y);
}

void print_pid(){
  Serial.print("Roll");
  Serial.print(",");
  Serial.print("P =  ");
  Serial.print(p_roll);
  Serial.print(",");
  Serial.print("I =  ");
  Serial.print(i_roll);
  Serial.print(",");
  Serial.print("D =  ");
  Serial.print(d_roll);
  Serial.print(",");

  Serial.print("Pitch");
  Serial.print(",");
  Serial.print("P =  ");
  Serial.print(p_pitch);
  Serial.print(",");
  Serial.print("I =  ");
  Serial.print(i_pitch);
  Serial.print(",");
  Serial.print("D =  ");
  Serial.print(d_pitch);
  Serial.print(",");

  Serial.print("Yaw");
  Serial.print(",");
  Serial.print("P =  ");
  Serial.print(p_yaw);
  Serial.print(",");
  Serial.print("I =  ");
  Serial.print(i_yaw);
  Serial.print(",");
  Serial.print("D =  ");
  Serial.print(d_yaw);
  Serial.println(",");
}

void print_input(){
  Serial.print(input_throttle);
  Serial.print(",");
  Serial.print(input_roll);
  Serial.print(",");
  Serial.print(input_pitch);
  Serial.print(",");
  Serial.print(input_yaw);
  Serial.print(",");
  Serial.print(input_aux1);
  Serial.print(",");
  Serial.println(input_aux2);
}

void print_looptime(){
  Serial.println(micros() - time_current);
}

void print_desired(){
  Serial.print(roll_desired);
  Serial.print(",");
  Serial.print(pitch_desired);
  Serial.print(",");
  Serial.print(yaw_desired);
  Serial.println(",");
}

bool checkPWM(){
  if(input_throttle < 1030){
    input_throttle = 1000;
    input_roll = 1500;
    input_pitch = 1500;
    input_yaw = 1500;
    input_aux1 = 1000;
    input_aux2 = 1000;
    return(HIGH);
  }
  else{
    return(LOW);
  }
}

ISR(PCINT2_vect){
  current_count = micros();

  //Channel 1
  if(PIND & B00000100){
    if(last_CH1_state == 0){
      last_CH1_state = 1;
      counter_1 = current_count;
    }
  }
  else if(last_CH1_state == 1){
    last_CH1_state = 0;
    input_throttle = current_count - counter_1;
  }

  //Channel 2
  if(PIND & B00010000){
    if(last_CH2_state == 0){
      last_CH2_state = 1;
      counter_2 = current_count;
    }
  }
  else if(last_CH2_state == 1){
    last_CH2_state = 0;
    input_roll = current_count - counter_2;
  }

  //Channel 3
  if(PIND & B00100000){
    if(last_CH3_state == 0){
      last_CH3_state = 1;
      counter_3 = current_count;
    }
  }
  else if(last_CH3_state == 1){
    last_CH3_state = 0;
    input_pitch = current_count - counter_3;
  }

  //Channel 4
  if(PIND & B01000000){
    if(last_CH4_state == 0){
      last_CH4_state = 1;
      counter_4 = current_count;
    }
  }
  else if(last_CH4_state == 1){
    last_CH4_state = 0;
    input_yaw = current_count - counter_4;
  }

  //Channel 5
  if(PIND & B10000000){
    if(last_CH5_state == 0){
      last_CH5_state = 1;
      counter_5 = current_count;
    }
  }
  else if(last_CH5_state == 1){
    last_CH5_state = 0;
    input_aux1 = current_count - counter_5;
  }
}

ISR(PCINT0_vect){
  current_count = micros();

  //Channel 1
  if(PINB & B00000001){
    if(last_CH6_state == 0){
      last_CH6_state = 1;
      counter_6 = current_count;
    }
  }
  else if(last_CH6_state == 1){
    last_CH6_state = 0;
    input_aux2 = current_count - counter_6;
  }

}
