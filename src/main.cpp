#include <Arduino.h>
#include "CRSFforArduino.h"
#include <Servo.h>
#include <Wire.h>

CRSFforArduino crsf = CRSFforArduino();

//function declaration
void IMU_init();
void IMU_error();
void IMU_read();
void Madgwick();
void angleControl();
// void rateControl();
void looprate(int);
float invSqrt(float);
void print_pid();
void print_input();
void print_desired();
void print_tunePitch();
void print_tuneRoll();
void print_tuneYaw();


//time vars
unsigned long time_current, time_prev;
float dt;

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
float roll_desired, pitch_desired, yaw_desired, roll_pid, pitch_pid, yaw_pid;
float roll_error, pitch_error, yaw_error, roll_errorPrev, pitch_errorPrev, yaw_errorPrev;
float p_roll, i_roll, iPrev_roll, d_roll, p_pitch, i_pitch, iPrev_pitch, d_pitch, p_yaw, i_yaw, iPrev_yaw, d_yaw;
float kp_rollAngle = 1.3, ki_rollAngle = 0.04, kd_rollAngle = 18.0;
float kp_pitchAngle = 1.3, ki_pitchAngle = 0.04, kd_pitchAngle = 18.0;
float kp_yaw = 4.0, ki_yaw = 0.02, kd_yaw = 0.0;

float kp_rollRate = 1.0, ki_rollRate = 0.04, kd_rollRate = 0.1;
float kp_pitchRate = 1.0, ki_pitchRate = 0.04, kd_pitchRate = 0.1;


float output_Throt, output_portAil, output_stbdAil, output_Elev;
float output_M1, output_M2, output_M3, output_M4;

byte roll_limit = 30, pitch_limit = 30;


bool boot = 1;
bool failsafe;

Servo aileronPort;
Servo aileronStbd;
Servo elevator;
Servo throttle;


void setup() {
  //begin serial comms
  Serial.begin(115200);

  //heartbeat
  pinMode(13, OUTPUT); 
  digitalWrite(13, HIGH);
  
  //begin crsf comms
  if(!crsf.begin()){
    Serial.println("Failed");
    while(1){
        ;
    }
  }

  //initialise IMU and calulate error
  IMU_init();
  IMU_error();

  //attach servos
  aileronPort.attach(4);
  aileronStbd.attach(5);
  elevator.attach(6);
  throttle.attach(11);

  //setup complete
  digitalWrite(13, LOW);
  Serial.println("Ready!");
}

void loop() {
  //timing
  time_prev = time_current;
  time_current = micros();
  dt = (time_current - time_prev)/1000000.0;

  //update crsf
  crsf.update();

  //update IMU, calculate vehicle attitude
  IMU_read();
  Madgwick();

  //angle controller
  angleControl();

  //servo output
  aileronPort.writeMicroseconds(roll_pid);
  aileronStbd.writeMicroseconds(roll_pid);
  elevator.writeMicroseconds(pitch_pid);
  throttle.writeMicroseconds(crsf.readRcChannel(3));


  looprate(2000);


  //debugging print functions
  // print_looptime();
  // print_pid();
  // print_input();
  // print_desired();
  // print_tunePitch();
  // print_tuneRoll();
  // print_tuneYaw();

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
  int iterations = 5000;
  //Acc error
  int i = 0;
  while(i < iterations){
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 6);
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
    Wire.requestFrom(MPU6050, 6);
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
  Wire.requestFrom(MPU6050, 6);
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
  Wire.requestFrom(MPU6050, 6);
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
  //map input to desired angle
  roll_desired = map(crsf.readRcChannel(1), 1000, 2000, -roll_limit, roll_limit);
  pitch_desired = map(crsf.readRcChannel(2), 1000, 2000, -pitch_limit, pitch_limit);
  //error
  roll_error = roll_desired - roll_measured;//degrees
  pitch_error = pitch_desired - pitch_measured;//degrees
  //proportional control
  p_roll = kp_rollAngle*roll_error;
  p_pitch = kp_pitchAngle*pitch_error;
  //integral control
  i_roll = iPrev_roll + roll_error*dt;
  i_roll = ki_rollAngle*constrain(i_roll, -25, 25);
  i_pitch = iPrev_pitch + pitch_error*dt;
  i_pitch = ki_pitchAngle*constrain(i_pitch, -25, 25);
  if(crsf.readRcChannel(3) < 1080){
    i_roll = 0;
    i_pitch = 0;
  }
  //derivative control
  d_roll = -kd_rollAngle*Gx;
  d_pitch = -kd_pitchAngle*Gy;
  //output
  roll_pid = constrain(p_roll + i_roll + d_roll,-400, 400);
  pitch_pid = constrain(p_pitch + i_pitch + d_pitch, -400, 400);
  //update vars
  iPrev_roll = i_roll;
  iPrev_pitch = i_pitch;
}

// void rateControl(){
//   //map input to desired rates
//   roll_desired = map(input_roll, 1000, 2000, -30, 30);
//   pitch_desired = map(input_pitch, 1000, 2000, -30, 30);
//   yaw_desired = -1*map(input_yaw, 1000, 2000, -30, 30);
//   //error
//   roll_error = roll_desired - Gx;//degrees/s
//   pitch_error = pitch_desired - Gy;//degrees/s
//   yaw_error = yaw_desired - Gz;//degrees/s
//   //proportional control
//   p_roll = kp_rollRate*roll_error;
//   p_pitch = kp_pitchRate*pitch_error;
//   p_yaw = kp_yaw*yaw_error;
//   //integral control
//   i_roll = iPrev_roll + roll_error*dt;
//   i_roll = ki_rollRate*constrain(i_roll, -25, 25);
//   i_pitch = iPrev_pitch + pitch_error*dt;
//   i_pitch = ki_pitchRate*constrain(i_pitch, -25, 25);
//   i_yaw = iPrev_yaw + yaw_error*dt;
//   i_yaw = ki_yaw*constrain(i_yaw, -25, 25);
//   if(input_throttle < 1080){
//     i_roll = 0;
//     i_pitch = 0;
//     i_yaw = 0;
//   }
//   //derivative control
//   d_roll = -kd_rollRate*((roll_error - roll_errorPrev)/dt);
//   d_pitch = -kd_pitchRate*((pitch_error - pitch_errorPrev)/dt);
//   d_yaw = -kd_yaw*((yaw_error - yaw_errorPrev)/dt);
//   //output
//   roll_pid = constrain(p_roll + i_roll + d_roll,-400, 400);
//   pitch_pid = constrain(p_pitch + i_pitch + d_pitch, -400, 400);
//   yaw_pid = constrain(p_yaw + i_yaw + d_yaw, -400, 400);
//   //update vars
//   iPrev_roll = i_roll;
//   iPrev_pitch = i_pitch;
//   iPrev_yaw = i_yaw;
//   roll_errorPrev = roll_error;
//   pitch_errorPrev = pitch_error;
//   yaw_errorPrev = yaw_error;
// }

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
  Serial.print("RC Channels <A: ");
  Serial.print(crsf.readRcChannel(1));
  Serial.print(", E: ");
  Serial.print(crsf.readRcChannel(2));
  Serial.print(", T: ");
  Serial.print(crsf.readRcChannel(3));
  Serial.print(", R: ");
  Serial.print(crsf.readRcChannel(4));
  Serial.print(", Aux1: ");
  Serial.print(crsf.readRcChannel(5));
  Serial.print(", Aux2: ");
  Serial.print(crsf.readRcChannel(6));
  Serial.print(", Aux3: ");
  Serial.print(crsf.readRcChannel(7));
  Serial.print(", Aux4: ");
  Serial.print(crsf.readRcChannel(8));
  Serial.println(">");
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


void print_tunePitch(){
  Serial.print(millis());
  Serial.print(", ");
  Serial.print(pitch_desired);
  Serial.print(", ");
  Serial.print(pitch_measured);
  Serial.print(", ");
  Serial.print(pitch_error);
  Serial.print(", ");
  Serial.print(pitch_pid);
  Serial.print(", ");
  Serial.print(p_pitch);
  Serial.print(", ");
  Serial.print(i_pitch);
  Serial.print(", ");
  Serial.print(d_pitch);
  Serial.println(", ");
}

void print_tuneRoll(){
  Serial.print(millis());
  Serial.print(", ");
  Serial.print(roll_desired);
  Serial.print(", ");
  Serial.print(roll_measured);
  Serial.print(", ");
  Serial.print(roll_error);
  Serial.print(", ");
  Serial.print(roll_pid);
  Serial.print(", ");
  Serial.print(p_roll);
  Serial.print(", ");
  Serial.print(i_roll);
  Serial.print(", ");
  Serial.print(d_roll);
  Serial.println(", ");
}

void print_tuneYaw(){
  Serial.print(millis());
  Serial.print(", ");
  Serial.print(yaw_desired);
  Serial.print(", ");
  Serial.print(yaw_measured);
  Serial.print(", ");
  Serial.print(yaw_error);
  Serial.print(", ");
  Serial.print(yaw_pid);
  Serial.print(", ");
  Serial.print(p_yaw);
  Serial.print(", ");
  Serial.print(i_yaw);
  Serial.print(", ");
  Serial.print(d_yaw);
  Serial.println(", ");
}