#include<Wire.h>
#include <Servo.h>

// Electronic Speed Controllers
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;
int ESC1_pin = 5; // temp val
int ESC2_pin = 6;
int ESC3_pin = 7;
int ESC4_pin = 8;

// Gyroscope
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// PID Constants
float P_pitch = 0.05; 
float I_pitch = 0;
int pitch_max = 0.5; 

float P_yaw = 0.05;
float I_yaw = 0;
int yaw_max = 0.5;

float P_roll = 0.05;
float I_roll = 0;
int roll_max = 0.05;

void setup() {
//  startGyro();
  attachESC();  
}

void loop() {
//  pollGyro();
//  printGyroData();
//  delay(333);

  int M1Speed = 0;
  int M2Speed = 0;
  int M3Speed = 0;
  int M4Speed = 0;
  M1Speed = map(M1Speed, 0, 1, 0, 180); // scales values to servo library (value between 0 and 180)
  M2Speed = map(M2Speed, 0, 1, 0, 180);
  M3Speed = map(M3Speed, 0, 1, 0, 180);
  M4Speed = map(M4Speed, 0, 1, 0, 180);
  ESC1.write(potValue);    // Send the signal to the ESC
  ESC2.write(potValue);    
  ESC3.write(potValue);  
  ESC4.write(potValue);   
}

void startGyro(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void pollGyro(){ 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void printGyroData(){
  Serial.print(" AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
}

void attachESC(){
  // (pin, min pulse width, max pulse width in microseconds) 
  ESC1.attach(ESC1_pin, 1000, 2000); 
  ESC2.attach(ESC2_pin, 1000, 2000);
  ESC3.attach(ESC3_pin, 1000, 2000);
  ESC4.attach(ESC4_pin, 1000, 2000);
}
