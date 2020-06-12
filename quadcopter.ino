#include <Servo.h> // Electronic Speed Controllers
#include <Wire.h> // Reading data from the MPU6050

// PID Constants

float kPX = 2;
float kIX = 0;
float kDX = 0;

float kPY = 2;
float kIY = 0;
float kDY = 0;

float minAngle = 3;
float desiredAngleX = 0;
float desiredAngleY = 0;
float errorX = 0;
float errorY = 0;
float previousErrorX = 0;
float previousErrorY = 0;

float pidPX = 0;
float pidIX = 0;
float pidDX = 0;
float PIDX = 0;

float pidPY = 0;
float pidIY = 0;
float pidDY = 0;
float PIDY = 0;

// MPU-6050 Gyroscope and Accelerometer variables

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t rawAccelX = 0;
int16_t rawAccelY = 0;
int16_t rawAccelZ = 0;
int16_t rawGyroX = 0;
int16_t rawGyroY;
float accelX = 0;
float accelY = 0;
float accelZ = 0;
float gyroX = 0;
float gyroY = 0;
float rad_to_deg = 180 / 3.141592654;
float accelAngleX = 0;
float accelAngleY = 0;
float gyroAngleX = 0;
float gyroAngleY = 0;
boolean resetGyro = false;
float angleX = 0;
float angleY = 0;

// Time

float loadTime = 5000;
float startTime = 0;
float elapsedTime = 0;
float currentTime = 0;
float previousTime = 0;

// Controller input variables

double throttle = 0;
double pitch = 0;
double yaw = 0;

// Motor variables and constants

double motor1;
double motor2;
double motor3;
double motor4;

int motorRest = 1000;
int motorMin = 1200;
int motorMax = 1800;

// RadioLink R12DS receiver channel pins
byte ch1pin = 7; // Yaw
byte ch2pin = 6; // Pitch
byte ch3pin = 5; // Throttle

// Channel PWM configurations for RadioLink R12DS receiver
short pwmTolerance = 30;

short ch1max = 1890;
short ch1min = 1090;
short ch1rest = 1500;

short ch2max = 1890;
short ch2min = 1090;
short ch2rest = 1500;

short ch3max = 1890;
short ch3min = 1090;
short ch3rest = 1090;

// Electronic Speed Controller (ESC) constants
short esc1pin = 8;
short esc2pin = 9;
short esc3pin = 10;
short esc4pin = 11;

Servo ESC1, ESC2, ESC3, ESC4;

void setup() {

  // RadioLink R12DS receiver channel pins
  pinMode(ch1pin, INPUT);
  pinMode(ch2pin, INPUT);
  pinMode(ch3pin, INPUT);

  // Attach ECSs
  ESC1.attach(esc1pin);
  ESC2.attach(esc2pin);
  ESC3.attach(esc3pin);
  ESC4.attach(esc4pin);

  // Begin wire communication with the MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Begin serial communication for debugging
  Serial.begin(9600);
  startTime = millis();
}

void loop() {
  if(millis() - startTime > loadTime && resetGyro == false){
    desiredAngleX = angleX;
    desiredAngleY = angleY;
//    resetAngles();
    resetGyro = true;
  }
  
  resetMotors();

  calculateTime();
  pollGyroData();
//  printGyroData();
  calculateAngle();
//  printPreCalculatedAngle();
//  printAngleData();
 
  readControllerValues() ;
//  printControllerValues();
  calculateInputs();

  calculatePID();
  printPID();
  
  runMotors();
  delay(10);
}

void printPID(){
  Serial.print("PIDX: ");
  Serial.print(PIDX);
  Serial.print("   PIDY: ");
  Serial.print(PIDY);
  Serial.println("");
}

void calculatePID(){
  errorX = angleX - desiredAngleX;
  errorY = angleY - desiredAngleY;

  pidPX = kPX * errorX;
  pidPY = kPY * errorY;

  if(-minAngle < errorX < minAngle && kIX != 0){
    pidIX = pidIX + (kIX * errorX);  
  }
  if(-minAngle < errorY < minAngle && kIY != 0){
    pidIY = pidIY + (kIY * errorY);
  }

  if(kDX != 0){
    pidDX = kDX * ((errorX - previousErrorX) / elapsedTime);
  }
  if(kDY != 0){
    pidDY = kDY * ((errorY - previousErrorY) / elapsedTime);
  }

  //Individual lines for debugging
  PIDX = 0;
  PIDY = 0;
  
  PIDX += pidPX + pidIX + pidDX;  
  PIDY += pidPY + pidIY + pidDY;

  previousErrorX = errorX;
  previousErrorY = errorY;
}

void resetAngles(){
  angleX = 0;
  angleY = 0;
}

void printAngleData() {
  Serial.print("angleX: ");
  Serial.print(angleX);
  Serial.print("   angleY: ");
  Serial.print(angleY);
  Serial.println("");
}

void printPreCalculatedAngle(){
  Serial.print("accelAngleX: ");
  Serial.print(accelAngleX);
  Serial.print("   accelAngleY: ");
  Serial.print(accelAngleY);
  Serial.print("   gyroAngleX: ");
  Serial.print(gyroAngleX);
  Serial.print("   gyroAngleY: ");
  Serial.print(gyroAngleY);
  Serial.print("");
}

void calculateAngle() {
  accelAngleX = atan((accelY) / sqrt(pow((accelX), 2) + pow((accelZ), 2))) * rad_to_deg;
  accelAngleY = atan((accelX) / sqrt(pow((accelY), 2) + pow((accelZ), 2))) * rad_to_deg;
  
  gyroAngleX += gyroX * elapsedTime;
  gyroAngleY += gyroY * elapsedTime;
  
  float gyroToAccelRatio = 0.96;
  angleX = gyroToAccelRatio * (angleX + gyroX * elapsedTime) + (1 - gyroToAccelRatio) * accelAngleX;
  angleY = gyroToAccelRatio * (angleY + gyroY * elapsedTime) + (1 - gyroToAccelRatio) * accelAngleY;
}

void printGyroData() { // Prints the converted gyroscope and accelerometer data
  Serial.print("AccelX: ");
  Serial.print(accelX);
  Serial.print("   AccelY: ");
  Serial.print(accelY);
  Serial.print("   AccelZ: ");
  Serial.print(accelZ);
  Serial.print("   GyroX: ");
  Serial.print(gyroX);
  Serial.print("   GyroY: ");
  Serial.print(gyroY);
  Serial.println("");
}

void convertGyroData() { // Convert MPU-6050 data to known units
  gyroX = rawGyroX / 131.0; // 131.0 is the conversion factor for the raw data to degrees/second from the datasheet
  gyroY = rawGyroY / 131.0;
  accelX = rawAccelX / 16384.0; // 16384.0 is the conversion factor for the raw data to "g" units from the datasheet
  accelY = rawAccelY / 16384.0;
  accelZ = rawAccelZ / 16384.0;
}

void pollGyroData() {

  // Request accelerometer values
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // Ask for the 0x3B register - corresponds to accelX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); // Request a total of 6 registers

  // Values are stored in 2 registers - higher register shifted to the left and added using or |
  rawAccelX = Wire.read() << 8 | Wire.read();
  rawAccelY = Wire.read() << 8 | Wire.read();
  rawAccelZ = Wire.read() << 8 | Wire.read();

  // Request gyroscope values
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43); // Ask for 0x43 register - corresponds to gyroX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 4, true); // Request a total of 4 registers

  rawGyroX = Wire.read() << 8 | Wire.read();
  rawGyroY = Wire.read() << 8 | Wire.read();

  convertGyroData(); // Converts the raw gyroscope data to data in known units
}

void calculateTime() {
  previousTime = currentTime;  // the previous time is stored before the actual time read
  currentTime = millis();  // actual time read
  elapsedTime = (currentTime - previousTime) / 1000;
}

void calculateInputs() {
  double tolerance = 0.02;
  if (throttle > tolerance) {
    motor1 += throttle;
    motor2 += throttle;
    motor3 += throttle;
    motor4 += throttle;
  }
  if (pitch > tolerance || pitch < -tolerance) {
    if (pitch > 0) {
      motor1 += (pitch * 1 / 4);
      motor2 += (pitch * 1 / 4);
    }
    else if (pitch < 0) {
      motor3 += -(pitch * 1 / 4);
      motor4 += -(pitch * 1 / 4);
    }
  }
  if (yaw > tolerance || yaw < -tolerance) {
    if (yaw > 0) {
      motor4 += (yaw * 1 / 4);
      motor1 += (yaw * 1 / 4);
    }
    else if (yaw < 0) {
      motor3 += -(yaw * 1 / 4);
      motor2 += -(yaw * 1 / 4);
    }
  }
}

void resetMotors() {
  motor1 = 0;
  motor2 = 0;
  motor3 = 0;
  motor4 = 0;
}

void runMotors() {
  int motor1Speed = ((int) map(motor1 * 100, 0, 100, motorRest, motorMax));
  int motor2Speed = ((int) map(motor2 * 100, 0, 100, motorRest, motorMax));
  int motor3Speed = ((int) map(motor3 * 100, 0, 100, motorRest, motorMax));
  int motor4Speed = ((int) map(motor4 * 100, 0, 100, motorRest, motorMax));

  // PID Influence
  if(PIDY < 0){
    motor1Speed += -PIDY;
    motor2Speed += -PIDY;  
  }
  else if(PIDY > 0){
    motor3Speed += PIDY;
    motor4Speed += PIDY;
  }
  
  if(PIDX < 0){
    motor3Speed += -PIDX;
    motor2Speed += -PIDX;
  }
  else if(PIDY > 0){
    motor4Speed += PIDX;
    motor1Speed += PIDX;
  }

  if (motor1Speed < motorMin) {
    motor1Speed = motorRest;
  }

  if (motor2Speed < motorMin) {
    motor2Speed = motorRest;
  }

  if (motor3Speed < motorMin) {
    motor3Speed = motorRest;
  }

  if (motor4Speed < motorMin) {
    motor4Speed = motorRest;
  }

  if(motor1Speed > motorMax){
    motor1Speed = motorMax;
  }

  if(motor2Speed > motorMax){
    motor2Speed = motorMax;
  }
  
  if(motor3Speed > motorMax){
    motor3Speed = motorMax;
  }
  
  if(motor4Speed > motorMax){
    motor4Speed = motorMax;
  }

  ESC1.writeMicroseconds(motor1Speed);
  ESC2.writeMicroseconds(motor2Speed);
  ESC3.writeMicroseconds(motor3Speed);
  ESC4.writeMicroseconds(motor4Speed);
}

void readControllerValues() {
  double channel1;
  double channel2;
  double channel3;
  channel1 = pulseIn(ch1pin, HIGH);
  yaw = convertPWMtoPercent(channel1, ch1max, ch1min, ch1rest, pwmTolerance);
  channel2 = pulseIn(ch2pin, HIGH);
  pitch = convertPWMtoPercent(channel2, ch2max, ch2min, ch2rest, pwmTolerance);
  channel3 = pulseIn(ch3pin, HIGH);
  throttle = convertPWMtoPercent(channel3, ch3max, ch3min, ch3rest, pwmTolerance);
}

void printControllerValues() {
  Serial.print("throttle: ");
  Serial.print(throttle);
  Serial.print("   pitch: ");
  Serial.print(pitch);
  Serial.print("   yaw: ");
  Serial.println(yaw);
}

double convertPWMtoPercent(double pwm, double pwmMax, double pwmMin, double pwmRest, double pwmTolerance) {
  double value = 0.0;
  if (pwmRest > pwmMin) {
    if (pwm < pwmRest - pwmTolerance || pwm > pwmRest + pwmTolerance) {
      if (pwm > pwmRest) { // 0 to 1
        value = (((pwmMax - pwm) / (pwmMax - pwmRest)) - 1);
      }
      else { // 0 to -1
        value = -(((pwm - pwmMin) / (pwmRest - pwmMin)) - 1);
      }
      if (value < -1) {
        value = -1.0;
      }
      else if (value > 1) {
        value = 1.0;
      }
      return (value);
    }
    else {
      return (0.0); // Neutral position
    }
  }
  else if (pwmRest == pwmMin) {
    if (pwm > pwmMin + pwmTolerance / 2) {
      value = (pwm - pwmMin) / (pwmMax - pwmMin);
      if (value > 1) {
        value = 1.0;
      }
      else if (value < 0) {
        value = 0.0;
      }
      return (value);
    }
    else {
      return (0.0); // Neutral position
    }
  }
  else {
    return (0.0); // If all else fails, return an input of 0 so nothing happens
  }
}
