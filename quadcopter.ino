#include <Servo.h> // Electronic Speed Controllers
#include <Wire.h> // Reading data from the MPU6050


// MPU-6050 Gyroscope and Accelerometer variables

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY;
float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;

// Time

float elapsedTime;
float currentTime;
float previousTime;

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
int motorMin = 1150;
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

void setup(){
  
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
}

void loop(){
  resetMotors();

  calculateTime();
  pollGyroData();
    
  readControllerValues() ;
  printControllerValues();
  calculateInputs();
  runMotors();
  delay(10);
}

void convertGyroData(){ // Convert MPU-6050 data to known units
  gyroX = rawGyroX / 131.0; // 131.0 is the conversion factor for the raw data to degrees/second from the datasheet
  gyroY = rawGyroY / 131.0;
  accelX = rawAccelX / 16384.0; // 16384.0 is the conversion factor for the raw data to "g" units from the datasheet
  accelY = rawAccelY / 16384.0;
  accelZ = rawAccelZ / 16384.0;
}

void pollGyroData(){
  
  // Request accelerometer values
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // Ask for the 0x3B register - corresponds to accelX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true); // Request a total of 6 registers

  // Values are stored in 2 registers - higher register shifted to the left and added using or |
  rawAccelX = Wire.read()<<8|Wire.read(); 
  rawAccelY = Wire.read()<<8|Wire.read();
  rawAccelZ = Wire.read()<<8|Wire.read();

  // Request gyroscope values
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43); // Ask for 0x43 register - corresponds to gyroX 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,4,true); // Request a total of 4 registers

  rawGyroX = Wire.read()<<8|Wire.read(); 
  rawGyroY = Wire.read()<<8|Wire.read();
}

void calculateTime(){
  previousTime = currentTime;  // the previous time is stored before the actual time read
  currentTime = millis();  // actual time read
  elapsedTime = (currentTime - previousTime) / 1000; 
}

void calculateInputs(){
  double tolerance = 0.02;
  if(throttle > tolerance){
    motor1 += throttle;
    motor2 += throttle;
    motor3 += throttle;
    motor4 += throttle;
  }
  if(pitch > tolerance || pitch < -tolerance){
    if(pitch > 0){
      motor1 += (pitch * 1/4);
      motor2 += (pitch * 1/4);
    }
    else if(pitch < 0){
      motor3 += -(pitch * 1/4);
      motor4 += -(pitch * 1/4);
    }
  }
  if(yaw > tolerance || yaw < -tolerance){
    if(yaw > 0){
      motor4 += (yaw * 1/4);
      motor1 += (yaw * 1/4);
    }
    else if(yaw < 0){
      motor3 += -(yaw * 1/4);
      motor2 += -(yaw * 1/4);
    }
  }
}

void resetMotors(){
  motor1 = 0;
  motor2 = 0;
  motor3 = 0;
  motor4 = 0;
}

void runMotors(){
  int motor1Speed = ((int) map(motor1*100, 0, 100, motorRest, motorMax));
  int motor2Speed = ((int) map(motor2*100, 0, 100, motorRest, motorMax));
  int motor3Speed = ((int) map(motor3*100, 0, 100, motorRest, motorMax));
  int motor4Speed = ((int) map(motor4*100, 0, 100, motorRest, motorMax));

  if(motor1Speed < motorMin){
    motor1Speed = motorRest;
  }

  if(motor2Speed < motorMin){
    motor2Speed = motorRest;
  }

  if(motor3Speed < motorMin){
    motor3Speed = motorRest;
  }

  if(motor4Speed < motorMin){
    motor4Speed = motorRest;
  }
  
  ESC1.writeMicroseconds(motor1Speed);
  ESC2.writeMicroseconds(motor2Speed);
  ESC3.writeMicroseconds(motor3Speed);
  ESC4.writeMicroseconds(motor4Speed);
}

void readControllerValues(){
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

void printControllerValues(){
   Serial.print("throttle: "); 
   Serial.print(throttle);
   Serial.print("   pitch: ");
   Serial.print(pitch);
   Serial.print("   yaw: ");
   Serial.println(yaw);
}

double convertPWMtoPercent(double pwm, double pwmMax, double pwmMin, double pwmRest, double pwmTolerance){
  double value = 0.0;
  if(pwmRest > pwmMin){
    if(pwm < pwmRest - pwmTolerance || pwm > pwmRest + pwmTolerance){
      if(pwm > pwmRest){ // 0 to 1
        value = (((pwmMax - pwm) / (pwmMax - pwmRest)) - 1);
      }
      else{ // 0 to -1
        value = -(((pwm - pwmMin) / (pwmRest - pwmMin)) - 1);
      }
      if(value < -1){
        value = -1.0;
      }
      else if(value > 1){
        value = 1.0;
      }
      return(value);
    }
    else{
      return(0.0); // Neutral position    
    }
  }
  else if(pwmRest == pwmMin){
    if(pwm > pwmMin + pwmTolerance/2){
      value = (pwm - pwmMin) / (pwmMax - pwmMin);
      if(value > 1){
        value = 1.0;
      }
      else if(value < 0){
        value = 0.0;
      }
      return(value);
    }
    else{
      return(0.0); // Neutral position
    }
  }
  else{
    return(0.0); // If all else fails, return an input of 0 so nothing happens
  }
}
