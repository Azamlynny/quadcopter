#include <Servo.h>

double throttle = 0;
double pitch = 0;
double yaw = 0;

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

short esc1min = 1200;
short esc1max = 1900; 

short esc2min;
short esc2max;

short esc3min;
short esc3max;

short esc4min;
short esc4max;

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
 
  Serial.begin(9600);
}

void loop(){
  resetMotors();
  readControllerValues() ;
  printControllerValues();
  calculateInputs();
  runMotors();
  delay(10);
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
