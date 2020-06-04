
double throttle = 0;
double pitch = 0;
double yaw = 0;

// RadioLink R12DS receiver channel pins
byte ch1pin = 9; // Yaw
byte ch2pin = 10; // Pitch
byte ch3pin = 11; // Throttle

// Channel PWM configurations for RadioLink R12DS receiver
short pwmTolerance = 25;

short ch1max = 1890;
short ch1min = 1090;
short ch1rest = 1500;

short ch2max = 1890;
short ch2min = 1090;
short ch2rest = 1500;

short ch3max = 1890;
short ch3min = 1090;
short ch3rest = 1090;

void setup(){

  // RadioLink R12DS receiver channel pins
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);

  Serial.begin(9600);
}

void loop(){
  readControllerValues() ;
  printControllerValues();
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
