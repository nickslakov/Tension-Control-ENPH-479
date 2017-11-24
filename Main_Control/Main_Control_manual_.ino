#include "HX711.h"

// Setup the scale
const int dOut = 6;
const int scaleClock = 7;

HX711 scale(dOut, scaleClock);
float calibration_factor = -440.8; // to be tuned

// setup motor pins

const int slpPin = 8;
const int pwmPin = 9;
const int dirPin = 10;

int dirVal = HIGH; // Changed though loop
    
// setup PID

double pwmValue = 0; // the pwm value to be written to the motor
double tensionSet = 200; // grams - the desired cable tension
const int arraySize = 100; // Number of errors to keep track of 
double errors[arraySize]; //array of errors, loops
int times[arraySize]; // dts
unsigned long lastTime = 0; // the time since last loop
int i = 0; 
int j = arraySize - 1;

const double Kp = 0.2;
const double Ki = 0.00001; // 0.00005 or something
const double Kd = 1; // 1-5 ish

double pError = 0;
double iError = 0;
double dError = 0;

// Encoder interupt routine

//const int encoderPinA = 3;
//const int encoderPinB = 4;

//volatile unsigned long encoder0Pos = 0;

void setup() {
  Serial.begin(9600);

  // SCALE
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  scale.set_scale(calibration_factor);

  // MOTOR
  pinMode(slpPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(slpPin, HIGH);
  digitalWrite(dirPin, dirVal);

  // ENCODER
  //pinMode(encoderPinA, INPUT);
  //digitalWrite(encoderPinA, HIGH);       // turn on pull-up resistor
  //pinMode(encoderPinB, INPUT);
  //digitalWrite(encoderPinB, HIGH);       // turn on pull-up resistor

  //attachInterrupt(1, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2 and 1 is pin3

  Serial.println("start");    

  lastTime = millis();             
}

void loop() {

  if(i == arraySize){
    i = 0;
  }
  if(j == arraySize){
    j = 0;
  }

  errors[i] = tensionSet - scale.get_units();
  times[i] = millis() - lastTime;
  lastTime = millis();

  if(errors[i]*errors[j] < 0){
    Serial.println("Swapped");
    int a = i + 1;
    for(int k=0; k < arraySize - 50; k++){
      if(a == arraySize){
        a = 0;
      }
      errors[a] = 0;
      times[a] = 0;
      a++;
    }
  }

  // Proportional error
  pError = Kp*errors[i];

  // Intergal error
  iError = Ki*trapz(errors, times, i);

  // Derivative Error
  dError = Kd*deriv(errors, times, i);
  
  pwmValue = pError + iError + dError;

  if(pwmValue >= 0){
    dirVal = HIGH;
  }
  if(pwmValue < 0){
    dirVal = LOW;
    pwmValue = -pwmValue;
  }
  if(pwmValue > 80){
    pwmValue = 80;
  }
  
  digitalWrite(dirPin, dirVal);
  analogWrite(pwmPin,pwmValue);

  Serial.println(pError);

  i++;
  j++;
  delay(10);
}

double trapz(double errorList[], int times[], int index){
  double integral = 0;
  int m = index;
  int n = index - 1;

  if(n < 0){
    n = arraySize - 1;
  }

  for(int j=0; j < arraySize ; j++){
    if (n >= arraySize){
      n = 0;
    }
    if (m >= arraySize){
      m = 0;
    }
    
    integral = integral + ((double)times[m])*(errorList[n]+errorList[m])/2.0;
    n++;
    m++;
  }
  
  return integral;
}


double deriv(double errorList[], int times[], int index){
  int n = index - 1;

  if (n < 0){
    n = arraySize - 1;
  }
  return (errorList[index] - errorList[n])/((double)times[index]);
}

//void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
//  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
 //   encoder0Pos++;
 // } else {
  //  encoder0Pos--;
  //}
//}

