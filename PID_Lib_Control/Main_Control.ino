#include <PID_v1.h>
#include "HX711.h"

// Setup the scale
const int dOut = 6;
const int scaleClock = 7;

HX711 scale(dOut, scaleClock);
float calibration_factor = -440.8; // to be tuned

double scaleVal = 0; // the value read from the scale also fed into the PID loop

// setup motor pins

const int slpPin = 8;
const int pwmPin = 9;
const int dirPin = 10;

int dirVal = HIGH;
    
// setup PID

double pwmValue = 0; // the pwm value to be written to the motor
double tensionSet = 200; // grams - the desired cable tension

double Kp = 5;
double Ki = 10;
double Kd = 0.5;

PID tensionPID(&scaleVal, &pwmValue, &tensionSet, Kp, Ki, Kd, DIRECT);

// Encoder interupt routine

const int encoderPinA = 3;
const int encoderPinB = 4;

volatile unsigned long encoder0Pos = 0;

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

  // PID
  tensionPID.SetMode(AUTOMATIC);

  // ENCODER
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH);       // turn on pull-up resistor
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(1, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2 and 1 is pin3

  Serial.println("start");                
}

void loop() {

  scaleVal = scale.get_units();
  tensionPID.Compute();

  pwmValue = (pwmValue/255.0-0.5)*60.0*2.0;

  if(pwmValue >= 0){
    dirVal = HIGH;
  }
  if(pwmValue < 0){
    dirVal = LOW;
    pwmValue = -pwmValue;
  }
  digitalWrite(dirPin, dirVal);
  analogWrite(pwmPin,pwmValue);

  //Serial.print("Scale value: ");
  //Serial.println(scaleVal);
  //Serial.print("PWM value: ");
  //Serial.println(pwmValue, DEC);
  //Serial.print("DIR value: ");
  //Serial.println(dirVal);
}


void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

