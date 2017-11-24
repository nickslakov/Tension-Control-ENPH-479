//setup scale

#include "HX711.h"

const int scaleOut = 6;
const int scaleClock = 7;

HX711 scale(scaleOut, scaleClock);
float calibration_factor = -440.8; 

int scaleVal = 0;

// setup motor

const int slpPin = 8;
const int pwmPin = 9;
const int dirPin = 10;
const int csPin = A0;
    
int pwmValue = 0;        
double current = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  scale.set_scale(calibration_factor);

  pinMode(slpPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(slpPin, HIGH);
  digitalWrite(dirPin, HIGH);
}

void loop() {

  scaleVal = scale.get_units();

  pwmValue = map(scaleVal, 0, 5000, 0, 255);

  if( pwmValue > 255){
    pwmValue = 255;
  }
  if( pwmValue < 0){
    pwmValue = 0;
  }

  analogWrite(pwmPin,pwmValue);

  current = analogRead(csPin);
  current = map(current, 0, 1023, 0, 5);
  current = (current - 50.0)/20.0;
  
  Serial.println(current);
  
  delay(2);
}
