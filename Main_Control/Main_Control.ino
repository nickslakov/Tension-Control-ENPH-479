/* This code controls the tension in a cable by way of spooling and unspooling the cable.
 * The tension is read by a strain gauge, and a motor is controlled using PID. The angualr
 * position of the spool is measured to help in spooling the cable cleanly.
 * 
 * Author - Nick Slakov
 */


#include "HX711.h"                      // Library needed for the strain gauge amp
#include <Servo.h>

// SERVO CONSTANTS
const float conPI = 3.1415926535897932384626433832795;     // The hugely extra high accuracy of our apparatus demands this bizzare level of precision
const float spacing = 1.2;                                 // Number of wire diameters per wrap. If this is 1 they will be tightly packed
const float spoolD = 25.4*5.05/1000.0 ;                    // Spool Diameter in METRES
const float wireD = 0.0015;                                // Wire Diameter in METRES
const float rackLen = 0.08 ;                               // Length of Rack and Pinion Track in Metres
const float oneLayerLen = conPI*spoolD*rackLen/(wireD*spacing);
const int servoPin = 10;                                   //Where is the servo at? (Needs to be on 9 or 10, also can't use 9 or 10 for other pwm uses)

// SERVO VARIABLES
float cableOnSpool = 0;                                   // Length of cable on spool in this layer
int layer = 1;
float minangle = 0;
float maxangle = 170;
float linServCng = 0; 
long numTicks = 0;    
long lastTicks = 0;                                       // Number of Ticks on the encoder last time
float rotations = 0;
int dirSign = 1;                                          // Switches when we're moving the other way on the spool, 1 = toward motor when unspooling

Servo servo;

// SCALE CONSTANTS
const int scaleClock = 8;
const int dOut = 9;                                       // White cable with black marks
const float calibration_factor = -440.8;                  // Measures force in grams

HX711 scale(dOut, scaleClock);

// MOTOR CONSTANTS

const int dirPin = 4;
const int pwmPin = 5;
const int slpPin = 6;
const int fltPin = 7;
const int csPin = A7;

// MOTOR VAIRABLES

int dirVal;                         // Which way are we going? 
double pwmVal;                      // How fast are we going? 
    
// PID CONSTANTS AND VARIABLES

const double tensionSet = 200.0;        // Grams - the desired cable tension
double curError = 0;                    // Current error reading
double lastError = 0;                   // Last error reading
double errorInt = 0;                    // Integral of the error
double errorDeriv = 0;                  // Derivative of the error
unsigned long curTime;                  // Time at begining of current loop (constant through one loop, which is why we don't use millis())
unsigned long lastTime;                 // Time at at start of last loop

const double Kp = 0.06;                  // Maybe 0.06?
const double Ki = 0.00014;               // 0.00015 or something
const double Kd = 0.3;                   // 0.1 ish

double pError;
double iError;
double dError;

// ENCODER CONSTANTS AND VARIABLES

const int encoderPinA = 2;
const int encoderPinB = 3;

volatile long encoderPos = 0;          // Holds the position of the spool shaft

// ON OFF SWITCH

const int switchPin = 11;


void setup() {
  Serial.begin(9600);                   // Probably won't need Serial running in final version?

  // SCALE
  scale.set_scale();
  scale.tare();                         // Reset the scale to 0
  scale.set_scale(calibration_factor);

  // MOTOR
  pinMode(slpPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(slpPin, HIGH);           // Turns on the h-bridge 

  // ATTACH SERVO
  servo.attach(servoPin);

  // ENCODER
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH);       // turn on pull-up resistor
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(0, doEncoder, CHANGE); // initiallize the interupt routine (0 means pin2 and 1 means pin3 for some reason)

  // SWITCH
  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH);        // pull up

  

  Serial.println("start");               // Good to know things have begun

  servo.write(minangle);                 // Best to start at the start?

  lastTime = millis();                   // It is currently "lastTime"

}

void loop() {


  if(digitalRead(switchPin)) {

    curTime = millis();
    lastTime = curTime;
    lastError = 0;

    errorInt = 0;
    
    dirVal = HIGH;
    pwmVal = 0;
  }
  else {
    curError = tensionSet - scale.get_units()/2.0;  // Record the error, divide by 2 since the cable runs in and out 
    curTime = millis();                             

    errorInt += ((double)(curTime - lastTime))*(curError + lastError)/2.0;  // Running trapezoid rule integral
    errorDeriv = (curError - lastError)/((double)(curTime - lastTime));     // Derivative of error (pretty rough though, could use double point or something?)

    lastTime = curTime;
    lastError = curError;
  
    pError = Kp*curError;                      // Proportional error
    iError = Ki*errorInt;                      // Intergal error
    dError = Kd*errorDeriv;                    // Derivative Error
  
    pwmVal = pError + iError + dError;

    if(pwmVal >= 0){
      dirVal = LOW;
    }
    if(pwmVal < 0){
      dirVal = HIGH;
      pwmVal = -pwmVal;
    }

  
    if(pwmVal > 50){                            // Truncated for testing (255 should be used in final version)
      pwmVal = 50;
    }
    if(abs(encoderPos) > 600){                 // For testing. If the spool rotates more than once (~40cm), stop moving
      pwmVal = 0;
    }
  }
  

  Serial.println(curError);
  
  digitalWrite(dirPin, dirVal);               // Write the direction
  analogWrite(pwmPin,pwmVal);                 // Write the PWM



  numTicks = encoderPos;                              // Get the number of ticks from the encoder since last time the loop ran
  rotations = (numTicks-lastTicks)/600.0;             // Roations since last loop
  lastTicks = numTicks;
  cableOnSpool += rotations*spoolD*conPI;             // Total cable on the spool
  
  // Over and Underflow
  if (cableOnSpool > oneLayerLen) { 
    cableOnSpool -= oneLayerLen;
    dirSign = -dirSign;
    layer++;
    }
  if (cableOnSpool < 0){ 
    cableOnSpool += oneLayerLen;
    dirSign = -dirSign;
    layer--;
    }

   // Set servo position to the right place on the track
  if (dirSign > 0){
      servo.write(minangle + (maxangle-minangle)*cableOnSpool/oneLayerLen); 
    }
  else{
      servo.write(maxangle - (maxangle-minangle)*cableOnSpool/oneLayerLen); 
    }
  
}




/*  Function to call when interrupt routine is triggered.
 *  Takes no inputed and returns void, changes global 
 *  variables. Note that this function double counts,
 *  so while the encoder has 300 slots per revolution,
 *  this code wil increment encoder0Pos 600 times per 
 *  revolution.
 */
void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

