/* This code controls the tension in a cable by way of spooling and unspooling the cable.
 * The tension is read by a strain gauge, and a motor is controlled using PID. The angualr
 * position of the spool is measured to help in spooling the cable cleanly.
 * 
 * Author - Nick Slakov
 */


#include "HX711.h"                      // Library needed for the strain gauge amp

// Setup the scale
const int dOut = 6;
const int scaleClock = 7;

HX711 scale(dOut, scaleClock);
float calibration_factor = -440.8;      // To be tuned once spools are set up

// setup motor pins

const int slpPin = 8;
const int pwmPin = 9;
const int dirPin = 10;

int dirVal = HIGH;                      // Changed though loop
    
// setup PID

double pwmVal = 0;                      // The pwm value to be written to the motor
double tensionSet = 200;                // Grams - the desired cable tension
const int arraySize = 100;              // Number of errors to keep track of 
double errors[arraySize];               // Array of errors, loops like a circular buffer
int times[arraySize];                   // Record of time changes
unsigned long lastTime = 0;             // The time since last loop
int i = 0; 
int j = arraySize - 1;

const double Kp = 0.2;                  // Maybe 1?
const double Ki = 0.00001;              // 0.00005 or something
const double Kd = 1;                    // 1-5 ish

double pError = 0;
double iError = 0;
double dError = 0;

// Encoder interupt routine

const int encoderPinA = 3;
const int encoderPinB = 4;

volatile unsigned long encoder0Pos = 0; // Holds the position of the spool shaft

void setup() {
  Serial.begin(9600);

  // SCALE
  scale.set_scale();
  scale.tare();                         // Reset the scale to 0
  scale.set_scale(calibration_factor);

  // MOTOR
  pinMode(slpPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(slpPin, HIGH);           // Turns on the h-bridge
  digitalWrite(dirPin, dirVal);

  // ENCODER
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH);       // turn on pull-up resistor
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(1, doEncoder, CHANGE); // encoder pin on interrupt 0 - pin 2 and 1 is pin3

  Serial.println("start");               // So we know things are working

  lastTime = millis();             
}

void loop() {

  if(i == arraySize){
    i = 0;
  }
  if(j == arraySize){
    j = 0;
  }

  errors[i] = tensionSet - scale.get_units();  // Record the error
  times[i] = millis() - lastTime;              // Record the time change
  lastTime = millis();

  if(errors[i]*errors[j] < 0){                 //Check when error changes sign
    Serial.println("Swapped");
    int a = i + 1;
    for(int k=0; k < arraySize - 50; k++){     // Helps for faster changes, number of values set to zero can be tuned
      if(a == arraySize){
        a = 0;
      }
      errors[a] = 0;
      times[a] = 0;
      a++;
    }
  }

  pError = Kp*errors[i];                      // Proportional error
  iError = Ki*trapz(errors, times, i);        // Intergal error
  dError = Kd*deriv(errors, times, i);        // Derivative Error
  
  pwmVal = pError + iError + dError;

  if(pwmVal >= 0){
    dirVal = HIGH;
  }
  if(pwmVal < 0){
    dirVal = LOW;
    pwmVal = -pwmVal;
  }
  if(pwmVal > 80){                            // Truncated for testing (255 should be used in final version)
    pwmVal = 80;
  }
  
  digitalWrite(dirPin, dirVal);               // Write the direction
  analogWrite(pwmPin,pwmVal);                 // Write the PWM

  i++;
  j++;
}


/*  Numericaly integrate the error using the trapezoid
 *  rule. 
 */
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

/*  This function takes the numerical derivative of the
 *  error. The approximation only uses two points, as 
 *  derivatives at the edge of a function are 
 *  notoriously difficult to make accurate. The error is
 *  of order dt, where dt is the time between error terms.
 */
double deriv(double errorList[], int times[], int index){
  int n = index - 1;

  if (n < 0){
    n = arraySize - 1;
  }
  return (errorList[index] - errorList[n])/((double)times[index]);
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
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

