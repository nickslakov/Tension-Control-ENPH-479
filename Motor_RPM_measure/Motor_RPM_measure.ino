/*
 * Code for measuring the RPM of a shaft attached to a rotary encoder.
 * Calculated by measuring the number of encoder steps in a given 
 * time interval and changing units from encoder steps to RPM. Change 
 * the speed by inputing digits 0-8 with 0 being no nmotion, and 8 
 * being maximum. Read the speed by inputting the digit 9. Increase 
 * the variable "dt" for higher accuracy.
 */


#define encoder0PinA  2
#define encoder0PinB  3

volatile unsigned long encoder0Pos = 0;

const int dirPin = 4;
const int pwmPin = 5;
const int slpPin = 6;

char inputValue = 0;        
int pwmValue = 0;        
int dt = 500;                // Delay time (ms) for calculating RPM 
int rpm = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  
  // H-Bridge Steup
  pinMode(slpPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(slpPin, HIGH);             // Turn on H-bridge
  digitalWrite(dirPin, HIGH);             // Set direction 

  // Encoder interrupt routine setup
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0
  Serial.println("start");                
}

void loop() {
  
  if(Serial.available()){
    inputValue = Serial.read();

    Serial.print("Read value of: ");
    Serial.println(inputValue);

    if((inputValue-'0') == 9){
      // Code for finding RPM
      float a = micros();
      float position1 = encoder0Pos;
      delay(dt);
      float b = micros();
      float position2 = encoder0Pos;
      rpm = (position2 - position1)*1000*1000*60/((b-a)*600);
      Serial.print("rpm: ");
      Serial.println(rpm);
      
    }
    else{
      // Code for setting speed
      pwmValue = map((inputValue-'0'), 0, 8, 0, 255);

      Serial.print("Wrote PWM value of: ");
      Serial.println(pwmValue);

      analogWrite(pwmPin,pwmValue);
    }
  }

  
  delay(2);
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
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}
