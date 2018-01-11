
const int slpPin = 4;
const int pwmPin = 5;
const int dirPin = 6;

char inputValue = 0;        
int pwmValue = 0;        

int dt = 500;
int rpm = 0;


// ENCODER CONSTANTS AND VARIABLES

const int encoderPinA = 2;
const int encoderPinB = 3;

volatile long encoderPos = 0;          // Holds the position of the spool shaft


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  pinMode(slpPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(slpPin, HIGH);
  digitalWrite(dirPin, HIGH);

    // ENCODER
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH);       // turn on pull-up resistor
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(0, doEncoder, CHANGE); // initiallize the interupt routine (0 means pin2 and 1 means pin3 for some reason)

  Serial.println("start");               // Good to know things have begun
}

void loop() {
  
  if(Serial.available()){
    inputValue = Serial.read();

    Serial.print("Read value of: ");
    Serial.println(inputValue);

    if((inputValue-'0') == 9){
      float a = micros();
      float position1 = encoderPos;
      delay(dt);
      float b = micros();
      float position2 = encoderPos;
      rpm = (position2 - position1)*1000*1000*60/((b-a)*600);
      Serial.print("rpm: ");
      Serial.println(rpm);
    }
    else{
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
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

