#define encoder0PinA  3
#define encoder0PinB  4

volatile unsigned long encoder0Pos = 0;

const int slpPin = 8;
const int pwmPin = 9;
const int dirPin = 10;

char inputValue = 0;        
int pwmValue = 0;        
int dt = 500;
int rpm = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  
  // H-Bridge Steup
  pinMode(slpPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(slpPin, HIGH);
  digitalWrite(dirPin, HIGH);

  // Encoder interrupt routine setup
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(1, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2 and 1 is pin3
  Serial.println("start");                // a personal quirk
}

void loop() {
  
  if(Serial.available()){
    inputValue = Serial.read();

    Serial.print("Read value of: ");
    Serial.println(inputValue);

    if((inputValue-'0') == 9){
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
      pwmValue = map((inputValue-'0'), 0, 8, 0, 255);

      Serial.print("Wrote PWM value of: ");
      Serial.println(pwmValue);

      analogWrite(pwmPin,pwmValue);
    }
  }

  
  delay(2);
}

void doEncoder() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}
