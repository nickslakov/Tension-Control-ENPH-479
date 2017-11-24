
const int slpPin = 8;
const int pwmPin = 9;
const int dirPin = 10;

char inputValue = 0;        
int pwmValue = 0;        
int counter = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  pinMode(slpPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(slpPin, HIGH);
  digitalWrite(dirPin, HIGH);
}

void loop() {
  
  if(Serial.available()){
    inputValue = Serial.read();

    Serial.print("Read value of: ");
    Serial.println(inputValue);

    if((inputValue-'0') == 9){
      if(counter % 2 == 0){
        digitalWrite(dirPin, HIGH);
        Serial.println("Direction Forwards");
      }
      else{
        digitalWrite(dirPin, LOW);
        Serial.println("Direction Reversed");
      }
      counter = counter + 1;
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
