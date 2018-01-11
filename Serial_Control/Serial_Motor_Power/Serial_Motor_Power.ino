


const int dirPin = 4;
const int pwmPin = 5;
const int slpPin = 6;
const int csPin = 7;

char inputValue = 0;        
int pwmValue = 0;     

double val = 0;


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
      Serial.println(analogRead(csPin));
    }
    else{
      pwmValue = map((inputValue-'0'), 0, 8, 0, 255);

      Serial.print("Wrote PWM value of: ");
      Serial.println(pwmValue);

      analogWrite(pwmPin,pwmValue);
    }
  }

  val = 0;
  for(int i = 0; i < 1000; i++){
    val += analogRead(csPin);
  }
  val = val/1000.0;

  val = val*5000.0/1023.0;
  val = (val - 29.0)/20.0;
  
  Serial.println(val);

  
  delay(2);
}
