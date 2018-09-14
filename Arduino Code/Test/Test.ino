int ch;
int ledPin = 13;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available())
    {
      ch = Serial.read();
      switch (ch)
      {
        case 1:
          digitalWrite(ledPin, HIGH);
          break;
        case 0:
          digitalWrite(ledPin, LOW);
        
      }
    }

  

}
