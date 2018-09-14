
// 64 counts per revolution (MOTOR) measured
// Corresponds to 2797 counts per revolution (OUTPUT SHAFT)

const byte encoder0pinALeft = 2;//A pin -> the interrupt pin 0 / digital pin 2
const byte encoder0pinBLeft = 4;//B pin -> the digital pin 4
const byte encoder0pinARight = 3;
const byte encoder0pinBRight = 5;
byte encoder0PinALeftLast;
byte encoder0PinARightLast;
int durationLeft;//the number of the pulses
int durationRight;
boolean DirectionLeft;//the rotation direction 
boolean DirectionRight;

const byte driverPWMpinLeft = 8;
const byte driverPWMpinRight = 9;
int driverPWMdutyLeft = 90; // 220;
int driverPWMdutyRight = 90; // 220;
const byte directionPinLeft = 23; 
const byte directionPinRight = 22;

void setup()
{  
  Serial.begin(9600);//Initialize the serial port
  EncoderInit();//Initialize the module
  pinMode(driverPWMpinLeft, OUTPUT);
  pinMode(directionPinLeft, OUTPUT);
  pinMode(driverPWMpinRight, OUTPUT);
  pinMode(directionPinRight, OUTPUT);
}
 
void loop()
{
  Serial.print("PulseLeft:");
  Serial.println(durationLeft);
  Serial.print(". PulseRight:");
  Serial.println(durationRight);
  durationRight = 0;
  durationLeft = 0;
  while(Serial.available())
  {
    int ch = Serial.read();
    switch (ch) {
      case 'w':
        analogWrite(driverPWMpinLeft, 0);
        analogWrite(driverPWMpinRight, 0);
        delay(100);
        digitalWrite(directionPinLeft, true);
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
        digitalWrite(directionPinRight, false);
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 'a':
        analogWrite(driverPWMpinLeft, 0);
        analogWrite(driverPWMpinRight, 0);
        delay(100);
        digitalWrite(directionPinLeft, false);
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
        digitalWrite(directionPinRight, false);
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 's':
        analogWrite(driverPWMpinLeft, 0);
        analogWrite(driverPWMpinRight, 0);
        delay(100);
        digitalWrite(directionPinLeft, false);
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
        digitalWrite(directionPinRight, true);
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 'd':
      // put these cases into functions, e.g. 'forward, reverse, turnLeft, turnRight'
        analogWrite(driverPWMpinLeft, 0);
        analogWrite(driverPWMpinRight, 0);
        delay(100);
        digitalWrite(directionPinLeft, true);
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
        digitalWrite(directionPinRight, true);
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 'o':
       analogWrite(driverPWMpinLeft, 0);
       analogWrite(driverPWMpinRight, 0);
      }  
  }
  
}
 
void EncoderInit()
{
  DirectionLeft = true;//default -> Forward  
  pinMode(encoder0pinBLeft,INPUT);  
  attachInterrupt(digitalPinToInterrupt(encoder0pinALeft), wheelSpeed, CHANGE);
  DirectionRight = true;//default -> Forward  
  pinMode(encoder0pinBRight,INPUT);  
  attachInterrupt(digitalPinToInterrupt(encoder0pinARight), wheelSpeed, CHANGE);
}
 
void wheelSpeed()
{
  int LstateLeft = digitalRead(encoder0pinALeft);
  if((encoder0PinALeftLast == LOW) && LstateLeft==HIGH)
  {
    int valL = digitalRead(encoder0pinBLeft);
    if(valL == LOW && DirectionLeft)
    {
      DirectionLeft = false; //Reverse
    }
    else if(valL == HIGH && !DirectionLeft)
    {
      DirectionLeft = true;  //Forward
    }
  }
  encoder0PinALeftLast = LstateLeft;
 
  if(!DirectionLeft)  durationLeft++;
  else  durationLeft--;
  
  int LstateRight = digitalRead(encoder0pinARight);
  if((encoder0PinARightLast == LOW) && LstateRight==HIGH)
  {
    int valR = digitalRead(encoder0pinBRight);
    if(valR == LOW && DirectionRight)
    {
      DirectionRight = false; //Reverse
    }
    else if(valR == HIGH && !DirectionRight)
    {
      DirectionRight = true;  //Forward
    }
  }
  encoder0PinARightLast = LstateRight;
 
  if(!DirectionRight)  durationRight++;
  else  durationRight--;
}


