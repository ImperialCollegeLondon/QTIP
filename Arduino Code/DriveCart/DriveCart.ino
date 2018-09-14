
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
int ch;

int timePrev = 0;
int timeNow = 0;
int timeChange = 0;


int zeroSpeed = 5;
const byte driverPWMpinLeft = 8;
const byte driverPWMpinRight = 9;
int driverPWMdutyLeft = zeroSpeed;
int driverPWMdutyRight = zeroSpeed;
const byte directionPinLeft = 23; 
const byte directionPinRight = 22;
int dutyIncrement = 5;
int dutyLeftPrev = zeroSpeed;
int dutyRightPrev = zeroSpeed;
int maxPWM = 50;

void setup()
{  
  Serial.begin(115200);//Initialize the serial port
  //EncoderInit();//Initialize the module
  pinMode(driverPWMpinLeft, OUTPUT);
  pinMode(directionPinLeft, OUTPUT);
  pinMode(driverPWMpinRight, OUTPUT);
  pinMode(directionPinRight, OUTPUT);
}
 
void loop()
{
  //Serial.println("Flag");
  //Serial.print("PulseLeft:");
  //Serial.println(durationLeft);
  //Serial.print(". PulseRight:");
  //Serial.println(durationRight);
  durationRight = 0;
  durationLeft = 0;
  if(Serial.available())
  {
    ch = Serial.read();
    DirectionLeft = digitalRead(directionPinLeft);
    DirectionRight = digitalRead(directionPinRight);
    switch (ch) {
      //delay(100);
      case 1:
        if(DirectionLeft == false){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        if(DirectionRight == true){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        digitalWrite(directionPinLeft, true);
        driverPWMdutyLeft += dutyIncrement;
        if (driverPWMdutyLeft > maxPWM) {driverPWMdutyLeft = maxPWM;}
        analogWrite(driverPWMpinLeft, 0.4*driverPWMdutyLeft);
        digitalWrite(directionPinRight, false);
        driverPWMdutyRight += dutyIncrement;
        if (driverPWMdutyRight > maxPWM) {driverPWMdutyRight = maxPWM;}
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 2:
      if(DirectionLeft == false){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        if(DirectionRight == true){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        digitalWrite(directionPinLeft, true);
        driverPWMdutyLeft += dutyIncrement;
        if (driverPWMdutyLeft > maxPWM) {driverPWMdutyLeft = maxPWM;}
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
        digitalWrite(directionPinRight, false); 
        driverPWMdutyRight += dutyIncrement;
        if (driverPWMdutyRight > maxPWM) {driverPWMdutyRight = maxPWM;}
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 3:
      if(DirectionLeft == false){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        if(DirectionRight == true){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        digitalWrite(directionPinLeft, true); 
        driverPWMdutyLeft += dutyIncrement;
        if (driverPWMdutyLeft > maxPWM) {driverPWMdutyLeft = maxPWM;}
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft);        
        digitalWrite(directionPinRight, false);        
        driverPWMdutyRight += dutyIncrement;
        if (driverPWMdutyRight > maxPWM) {driverPWMdutyRight = maxPWM;}
        analogWrite(driverPWMpinRight, 0.4*driverPWMdutyRight);
        break;
      case 4:
      if(DirectionLeft == true){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        if(DirectionRight == true){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        digitalWrite(directionPinLeft, false);        
        driverPWMdutyLeft += dutyIncrement;
        if (driverPWMdutyLeft > maxPWM) {driverPWMdutyLeft = maxPWM;}
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft);        
        digitalWrite(directionPinRight, false);        
        driverPWMdutyRight += dutyIncrement;
        if (driverPWMdutyRight > maxPWM) {driverPWMdutyRight = maxPWM;}
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 5:
        driverPWMdutyLeft = zeroSpeed;
        driverPWMdutyRight = zeroSpeed;
        analogWrite(driverPWMpinLeft, zeroSpeed);
        analogWrite(driverPWMpinRight, zeroSpeed);
        break;
      case 6:
      // put these cases into functions, e.g. 'forward, reverse, turnLeft, turnRight'
    
        if(DirectionLeft == false){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        if(DirectionRight == false){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        digitalWrite(directionPinLeft, true);
        driverPWMdutyLeft += dutyIncrement;
        if (driverPWMdutyLeft > maxPWM) {driverPWMdutyLeft = maxPWM;}
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
        digitalWrite(directionPinRight, true);
        driverPWMdutyRight += dutyIncrement;
        if (driverPWMdutyRight > maxPWM) {driverPWMdutyRight = maxPWM;}
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 7:
        if(DirectionLeft == true){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        if(DirectionRight == false){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        digitalWrite(directionPinLeft, false);
        driverPWMdutyLeft += dutyIncrement;
        if (driverPWMdutyLeft > maxPWM) {driverPWMdutyLeft = maxPWM;}
        analogWrite(driverPWMpinLeft, 0.4*driverPWMdutyLeft);        
        digitalWrite(directionPinRight, true);        
        driverPWMdutyRight += dutyIncrement;
        if (driverPWMdutyRight > maxPWM) {driverPWMdutyRight = maxPWM;}
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 8:
        if(DirectionLeft == true){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        if(DirectionRight == false){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        digitalWrite(directionPinLeft, false);
        driverPWMdutyLeft += dutyIncrement;
        if (driverPWMdutyLeft > maxPWM) {driverPWMdutyLeft = maxPWM;}
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft); 
        digitalWrite(directionPinRight, true);
        driverPWMdutyRight += dutyIncrement;
        if (driverPWMdutyRight > maxPWM) {driverPWMdutyRight = maxPWM;}
        analogWrite(driverPWMpinRight, driverPWMdutyRight);
        break;
      case 9:
        if(DirectionLeft == true){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        if(DirectionRight == false){driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;}
        digitalWrite(directionPinLeft, false);
        driverPWMdutyLeft += dutyIncrement;
        if (driverPWMdutyLeft > maxPWM) {driverPWMdutyLeft = maxPWM;}
        analogWrite(driverPWMpinLeft, driverPWMdutyLeft);        
        digitalWrite(directionPinRight, true);        
        driverPWMdutyRight += dutyIncrement;
        if (driverPWMdutyRight > maxPWM) {driverPWMdutyRight = maxPWM;}
        analogWrite(driverPWMpinRight, 0.4*driverPWMdutyRight);
        break;
      
      case 0:
       driverPWMdutyLeft -= dutyIncrement;
       driverPWMdutyRight -= dutyIncrement;
       if(driverPWMdutyLeft < zeroSpeed){driverPWMdutyLeft = zeroSpeed;}
       if(driverPWMdutyRight < zeroSpeed){driverPWMdutyRight = zeroSpeed;}
       analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
       analogWrite(driverPWMpinRight, driverPWMdutyRight);
      }
      timePrev = millis();
        
  }
  timeNow = millis();
  timeChange = timeNow - timePrev;
  if(timeChange > 500){ch = 0; analogWrite(driverPWMpinLeft, zeroSpeed); analogWrite(driverPWMpinRight, zeroSpeed);} 
          

  //Serial.print(". Input:");
  //Serial.println(ch);
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


