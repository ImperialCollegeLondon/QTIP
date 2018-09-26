//The sample code for driving one way motor encoder
const byte encoder0pinA1 = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB1 = 4;//B pin -> the digital pin 4
byte encoder0PinALast1;
int duration1;//the number of the pulses
boolean Direction1;//the rotation direction

const byte encoder0pinA2 = 3;//A pin -> the interrupt pin 0
const byte encoder0pinB2 = 5;//B pin -> the digital pin 4
byte encoder0PinALast2;
int duration2;//the number of the pulses
boolean Direction2;//the rotation direction 
 
 
void setup()
{  
  Serial.begin(57600);//Initialize the serial port
  EncoderInit();//Initialize the module
}
 
void loop()
{
  Serial.print("Pulse:");
  
  Serial.println(duration1);
  duration1 = 0;
  Serial.println(duration2);
  duration2 = 0;
  
  delay(100);
}
 
void EncoderInit()
{
  Direction1 = true;//default -> Forward  
  pinMode(encoder0pinB1,INPUT);  
  
  Direction2 = true;//default -> Forward  
  pinMode(encoder0pinB2,INPUT);  
  
  attachInterrupt(0, wheelSpeed, CHANGE);
}
 
void wheelSpeed()
{
  int Lstate1 = digitalRead(encoder0pinA1);
  if((encoder0PinALast1 == LOW) && Lstate1==HIGH)
  {
    int val1 = digitalRead(encoder0pinB1);
    if(val1 == LOW && Direction1)
    {
      Direction1 = false; //Reverse
    }
    else if(val1 == HIGH && !Direction1)
    {
      Direction1 = true;  //Forward
    }
  }
  encoder0PinALast1 = Lstate1;
 
  if(!Direction1)  duration1++;
  else  duration1--;

    int Lstate2 = digitalRead(encoder0pinA2);
  if((encoder0PinALast2 == LOW) && Lstate2==HIGH)
  {
    int val2 = digitalRead(encoder0pinB2);
    if(val2 == LOW && Direction2)
    {
      Direction2 = false; //Reverse
    }
    else if(val2 == HIGH && !Direction2)
    {
      Direction2 = true;  //Forward
    }
  }
  encoder0PinALast2 = Lstate2;
 
  if(!Direction2)  duration2++;
  else  duration2--;
}
