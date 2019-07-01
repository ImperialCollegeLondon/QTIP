/*
 * 
 *  This code is to drive the cart using the Mega
 * 
 */
// 64 counts per revolution (MOTOR) measured
// Corresponds to 2797 counts per revolution (OUTPUT SHAFT)



// Define the encoder variables
const byte encoder0pinALeft = 2;  // Left encoder A pin -> interrupt pin 0 / digital pin 2
const byte encoder0pinBLeft = 4;  // Left encoder B pin -> digital pin 4
const byte encoder0pinARight = 3; // Right encoder A pin -> interrupt pin 1 / digital pin 3
const byte encoder0pinBRight = 5; // Right encoder B pin -> digital pin 5
byte encoder0PinALeftLast; // Last A state on the left encoder (HIGH or LOW)
byte encoder0PinARightLast; // Last A state on the right encoder (HIGH or LOW)
int durationLeft; // number of pulses
int durationRight; // number of pulses
boolean DirectionLeft; // Left encoder rotation direction 
boolean DirectionRight; // Right encoder rotation direction


// initialise time interval
int timePrev = 0; // Last time
int timeNow = 0; // Current time
int timeChange = 0; // dt

// Define driver variables
int zeroSpeed = 5;   // Maximum PWM for which motors are stationary (May need tuning)
const byte driverPWMpinLeft = 8;   // Left driver PWM pin
const byte driverPWMpinRight = 9;  // Right driver PWM pin
int driverPWMdutyLeft = zeroSpeed;   // Set intial speed to zero.
int driverPWMdutyRight = zeroSpeed;  // Set initial speed to zero.
const byte directionPinLeft = 23;  // Left driver direction pin.
const byte directionPinRight = 22;   // Right driver direction pin.
int dutyIncrement = 5;  // Acceleration. Increment size for every signal.
int dutyLeftPrev = zeroSpeed; // Previous left motor speed intialised at zero.
int dutyRightPrev = zeroSpeed; // Previous right mover speed initialsed at 0.
int maxPWM = 50; // Top speed for the motors.

int ch; // byte value read from HC-05
int chOld=0; // previous byte value from the HC-05

void setup()
{  
  Serial.begin(115200); //Initialize serial port 0. HC-05 configured for 115200 baud.
  //EncoderInit();  //Initialize the enoders
  
  //Set PWM pins to outputs.
  pinMode(driverPWMpinLeft, OUTPUT);
  pinMode(directionPinLeft, OUTPUT);
  pinMode(driverPWMpinRight, OUTPUT);
  pinMode(directionPinRight, OUTPUT);
}
 
void loop()
{
    //Print speeds.
  //Serial.print("PulseLeft:");
  //Serial.println(durationLeft);
  //Serial.print(". PulseRight:");
  //Serial.println(durationRight);


  // Loop checks to see if a direction command has been sent. Directions are listed in the paper.
  // Use Serial.println(ch) with wheel motors off to get a feel for what the loop is doing.
  // If 'ch' is the same as in the previous 'ch' reading, increased wheel PWM by 'dutyIncrement'.
  // If 'ch' is 0, the wheel motor PWM is decreased by 'dutyIncrement'. A 0 is only sent if no button is held on the app.
  // The wheel motor PWMs are saturated between 'zeroSpeed' and 'maxPWM'.
  
  durationRight = 0;
  durationLeft = 0;
  if(Serial.available())
  {
    ch = Serial.read();
      DirectionLeft = digitalRead(directionPinLeft);
      DirectionRight = digitalRead(directionPinRight);
      switch (ch) {
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

    timeNow = millis();
    timeChange = timeNow - timePrev;
    if(timeChange > 500){ch = 0; analogWrite(driverPWMpinLeft, zeroSpeed); analogWrite(driverPWMpinRight, zeroSpeed);} 
         
    //Serial.print(". Input:");
    //Serial.println(ch);
    
  }
}

 
void EncoderInit() // Initialised encoder
{
  DirectionLeft = true;//default -> Forward  
  pinMode(encoder0pinBLeft,INPUT);  
  attachInterrupt(digitalPinToInterrupt(encoder0pinALeft), wheelSpeed, CHANGE);
  DirectionRight = true;//default -> Forward  
  pinMode(encoder0pinBRight,INPUT);  
  attachInterrupt(digitalPinToInterrupt(encoder0pinARight), wheelSpeed, CHANGE);
  
}


void wheelSpeed() // Compute the number of encoder pulses and direction during the loop.
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
