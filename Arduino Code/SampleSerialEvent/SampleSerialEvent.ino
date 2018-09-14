#include "Keyboard.h"

const byte encoder0pinA = 2;//A pin -> the interrupt pin 0 / digital pin 2
const byte encoder0pinB = 4;//B pin -> the digital pin 4
byte encoder0PinALast;
int duration;//the number of the pulses
boolean Direction;//the rotation direction 

const byte driverPWMpin = 8;
int driverPWMduty = 0; // 220;
const byte directionPin = 23; 

const int KEY_UP_ARROW = 218;
const int KEY_DOWN_ARROW = 217;
const int KEY_LEFT_ARROW = 216;
const int KEY_RIGHT_ARROW = 215;

void setup()
{  
  Serial.begin(57600);//Initialize the serial port
  pinMode(driverPWMpin, OUTPUT);
  pinMode(directionPin, OUTPUT);
}
 
void loop()
{
  Serial.print("Pulse:");
  //Serial.println(duration);
  duration = 0;
  digitalWrite(directionPin, true);
  analogWrite(driverPWMpin, driverPWMduty);
  delay(100);
}

void serialEvent()
{
  
}

