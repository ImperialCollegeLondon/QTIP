/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
https://www.instructables.com/id/ESC-Programming-on-Arduino-Hobbyking-ESC/
*/
#include <Servo.h>

int value = 0; // set values you need to zero

Servo firstESC, secondESC, thirdESC, fourthESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() {

  firstESC.attach(10);    // attached to pin 9 I just do this with 1 Servo
  secondESC.attach(11);
  thirdESC.attach(12);
  fourthESC.attach(13);
  Serial.begin(9600);    // start serial at 9600 baud

}

void loop() {

/* 1. Connect the ESC(s) powered off.
 * 2. Open the Serial monitor.
 * 3. Enter max speed PWM (2000) into the Serial monitor and send.
 * 4. Turn on the power. Should hear 3 beeps, a brief pause, and a 4th beep.
 * 5. Enter min speed PMW (600) and send. Should hear 2 shorts beeps and 1 long beep.
 * 6. Send values between 600 and 2000 to vary speed.
 */
  firstESC.writeMicroseconds(value);
  secondESC.writeMicroseconds(value);
  thirdESC.writeMicroseconds(value);
  fourthESC.writeMicroseconds(value);
 
  if(Serial.available()) 
    value = Serial.parseInt();    // Parse an Integer from Serial

}
