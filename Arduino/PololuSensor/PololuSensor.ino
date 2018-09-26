/*=========================================================<br>Infra-Red distance measures: Send distance value in centimeters to serial
Written By Tiago A. on 21 Mar 2015
===========================================================
How to connect the sensor to the Arduino: 
  sensor - arduino
     VCC - 5V
     Gnd - GND
     OUT - A0
     EN  - Do not connect</p><p>Briefing:  
This program has the sole purpose of measuring distances with the Infra-Red sensor.
It uses a previous calculated regression to convert analogRead values into distance values.</p><p>How to use:
After uploading this program, the Arduino will start to send the distance of the sensor to the object through the serial port.
Don't forget to open the Serial Monitor in the Tools tab for debugging purposes. (Ctrl + Shift + M)
*/
const int irsensorpin = A0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop(){
  // Reads the InfraRed sensor analog values and convert into distance.
  int sensorValue = analogRead(irsensorpin);
  double IRdistance = 187754 * pow(sensorValue, -1.51);
  
  Serial.println(IRdistance);
  
  // A delay is added for a stable and precise input
  delay(100);
}
