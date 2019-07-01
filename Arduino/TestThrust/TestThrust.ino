// TestThrust.ino
// Written to measure thrust of rotor 1

#include <I2Cdev.h>
#include <Servo.h>
#include <I2C.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

MPU6050 mpu;

#define ACCELEROMETER_SENSITIVITY 16384.0
#define GYROSCOPE_SENSITIVITY 131.0
#define SCL A5
#define SDA A4


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// ESC initialised
Servo firstESC, secondESC, thirdESC, fourthESC;
int firstESCSpeed = 0;
int firstESCSpeedPrev = 0;
int secondESCSpeed = 0;
int secondESCSpeedPrev = 0;
int thirdESCSpeed = 0;
int thirdESCSpeedPrev = 0;
int fourthESCSpeed = 0;
int fourthESCSpeedPrev = 0;
int ESCSpeedMAX = 2000;
int ESCSpeedMIN = 1000;
int ESCSpeedDMAX = 800; // Limits the range of ESC speeds. Eg. with ESCSpeedMIN=1000, and ESCSpeedDMAX at 800, the max speed is 1800.
int ESCRateCap = 500; // Limits the increments size of the ESC speed.
int value = 0;
int flag = 0;

unsigned long timeBegin;
unsigned long endTime = 10000; // in millis

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  Serial.begin(115200);
  //setupMPU();
  //SetTunings(41,0.0005,8000);// Stable @1000 Minspeed
  setupESC();
  timeBegin = millis();
}


void loop() {
    testRotors();
    setupESC();
}

void setupESC(){ // Setup ESCs for top motors.
  firstESC.attach(3);
  secondESC.attach(5);
  thirdESC.attach(6);
  fourthESC.attach(9);
  while (!Serial);
    
  firstESC.writeMicroseconds(value);
  Serial.write(ESCSpeedMAX);
  secondESC.writeMicroseconds(value);
  Serial.write(ESCSpeedMAX);
  thirdESC.writeMicroseconds(value);
  Serial.write(ESCSpeedMAX);
  fourthESC.writeMicroseconds(value);
  Serial.write(ESCSpeedMAX);

  Serial.println(F("Turn on the mofo. Enter char to continue."));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()); // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again  
  Serial.println("Flag");
}

void testRotors(){
    for(int iter = 1; iter<20; iter++){
        flag = 0;
        firstESCSpeed = iter*10;

        while(flag != 1){
            Serial.print("testing rotor 1 at ");
            Serial.println(firstESCSpeed);

            firstESC.write(firstESCSpeed);
            delay(10000);
            firstESC.write(0);

            Serial.println(F("Enter 0 to retest, 1 to continue"));
            while (Serial.available() && Serial.read()); // empty buffer
            while (!Serial.available()); // wait for data
            flag = Serial.parseInt();

            while (Serial.available() && Serial.read()); // empty buffer again  
            Serial.println("Flag");
        }
    }
}
