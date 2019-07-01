#include <I2Cdev.h>
#include <Servo.h>
#include <I2C.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

MPU6050 mpu;

#define ACCELEROMETER_SENSITIVITY 16384.0
#define GYROSCOPE_SENSITIVITY 131.0
#define SCL 21
#define SDL 20
#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Initialise the angle variables for complementary filter
long double accelX, accelY, accelZ, accData[3];
float gForceX, gForceY, gForceZ;
long double gyroX, gyroY, gyroZ, gyrData[3];
float rotX, rotY, rotZ;

// Initialise the roll pitch yaw variables
float roll = 0;
float pitch = 0;
float yaw = 0;
int i = 0;

double rollTarget = 0;
double pitchTarget = 0;
double rollErrSum, lastRollErr, pitchErrSum, lastPitchErr;
double kp, ki, kd;
double lastTime = 0;
double dRollErr, rollError, dRoll, dPitchErr, pitchError, dPitch;
double lastRoll = 0;
double lastPitch = 0;
unsigned long timeChange = 0;
const float RADIANS_TO_DEGREES = 57.2958;
unsigned long prevMillis = 0;
float dt;


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

unsigned long timeBegin;
unsigned long endTime = 10000; // in millis

// Used in 'turnOff' function to assign new gains.
double newP;
double newI;
double newD;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  Serial.begin(115200);
  setupMPU();
  SetTunings(41,0.0005,8000);// Stable @1000 Minspeed
  setupESC();
  timeBegin = millis();
}


void loop() {
  if(lastTime<endTime){
    processDMPdata();
    feedbackLoop();
    exportData();
    //printData();
  }
  else{turnOff();}
}

void turnOff(){ // Turns off the motors after 'endTime' milliseconds. Can then restart the test with new gains or the same.
  firstESC.write(0);
  secondESC.write(0);
  thirdESC.write(0);
  fourthESC.write(0);
  Serial.println(F("Enter new tunings. Proportional: ")); // Enter 0 to use the same readings. Enter anything for I and D after.
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()); // wait for data
  newP = Serial.parseFloat();
  Serial.println(F("Enter new tunings. Integral: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()); // wait for data
  newI = Serial.parseFloat();
  Serial.println(F("Enter new tunings. Derivative: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()); // wait for data
  newD = Serial.parseFloat(); 
  Serial.println(F("Stand QTIP upright. Press any character to restart ESCs"));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()); // wait for data
  while (Serial.available() && Serial.read());
  if(newP!=0){SetTunings(newP,newI,newD);}
  timeBegin=millis();
  Serial.println(timeBegin);
  lastTime=0;
  rollError=0;
  pitchError=0;
  rollErrSum=0;
  pitchErrSum=0;
}

void exportData(){ // Print data in a format for plotting
      Serial.print(lastTime);
      Serial.print(",");
      Serial.print(ypr[0]*180/M_PI);
      Serial.print(",");
      Serial.print(ypr[1]*180/M_PI);
      Serial.print(",");
      Serial.println(ypr[2]*180/M_PI);
/*      Serial.print(",");
      Serial.print(ESCSpeedMIN + firstESCSpeed);
      Serial.print(",");
      Serial.print(ESCSpeedMIN + secondESCSpeed);
      Serial.print(",");
      Serial.print(ESCSpeedMIN + thirdESCSpeed);
      Serial.print(",");
      Serial.println(ESCSpeedMIN + fourthESCSpeed);
*/
}

void setupMPU(){ // Setup MPU6050 using the DMP.
    // join I2C bus (I2Cdev library doesn't do this automatically)
     I2c.begin();
     I2c.timeOut(50); 

    Serial.begin(115200);
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-12);
    mpu.setYGyroOffset(20);
    mpu.setZGyroOffset(-69);
    mpu.setXAccelOffset(-5679);
    mpu.setYAccelOffset(2988);
    mpu.setZAccelOffset(1532); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
}


void setupESC(){ // Setup ESCs for top motors.
  firstESC.attach(10);
  secondESC.attach(11);
  thirdESC.attach(12);
  fourthESC.attach(13);
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

void processDMPdata(){ // Process angle data from the DMP sensor fusion.
// if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) //{do something;}

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024 || fifoCount % 42 != 0) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}


void feedbackLoop(){ // Control feedback goes in here.
  
   /*How long since we last calculated*/
   unsigned long now = millis() - timeBegin;
   timeChange = now - lastTime;

   roll = ypr[2]*180/M_PI;
   
   /*Roll feedback*/
   rollError = rollTarget - roll;
   rollErrSum += (rollError * timeChange);
   dRoll = roll - lastRoll;
  
   /*Compute PID Output*/
   //thirdESCSpeed = (kp * rollError + ki * rollErrSum - kd * dRoll/1000);
   thirdESCSpeed = (kp * rollError + ki * rollErrSum - kd * dRoll/timeChange);
   if (thirdESCSpeed < 0){thirdESCSpeed = 0;}
   if (thirdESCSpeed > ESCSpeedDMAX){thirdESCSpeed = ESCSpeedDMAX;}
   if (thirdESCSpeed - thirdESCSpeedPrev > ESCRateCap){thirdESCSpeed = thirdESCSpeedPrev + ESCRateCap;}
   if (thirdESCSpeed - thirdESCSpeedPrev < -ESCRateCap){thirdESCSpeed = thirdESCSpeedPrev - ESCRateCap;}
   thirdESC.write(ESCSpeedMIN + thirdESCSpeed);
   
   //fourthESCSpeed = -(kp * rollError + ki * rollErrSum - kd * dRoll);
   fourthESCSpeed = -(kp * rollError + ki * rollErrSum - kd * dRoll/timeChange);
   if (fourthESCSpeed < 0){fourthESCSpeed = 0;}
   if (fourthESCSpeed > ESCSpeedDMAX){fourthESCSpeed = ESCSpeedDMAX;}
   if (fourthESCSpeed - fourthESCSpeedPrev > ESCRateCap){fourthESCSpeed = fourthESCSpeedPrev + ESCRateCap;}
   if (fourthESCSpeed - fourthESCSpeedPrev < -ESCRateCap){fourthESCSpeed = fourthESCSpeedPrev - ESCRateCap;}
   fourthESC.write(ESCSpeedMIN + fourthESCSpeed);

   pitch = ypr[1]*180/M_PI;

  /*Pitch feedback*/
   pitchError = pitchTarget - pitch;
   pitchErrSum += (pitchError * timeChange);
   dPitch = pitch - lastPitch;
  
   /*Compute PID Output*/
   //firstESCSpeed = -(kp * pitchError + ki * pitchErrSum - kd * dPitch);
   firstESCSpeed = (kp * pitchError + ki * pitchErrSum - kd * dPitch/timeChange);
   if (firstESCSpeed < 0){firstESCSpeed = 0;}
   if (firstESCSpeed > ESCSpeedDMAX){firstESCSpeed = ESCSpeedDMAX;}
   if (firstESCSpeed - firstESCSpeedPrev > ESCRateCap){firstESCSpeed = firstESCSpeedPrev + ESCRateCap;}
   if (firstESCSpeed - firstESCSpeedPrev < -ESCRateCap){firstESCSpeed = firstESCSpeedPrev - ESCRateCap;}
   firstESC.write(ESCSpeedMIN + firstESCSpeed);
   
   //secondESCSpeed = (kp * pitchError + ki * pitchErrSum - kd * dPitch);
   secondESCSpeed = -(kp * pitchError + ki * pitchErrSum - kd * dPitch/timeChange);
   if (secondESCSpeed < 0){secondESCSpeed = 0;}
   if (secondESCSpeed > ESCSpeedDMAX){secondESCSpeed = ESCSpeedDMAX;}
   if (secondESCSpeed - secondESCSpeedPrev > ESCRateCap){secondESCSpeed = secondESCSpeedPrev + ESCRateCap;}
   if (secondESCSpeed - secondESCSpeedPrev < -ESCRateCap){secondESCSpeed = secondESCSpeedPrev - ESCRateCap;}
   secondESC.write(ESCSpeedMIN + secondESCSpeed);

   firstESCSpeedPrev = firstESCSpeed;
   secondESCSpeedPrev = secondESCSpeed;
   thirdESCSpeedPrev = thirdESCSpeed;
   fourthESCSpeedPrev = fourthESCSpeed;
   
   /*Remember some variables for next time*/
   lastRoll = roll;
   lastPitch = pitch;
   //lastPitchErr = pitchError;
   lastTime = now;
  
}

// Set the PID gains.
void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}

void printData() {

  /*
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
  //*/

  //*
  Serial.print("Proportional: ");
  Serial.print(kp* rollError);
  Serial.print("  Integral: ");
  Serial.print(ki * rollErrSum);
  Serial.print("  Derivative ");
  Serial.println(-kd * dRoll/timeChange);
  //*/

  /*
  Serial.print("First: ");
  Serial.print(ESCSpeedMIN + firstESCSpeed);
  Serial.print("  Second: ");
  Serial.print(ESCSpeedMIN + secondESCSpeed);
  Serial.print("  Third: ");
  Serial.print(ESCSpeedMIN + thirdESCSpeed);
  Serial.print("  Fourth: ");
  Serial.print(ESCSpeedMIN + fourthESCSpeed);
  Serial.print("  dt: ");
  Serial.println(timeChange);
  //*/
}