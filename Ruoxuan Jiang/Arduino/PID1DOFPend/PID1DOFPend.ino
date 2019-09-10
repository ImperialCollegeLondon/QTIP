// PID2DOFPend v.1.5.2 (stable)
// updated 24 June 2019
// drives ESCs in parallel to PIDCart_RTData or PIDCart_saveData
// on Arduino Pro Mini via Serial
// sends roll, pitch, and sampling time for each feedback iteration
// set enableRotors as false to disable ESCs for debugging

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
#define INTERRUPT_PIN 10  // use pin 2 on Arduino Uno & most boards


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Initialise the roll pitch yaw variables
float roll = 0;
int i = 0;

double rollTarget = 0;
double rollErrSum, lastRollErr;
double kp,ki,kd;
double dRollErr, rollError, dRoll;
double lastRoll = 0;

// times
unsigned long timeChange = 0;
unsigned long lastTime = 0;
unsigned long timeBegin;
unsigned long endTime = 20000; // duration of controller activation in milliseconds

union ftb {
    float fval;
    byte bval[4];
}; // union type declaration to convert float to bytes and vice versa
ftb floatToBytes; // union variable that converts float to bytes and vice versa

// ESC initialised
Servo firstESC, secondESC;
int firstESCSpeed = 0;
int firstESCSpeedPrev = 0;
int secondESCSpeed = 0;
int secondESCSpeedPrev = 0;
int ESCSpeedMAX = 2000;
int ESCSpeedMIN = 1000;
int ESCSpeed1 = 1000;
int ESCSpeed2 = 1000;
int ESCSpeedDMAX = 800; // Limits the range of ESC speeds. Eg. with ESCSpeedMIN=1000, and ESCSpeedDMAX at 800, the max speed is 1800.
int ESCRateCap = 500; // Limits the increments size of the ESC speed.
int value = 0;

// Used in 'turnOff' function to assign new gains.
double newP, newI, newD;

// debugger flags
bool enableRotors = true; // set to true to enable rotors
//bool enableRotorData = false; // set to true to print control inputs for ESC

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

unsigned short sysStatus = 0;

#define SETTING_UP 0
#define ESCSTANDBY 1
#define ESCSTARTED 2
#define ESCSTOPPED 3
#define WAITFOR_KP 4
#define WAITFOR_KI 5
#define WAITFOR_KD 6
#define ERROR_FAIL 7

// define 1-byte status flags to write to Mega
#define MINI_COMSETUP   1
#define MINI_I2CINIT    2
#define MINI_MPUTEST    3
#define MINI_MPUREADY   4
#define MINI_MPUFAIL    5
#define MINI_DMPINIT    6
#define MINI_DMPENABLE  7
#define MINI_DMPINTRUPT 8
#define MINI_DMPREADY   9
#define MINI_DMPFAIL1   10
#define MINI_DMPFAIL2   11
#define MINI_DMPFAIL0   12
#define MINI_ESCSTANDBY 13
#define MINI_ESCSTARTED 14
#define MINI_DATADUMP   15
#define MINI_ESCENDED   16
#define MINI_NEWGAINP   17
#define MINI_NEWGAINI   18
#define MINI_NEWGAIND   19
#define MINI_ERROR      20

// define 1-byte status flags to read from Mega
#define MEGA_COMSETUP   1
#define MEGA_STARTESC   2
#define MEGA_PRINTEND   3
#define MEGA_COMFLOAT   4

///////////////////////////////////////////////////////////////
//-----------------------EXECUTION LOOP----------------------//
///////////////////////////////////////////////////////////////

void setup(){
    Serial.begin(115200);
    while(Serial.read()>=0);
    setupConnection();
    setupMPU();
    setupESC();
    kp = 50;    ki = 0.01;   kd = 1000;
    sysStatus = ESCSTANDBY;
}

void loop(){
    switch(sysStatus){
        case ESCSTANDBY:
            Serial.write(MINI_ESCSTANDBY);
            while(!Serial.available());
            if(Serial.read() == MEGA_STARTESC){
                Serial.write(MINI_ESCSTARTED);
                sysStatus = ESCSTARTED;
                timeBegin = millis();
            }
            else{
                sysStatus = ERROR_FAIL;
            }
            break;
        case ESCSTARTED:
            if(lastTime<endTime){
                processDMPdata();
                feedbackLoop(); //the time recored as lastTime
                dumpData();
            }
            else{
                Serial.write(MINI_ESCENDED);
                sysStatus = ESCSTOPPED;
            }
            break;
        case ESCSTOPPED:
            turnOff();
            while(!Serial.available());
            if(Serial.read() == MEGA_PRINTEND){
                sysStatus = WAITFOR_KP;
            }
            else {
                sysStatus = ERROR_FAIL;
            }
            break;
        case WAITFOR_KP:
            Serial.write(MINI_NEWGAINP);
            receiveFloat();
            if(floatToBytes.fval == 0){
                sysStatus = ESCSTANDBY;
            }
            else{
                kp = floatToBytes.fval;
                sysStatus = WAITFOR_KI;
            }
            break;
        case WAITFOR_KI:
            Serial.write(MINI_NEWGAINI);
            receiveFloat();
            ki = floatToBytes.fval;
            sysStatus = WAITFOR_KD;
            break;                        
        case WAITFOR_KD:
            Serial.write(MINI_NEWGAIND);
            receiveFloat();
            kd = floatToBytes.fval;
            lastTime=0;
            rollError=0;
            rollErrSum=0;
            Serial.write(MINI_ESCSTANDBY);
            sysStatus = ESCSTANDBY;
            break;
        default:
            turnOff();
            Serial.write(MINI_ERROR);
            while(true);
            break;
    }
}

///////////////////////////////////////////////////////////////
//----------------SETUP FUNCTION DEFINITIONS-----------------//
///////////////////////////////////////////////////////////////

// wait for Mega's setup flag and return the flag
void setupConnection(){ 
    Serial.write(MINI_COMSETUP);
    while (!(Serial.available() && Serial.read() == 1)); // wait for reply flag
}

// Setup MPU6050 using the DMP.
void setupMPU(){ 
    // join I2C bus (I2Cdev library doesn't do this automatically)
    I2c.begin();
    I2c.timeOut(30); //50

    // initialize device
    Serial.write(MINI_I2CINIT); // Initialising I2C devices
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.write(MINI_MPUTEST); // Testing MPU connection
    if(mpu.testConnection()){
        Serial.write(MINI_MPUREADY); // MPU connection success
    }
    else{
        Serial.write(MINI_MPUFAIL); // MPU connection failed
    }

    // load and configure the DMP
    Serial.write(MINI_DMPINIT); // Initialising DMP
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
        Serial.write(MINI_DMPENABLE); // Enabling DMP
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.write(MINI_DMPINTRUPT); //"Enabling interrupt detection (Arduino external interrupt 0)..."
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.write(MINI_DMPREADY); //"DMP ready! Waiting for first interrupt..."
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else {
        if(devStatus == 1){ // initial memory load failed
            Serial.write(MINI_DMPFAIL1); // "DMP initialisation failed: error code 1"
        }
        else if(devStatus == 2){ // DMP configuration updates failed
            Serial.write(MINI_DMPFAIL2); // "DMP initialisation failed: error code 2"
        }
        else{
            Serial.write(MINI_DMPFAIL0); // "DMP initialisation failed: misc"
        }
    }
}

// Setup ESCs for top motors.
void setupESC(){
    firstESC.attach(3);
    secondESC.attach(5);
    //while (!Serial); // what does this do??

    firstESC.writeMicroseconds(value);
    //Serial.write(ESCSpeedMAX);
    secondESC.writeMicroseconds(value);
    //Serial.write(ESCSpeedMAX);
}

// Process angle data from the DMP sensor fusion.
void processDMPdata(){ 
    // see https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_raw/MPU6050_raw.ino
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    // while (!mpuInterrupt && fifoCount < packetSize) {
    //     if (mpuInterrupt && fifoCount < packetSize) {
    //         // try to get out of the infinite loop 
    //         fifoCount = mpu.getFIFOCount();
    //     }  
    //     // other program behavior stuff here
    //     // .
    //     // if you are really paranoid you can frequently test in between other
    //     // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    //     // while() loop to immediately process the MPU data
    //     // .
    // }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024 || fifoCount % 42 != 0) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    }
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02) {
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

// Turns off the motors after 'endTime' milliseconds. Can then restart the test with new gains or the same.
void turnOff(){ 
    firstESC.write(0);
    secondESC.write(0);
}

///////////////////////////////////////////////////////////////
//-----------------RUN FUNCTION DEFINITIONS------------------//
///////////////////////////////////////////////////////////////

// cast incoming 4 bytes in Serial1 read buffer to float
void receiveFloat(){
    while(Serial.available() < 4); // wait until 4+ bytes have arrived at Serial1 buffer
    Serial.readBytes(floatToBytes.bval, 4);
}

// cast float variable to 4 bytes and send to Serial
void sendFloat(float num){
    floatToBytes.fval = num;
    Serial.write(floatToBytes.bval[0]);
    Serial.write(floatToBytes.bval[1]);
    Serial.write(floatToBytes.bval[2]);
    Serial.write(floatToBytes.bval[3]);
}

// dump angle data to Arduino Mega (APM doesn't have enough storage)
void dumpData(){
    Serial.write(MINI_DATADUMP); // data dumping flag
    sendFloat(roll);

    if(timeChange < 255){
        Serial.write(timeChange);
    }
    else{
        Serial.write(0);
    }
}

// compute PID control feedback
void feedbackLoop(){ 
    /*How long since we last calculated*/
    unsigned long now = millis() - timeBegin;
    timeChange = now - lastTime;

    roll = ypr[2]*180/M_PI;

    /*Roll feedback*/
    rollError = rollTarget - roll;
    rollErrSum += (rollError * timeChange);
    dRoll = roll - lastRoll;

    /*Compute PID Output*/
    //u =  kp * rollError + ki * rollErrSum - kd * dRoll/timeChange;
    firstESCSpeed =  -(kp * rollError + ki * rollErrSum - kd * dRoll/timeChange);
    //firstESCSpeed = -(k * rollError - s * dRoll/timeChange);
    if (firstESCSpeed < 0){firstESCSpeed = 0;}
    if (firstESCSpeed > ESCSpeedDMAX){firstESCSpeed = ESCSpeedDMAX;}
    if (firstESCSpeed - firstESCSpeedPrev > ESCRateCap){firstESCSpeed = firstESCSpeedPrev + ESCRateCap;}
    if (firstESCSpeed - firstESCSpeedPrev < -ESCRateCap){firstESCSpeed = firstESCSpeedPrev - ESCRateCap;}

    //secondESCSpeed = (kp * pitchError + ki * pitchErrSum - kd * dPitch);
    secondESCSpeed = (kp * rollError + ki * rollErrSum - kd * dRoll/timeChange);
    //= (kp * rollError + ki * rollErrSum - kd * dRoll/timeChange);
    if (secondESCSpeed < 0){secondESCSpeed = 0;}
    if (secondESCSpeed > ESCSpeedDMAX){secondESCSpeed = ESCSpeedDMAX;}
    if (secondESCSpeed - secondESCSpeedPrev > ESCRateCap){secondESCSpeed = secondESCSpeedPrev + ESCRateCap;}
    if (secondESCSpeed - secondESCSpeedPrev < -ESCRateCap){secondESCSpeed = secondESCSpeedPrev - ESCRateCap;}

    
    if(enableRotors){
        firstESC.write(ESCSpeed1 + firstESCSpeed);
        secondESC.write(ESCSpeed2 + secondESCSpeed);
    }

    firstESCSpeedPrev = firstESCSpeed;
    secondESCSpeedPrev = secondESCSpeed;

    /*Remember some variables for next time*/
    lastRoll = roll;
    //lastPitchErr = pitchError;
    lastTime = now;
}
