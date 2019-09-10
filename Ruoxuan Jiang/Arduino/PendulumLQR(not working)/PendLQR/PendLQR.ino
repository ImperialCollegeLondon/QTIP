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
float pitch = 0;
int i = 0;

double rollTarget = 0;
double pitchTarget = 0;
double lastTime = 0;
double dRoll, dPitch;
double lastRoll = 0;
double lastPitch = 0;

// ESC initialised
Servo ESC1, ESC2, ESC3, ESC4;
int u1 = 0;
int u1_prev = 0;
int u2 = 0;
int u2_prev = 0;
int u3 = 0;
int u3_prev = 0;
int u4 = 0;
int u4_prev = 0;
int speedMax = 1800;
int speedMin = 900;
int uRangeMax = 800; // Limits the range of ESC speeds. Eg. with ESCSpeedMIN=1000, and ESCSpeedDMAX at 800, the max speed is 1800.
int duMax = 500; // Limits the increments size of the ESC speed.
int value = 0;

unsigned long timeBegin;
unsigned long timeChange = 0;
unsigned long now = 0;
unsigned long endTime = 10000; // in millis
unsigned long anomalyCounter = 0;
#define Ts 15

// LQR gain matrix K
int K1 = 952;
int K2 = 180;
// #define K11 1800
// #define K12 4000
// #define K13 0
// #define K14 0
// #define K21 0
// #define K22 0
// #define K23 1800
// #define K24 360

int cancel = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    setupMPU();
    setupESC();
    timeBegin = millis();
    anomalyCounter = 0;
}

void loop() {
    if (lastTime < endTime && cancel == 0) {      
        processDMPdata();

        // compute time since last iteration
        now = millis() - timeBegin;
        timeChange = now - lastTime;
        // if timeChange < Ts then wait
        if(timeChange > 15){
            anomalyCounter++;
        }
        while(timeChange <= Ts) {
            now = millis() - timeBegin;
            timeChange = now - lastTime;
        }

        feedbackloop();

        // read serial for stop signal
        if(Serial.available()){
            cancel = Serial.parseInt();
        }
    }
    else {
        while (Serial.available() && Serial.read()); // empty buffer
        turnOff();
    }
}



void setupMPU() { 
    // Setup MPU6050 using the DMP.
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
    }
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void setupESC() { 
    // Setup ESCs for top motors.
    ESC1.attach(3);
    ESC2.attach(5);
    ESC3.attach(9);
    ESC4.attach(6);
    while (!Serial);

    ESC1.writeMicroseconds(value);
    Serial.write(speedMax);
    ESC2.writeMicroseconds(value);
    Serial.write(speedMax);
    ESC3.writeMicroseconds(value);
    Serial.write(speedMax);
    ESC4.writeMicroseconds(value);
    Serial.write(speedMax);

    Serial.println(F("Turn on the mofo. Enter char to continue."));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available()); // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again  
    Serial.println("Starting ESC. Enter anything else than 0 to stop.");
}

void processDMPdata() { 
    // Process angle data from the DMP sensor fusion.
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    //if (!mpuInterrupt && fifoCount < packetSize) //{do something;}

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

void turnOff() {
    ESC1.write(0);
    ESC2.write(0);
    ESC3.write(0);
    ESC4.write(0);

    Serial.print(F("Sampling time anomalies: "));
    Serial.println(anomalyCounter);

    Serial.println(F("Enter new tunings. Angular position cost: ")); // Enter 0 to use the same readings. Enter anything for I and D after.
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available()); // wait for data
    int newK1 = Serial.parseInt();
    Serial.println(F("Enter new tunings. Angular velocity cost: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available()); // wait for data
    int newK2 = Serial.parseInt();

    Serial.print("Begin testing K1, K2 values of: ");
    Serial.print(newK1);
    Serial.print(",");
    Serial.println(newK2);
    Serial.println(F("Stand QTIP upright. Press any character to restart ESCs."));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available()); // wait for data
    while (Serial.available() && Serial.read());
    if (newK1 != 0) {
        K1 = newK1;
        K2 = newK2;
    }
    lastTime = 0;
    lastRoll = 0;
    lastPitch = 0;
    u1_prev = 0;
    u2_prev = 0;
    u3_prev = 0;
    u4_prev = 0;
    anomalyCounter = 0;
    timeBegin = millis();
}

void feedbackloop() {
    roll = ypr[2]; // phi
    dRoll = (roll - lastRoll) / timeChange; // phidot/1000
    
    pitch = ypr[1]; // theta
    dPitch = (pitch - lastPitch) / timeChange; // thetadot/1000

    int ux = (int) -(K1*roll + 1000*K2*dRoll);
    int uy = (int) -(K1*pitch + 1000*K2*dPitch);

    if (ux > 0) {
        u1 = 0;
        u2 = 100+computeInput(ux, u2_prev);
    }
    else {
        u2 = 0;
        u1 = 100+computeInput(-ux, u1_prev);
    }

    if (uy > 0) {
        u3 = 0;
        u4 = 100+computeInput(uy, u4_prev);
    }
    else {
        u4 = 0;
        u3 = 100+computeInput(-uy, u3_prev);
    }

    ESC1.write(speedMin + u1);
    ESC2.write(speedMin + u2);
    ESC3.write(speedMin + u3);
    ESC4.write(speedMin + u4);

    //Serial.print(speedMin + u1);
    //Serial.print(",");
    //Serial.println(speedMin + u2);

    lastTime = now;
    lastRoll = roll;
    lastPitch = pitch;

    u1_prev = u1;
    u2_prev = u2;
    u3_prev = u3;
    u4_prev = u4;
}

int computeInput (int u, int u_prev) {
    if (u > uRangeMax) {
        u = uRangeMax;
    }
    if (u - u_prev > duMax) {
        u = u_prev + duMax;
    }
    if (u - u_prev < -duMax) {
        u = u_prev - duMax;
    }

    return u;
}
