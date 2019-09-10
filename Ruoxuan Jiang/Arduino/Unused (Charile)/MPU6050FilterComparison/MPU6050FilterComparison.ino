#include <I2Cdev.h>
#include <Servo.h>
#include <I2C.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

MPU6050 mpu;

#define MPU6050 0x68
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

long double accelX, accelY, accelZ, accData[3];
float gForceX, gForceY, gForceZ;

long double gyroX, gyroY, gyroZ, gyrData[3];
float rotX, rotY, rotZ;
float roll = 0;
float pitch = 0;
float yaw = 0;
int i = 0;

float pitchAcc = 0;
float rollAcc = 0;
float pitchGyr = 0;
float rollGyr = 0;

float timeChange = 0;

const float RADIANS_TO_DEGREES = 57.2958;

unsigned long lastTime = 0;
unsigned long timeBegin;
unsigned long endtime = 10000; // in millis

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
  Serial.begin(115200);
  setupMPU();
  timeBegin = millis();
}


void loop() {
  if(lastTime<endtime){
    processDMPdata();
    recordAccelRegisters();
    complementaryFilter();
    exportData();
    //printData();
  }
  else{turnOff();}
}

void turnOff(){
  Serial.println(F("Stand QTIP upright. Press any character to restart ESCs"));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()); // wait for data
  while (Serial.available() && Serial.read());
  lastTime=0;
  timeBegin=millis();
}

void exportData(){
      Serial.print(lastTime);
      Serial.print(",");
      Serial.print(rollAcc);
      Serial.print(",");
      Serial.print(rollGyr);
      Serial.print(",");
      Serial.println(roll);
}

void setupMPU(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    I2c.timeOut(50);
    I2c.begin();
    I2c.write(MPU6050, 0x6B, 0b00000000);
    I2c.write(MPU6050, 0x1B, 0x00000000);
    I2c.write(MPU6050, 0x1C, 0b00000000);
    
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

void processDMPdata(){
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

void recordAccelRegisters() {
  I2c.read(MPU6050, 0x3B, 6);
  accelX = I2c.receive() << 8 | I2c.receive();
  accelY = I2c.receive() << 8 | I2c.receive();
  accelZ = I2c.receive() << 8 | I2c.receive();
  processAccelData();
}

void processAccelData(){
  accData[0] = accelX;
  accData[1] = accelY; 
  accData[2] = accelZ;
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() { 
  I2c.read(MPU6050, 0x43, 6);
  gyroX = I2c.receive() << 8 | I2c.receive();
  gyroY = I2c.receive() << 8 | I2c.receive();
  gyroZ = I2c.receive() << 8 | I2c.receive();
  processGyroData();
}

void processGyroData() {
  gyrData[0] = gyroX; // 131.0;
  gyrData[1] = gyroY; // 131.0; 
  gyrData[2] = gyroZ; // 131.0;
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}

void complementaryFilter()
{
  if (!(accData[0] == 0 || accData[1] == 0 || accData[2] == 0))
  {            
    timeChange = float((millis() - lastTime))/1000;
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    rollGyr = (gyrData[0] / GYROSCOPE_SENSITIVITY) * timeChange + roll; // Angle around the X-axis
    pitchGyr = (gyrData[1] / GYROSCOPE_SENSITIVITY) * timeChange + pitch; // Angle around the Y-Axis

  // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan(-1*accData[0]/sqrt(pow(accData[1],2) + pow(accData[2],2)))*RADIANS_TO_DEGREES;
        rollAcc = atan(accData[1]/sqrt(pow(accData[0],2) + pow(accData[2],2)))*RADIANS_TO_DEGREES;
        pitch = pitchGyr * 0.98 + pitchAcc * 0.02;
  // Turning around the Y axis results in a vector on the X-axis
        roll = rollGyr * 0.98 + rollAcc * 0.02;
    //}
    lastTime = lastTime + timeChange*1000 - timeBegin;
  }
}

