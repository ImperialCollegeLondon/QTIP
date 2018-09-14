/*
===Contact & Support===
Website: http://eeenthusiast.com/
Youtube: https://www.youtube.com/EEEnthusiast
Facebook: https://www.facebook.com/EEEnthusiast/
Patreon: https://www.patreon.com/EE_Enthusiast
Revision: 1.0 (July 13th, 2016)

===Hardware===
- Arduino Uno R3
- MPU-6050 (Available from: http://eeenthusiast.com/product/6dof-mpu-6050-accelerometer-gyroscope-temperature/)

===Software===
- Latest Software: https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
- Arduino IDE v1.6.9
- Arduino Wire library

===Terms of use===
The software is provided by EEEnthusiast without warranty of any kind. In no event shall the authors or 
copyright holders be liable for any claim, damages or other liability, whether in an action of contract, 
tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in 
the software.
*/

#include <Wire.h>
#include <Servo.h>

/*
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16
*/

#define ACCELEROMETER_SENSITIVITY 16384.0
#define GYROSCOPE_SENSITIVITY 131.0
#define SCL 21
#define SDL 20

#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

long double accelX, accelY, accelZ, accData[3];
float gForceX, gForceY, gForceZ;

long double gyroX, gyroY, gyroZ, gyrData[3];
float rotX, rotY, rotZ;
float *rollP;
float *pitchP;
float *yawP;
float roll = 0;
float pitch = 0;
float yaw = 0;
int i = 0;

const float RADIANS_TO_DEGREES = 57.2958;
float prevMillis = 0;
float dt;

Servo firstESC, secondESC;
int firstESCSpeed = 0;
int secondESCSpeed = 0;
int ESCSpeedMAX = 1800;
int ESCSpeedMIN = 700;
int value = 0;

void setup() {
  Serial.begin(38400);
  Wire.begin();
  setupMPU();
  setupESC();
}



void loop() {
  Serial.print("Flag1. ");
  recordAccelRegisters();
  Serial.print("Flag2. ");
  recordGyroRegisters();

  //delay(1);
  Serial.print("Flag3. ");

  Serial.print("Flag4. ");
  complementaryFilter();

  Serial.print("Flag5. ");
  feedbackLoop();

  Serial.print("Flag6. ");
  printData();

  Serial.print("Flag7. ");
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}



void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void setupESC(){
  firstESC.attach(10);    // attached to pin 11 I just do this with 1 Servo
  secondESC.attach(11);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
  firstESC.writeMicroseconds(value);
  Serial.write(ESCSpeedMAX);
  secondESC.writeMicroseconds(value);
  Serial.write(ESCSpeedMAX);

  Serial.println(F("Turn on the mofo. Enter char to continue."));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again  
}

void recordAccelRegisters() {
  Serial.println(" ");
  Serial.print("Blag1");
  Wire.beginTransmission(0b1101000); //I2C address of the MPU

  Serial.print("Blag2");
  Wire.write(0x3B); //Starting register for Accel Readings

  Serial.print("Blag3");
  Wire.endTransmission();
  
  Serial.print("Blag4 ");
  Serial.print(digitalRead(SCL));
  Serial.print(" ");
  Serial.print(digitalRead(SDA));
  delay(1);
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)

  Serial.print(" Blag5");
  if (!(Wire.available() < 6));{
  Serial.print("Blag6");
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  Serial.print("Blag7");
  }
  processAccelData();
  Serial.println("Blag8");
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
  Serial.println(" ");
  Serial.print("Clag1");
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Serial.print("Clag2");
  Wire.write(0x43); //Starting register for Gyro Readings
  Serial.print("Clag3");
  Wire.endTransmission();
  Serial.print("Clag4 ");
  Serial.print(digitalRead(SCL));
  Serial.print(" ");
  Serial.print(digitalRead(SDA));
  delay(1);
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  Serial.print(" Clag5");
  if (!(Wire.available() < 6));{
  Serial.print("Clag6");
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  Serial.print("Clag7");
  }
  processGyroData();
  Serial.print("Clag8");
}

void processGyroData() {
  gyrData[0] = gyroX; // 131.0;
  gyrData[1] = gyroY; // 131.0; 
  gyrData[2] = gyroZ; // 131.0;
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}



void feedbackLoop(){
  firstESCSpeed = gForceX * -2000; //M_PI/180 *-1;
  if(firstESCSpeed < 0){firstESCSpeed = 0;}
  firstESC.write(ESCSpeedMIN + firstESCSpeed); // pitch feedback
  secondESCSpeed = gForceX * 2000; //M_PI/180 *1;
  if(secondESCSpeed < 0){secondESCSpeed = 0;}
  secondESC.write(ESCSpeedMIN + secondESCSpeed);
}

void complementaryFilter()
{               
    dt = (millis() - prevMillis)/1000;
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    roll = (gyrData[0] / GYROSCOPE_SENSITIVITY) * dt + roll; // Angle around the X-axis
    pitch = (gyrData[1] / GYROSCOPE_SENSITIVITY) * dt + pitch; // Angle around the Y-Axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    //int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    //if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    //{
    //Serial.println((long int)accData[0]);
    //Serial.println((long int)accData[1]);
    //Serial.println((long int)accData[2]);
  // Turning around the X axis results in a vector on the Y-axis
        float pitchAcc = atan(-1*accData[0]/sqrt(pow(accData[1],2) + pow(accData[2],2)))*RADIANS_TO_DEGREES;
        float rollAcc = atan(accData[1]/sqrt(pow(accData[0],2) + pow(accData[2],2)))*RADIANS_TO_DEGREES;
        pitch = pitch * 0.98 + pitchAcc * 0.02;
  // Turning around the Y axis results in a vector on the X-axis
        roll = roll * 0.98 + rollAcc * 0.02;
    //}
    prevMillis = prevMillis + dt*1000;
}

void printData() {
  /*
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
  */
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("  Pitch: ");
  Serial.println(pitch);

}

