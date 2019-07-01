// PIDCart_RTdata v.1.3 (stable)
// updated 24 June 2019
// drives PID2DOFPend.ino on Arduino Pro Mini via Serial1
// prints roll, pitch, and sampling time on each feedback iteration in real time
// comment out function call to driveCart() or testCart() to disable cart movement

#include <Servo.h>
#include <Wire.h>

#define RESETMINI 30 // reset pin for Arduino Pro Mini

int durationLeft; //the number of the pulses
int durationRight;
boolean DirectionLeft; //the rotation direction 
boolean DirectionRight;
int ch;

int zeroSpeed = 5;
const byte driverPWMpinLeft = 8;
const byte driverPWMpinRight = 9;
int driverPWMdutyLeft = zeroSpeed;
int driverPWMdutyRight = zeroSpeed;
const byte directionPinLeft = 23; 
const byte directionPinRight = 22;
int dutyIncrement = 5;
int dutyLeftPrev = zeroSpeed;
int dutyRightPrev = zeroSpeed;
int maxPWM = 50;

union ftb {
    float fval;
    byte bval[4];
}; // union type declaration to convert float to bytes and vice versa
ftb floatToBytes; // union variable that converts float to bytes and vice versa

//#define ARRAYSIZE 600
//float angleLog[ARRAYSIZE][2];
//unsigned short timeLog[ARRAYSIZE];
//int savedData = 0;
//bool arrayOverflow = false;

unsigned short sysStatus = 0; // system status flag
unsigned short prevStatus = 0; // previous system status for debugging and error message
byte flagMini = 0;
bool skipReadSerial = false;
unsigned long then;
unsigned long now;

float newK;

//debugger flags
//bool enableCartTest = false;
//bool enableRotorData = false;

// renamed serial ports so I don't mistake Serial with Serial1
#define USBS Serial
#define MINI Serial1
// #define BLUE Serial2

int cartStatus = 0;
#define TESTDCSPEED 60
#define CARTSTART 6000
#define CARTTIME 300

#define MEGA_SETUP 0
#define MINI_SETUP 1
#define ESCSTANDBY 2
#define ESCSTARTED 3
#define ESCSTOPPED 4
#define WAITFOR_KP 5
#define WAITFOR_KI 6
#define WAITFOR_KD 7
#define ERROR_FAIL 8

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
    digitalWrite(RESETMINI, LOW);
    // Serial comms setup
    USBS.begin(115200); // USB serial comms with PC
    MINI.begin(115200); // TX1 and RX1 connecton to Mini
    while(USBS.read() >= 0);
    while(MINI.read() >= 0); // empty both serial buffers

    setupCart();
}

void loop(){
    skipReadSerial = false;
    switch(sysStatus){
        case MEGA_SETUP:
            digitalWrite(RESETMINI, HIGH);
            setupConnection();
            sysStatus = MINI_SETUP;
            skipReadSerial = true;
            break;
        case MINI_SETUP:
            while(!MINI.available());
            // print setup status messages in readSerial()
            // readSerial: if flagMini = MINI_ESCSTANDBY then sysStatus = ESCSTANDBY
            break;
        case ESCSTANDBY:
            USBS.println("Pendulum setup complete and on standby");
            USBS.println("Stand QTIP upright and turn on the main power switch");
            USBS.println("Enter any char to start rotors");
            while(!USBS.available()); // wait for USB startup flag
            while(USBS.read() >= 0); // empty USBS buffer
            cartStatus = 0;
            USBS.println("Enter 2 to stop");
            MINI.write(MEGA_STARTESC);
            while(!MINI.available());
            then = millis();
            // readSerial: if flagMini = MINI_ESCSTARTED then sysStatus := ESCSTARTED
            //                                                then = millis();
            break;
        case ESCSTARTED:
            //driveCart();
            testCart();
            if(USBS.available()){ // read for killSwitch
                if(USBS.read() == 2){
                    killSwitch();
                    sysStatus = MEGA_SETUP;
                    skipReadSerial = true;
                }
            }
            // readSerial: if flagMini = MINI_DATADUMP then printDataRealTime()
            //             if flagMini = MINI_ESCENDED then sysStatus := ESCSTOPPED
            break;
        case ESCSTOPPED:
            // printData();
            MINI.write(MEGA_PRINTEND);
            while(!MINI.available());
            // readSerial: flagMini = MINI_NEWGAINP
            break;
        case WAITFOR_KP:
            USBS.println("Enter new tunings. Enter zero to keep same settings.");
            USBS.println("Proportional:");
            sendNewGain();
            // readSerial: if flagMini = MINI_NEWGAINI then sysStatus := WAITFOR_KI
            //             if flagMini = MINI_ESCSTANDBY then sysStatus := ESCSTANDBY
            break;
        case WAITFOR_KI:
            USBS.println("Integral:");
            sendNewGain();
            break;
        case WAITFOR_KD:
            USBS.println("Derivative:");
            sendNewGain();
            break;
        case ERROR_FAIL:
        default:
            killSwitch();
            USBS.print("Error: sysStatus = ");
            USBS.print(prevStatus);
            USBS.print(", MINI message = ");
            USBS.println(flagMini);
            USBS.println("Reset Arduino Mega");
            while(true);
            break;
    }
    if(!skipReadSerial){
        sysStatus = readSerial(sysStatus);
    }
}

///////////////////////////////////////////////////////////////
//----------------SETUP FUNCTION DEFINITIONS-----------------//
///////////////////////////////////////////////////////////////

// set up cart driver
void setupCart(){
    pinMode(driverPWMpinLeft, OUTPUT);
    pinMode(directionPinLeft, OUTPUT);
    pinMode(driverPWMpinRight, OUTPUT);
    pinMode(directionPinRight, OUTPUT);
}

// sends a test setup flag to Mini and reads the reply
void setupConnection(){
    USBS.println("Waiting serial setup flag from Arduino Pro Mini...");
    while (!MINI.available()); // wait for data;
    if (MINI.read() == MINI_COMSETUP) {
        MINI.write(MEGA_COMSETUP);
        USBS.println("Serial connection successful.");
    }
    else {
        USBS.println("Serial connection failed.");
    }
}

// send new gain settings to APM and relevant flags
void sendNewGain(){
    while(!USBS.available());
    newK = USBS.parseFloat();
    USBS.println(newK,2);
    while(USBS.read() >= 0); // empty USBS buffer
    sendFloat(newK);
    while(!MINI.available());
}

// force restart APM
void killSwitch(){
    digitalWrite(RESETMINI, LOW);
    USBS.println("Resetting Arduino Pro Mini");
    delay(1000);
}

///////////////////////////////////////////////////////////////
//----------------------DATA PROCESSING----------------------//
///////////////////////////////////////////////////////////////

// cast float variable to 4 bytes and send to MINI
void sendFloat(float num){
    floatToBytes.fval = num;
    MINI.write(floatToBytes.bval[0]);
    MINI.write(floatToBytes.bval[1]);
    MINI.write(floatToBytes.bval[2]);
    MINI.write(floatToBytes.bval[3]);
}

// cast incoming 4 bytes in MINI read buffer to float
void receiveFloat(){
    while(MINI.available() < 4); // wait until 4+ bytes have arrived at MINI buffer
    MINI.readBytes(floatToBytes.bval, 4);
}

// receive dumped data and print in real time
void printDataRealTime(){
    receiveFloat();
    USBS.print(floatToBytes.fval,2); // roll

    receiveFloat();
    USBS.print(",");
    USBS.print(floatToBytes.fval,2); // pitch

    USBS.print(",");
    while(!MINI.available());
    USBS.println(MINI.read()); // dt
}

// // saves angle data (roll, pitch) one instance at a time
// void saveData(){
//     if(savedData < ARRAYSIZE){
//         receiveFloat(); // receive roll
//         angleLog[savedData][0] = floatToBytes.fval;
//         receiveFloat(); // receive float
//         angleLog[savedData][1] = floatToBytes.fval;

//         timeLog[savedData] = MINI.read();
//         savedData ++;
//     }
// }

// // print all data stored in angleLog
// void printData(){
//     for(int num = 0; num<savedData; num ++){
//         USBS.print(angleLog[num][0]);
//         USBS.print(",");
//         USBS.print(angleLog[num][1]);
//         USBS.print(",");
//         USBS.println(timeLog[num]);
//     }
// }

///////////////////////////////////////////////////////////////
//-----------------RUN FUNCTION DEFINITIONS------------------//
///////////////////////////////////////////////////////////////

// reads in incoming serial comms with Mini
unsigned short readSerial(unsigned short oldStat){
    prevStatus = oldStat;
    if(MINI.available()){
        flagMini = MINI.read();
        switch(flagMini){
            // setup status messages for MINI_SETUP
            case MINI_I2CINIT:      USBS.println("Initialising I2C devices...");
                                    return oldStat;
                                    break;
            case MINI_MPUTEST:      USBS.println("Testing MPU connection");
                                    return oldStat;
                                    break;
            case MINI_MPUREADY:     USBS.println("MPU connection success");
                                    return oldStat;
                                    break;
            case MINI_MPUFAIL:      USBS.println("MPU connection failed");
                                    return ERROR_FAIL;
                                    break;
            case MINI_DMPINIT:      USBS.println("Initialising DMP...");
                                    return oldStat;
                                    break;
            case MINI_DMPENABLE:    USBS.println("Enabling DMP...");
                                    return oldStat;
                                    break;
            case MINI_DMPINTRUPT:   USBS.println("Enabling interrupt detection");
                                    return oldStat;
                                    break;
            case MINI_DMPREADY:     USBS.println("DMP ready! Waiting for first interrupt...");
                                    return oldStat;
                                    break;
            case MINI_DMPFAIL1:     USBS.println("DMP initialisation failed: error code 1");
                                    return ERROR_FAIL;
                                    break;
            case MINI_DMPFAIL2:     USBS.println("DMP initialisation failed: error code 2");
                                    return ERROR_FAIL;
                                    break;
            case MINI_DMPFAIL0:     USBS.println("DMP initialisation failed: misc error");
                                    return ERROR_FAIL;
                                    break;

            // status transition flags
            case MINI_ESCSTANDBY:   return ESCSTANDBY;
                                    break;
            case MINI_ESCSTARTED:   return ESCSTARTED;
                                    then = millis();
                                    break;
            case MINI_DATADUMP:     printDataRealTime();
                                    //saveData();
                                    return oldStat;
                                    break;
            case MINI_ESCENDED:     return ESCSTOPPED;
                                    break;
            case MINI_NEWGAINP:     return WAITFOR_KP;
                                    break;
            case MINI_NEWGAINI:     return WAITFOR_KI;
                                    break;
            case MINI_NEWGAIND:     return WAITFOR_KD;
                                    break;

            // really not supposed to happen
            default:                return ERROR_FAIL;
                                    break;
        }
    }
    else{
        return oldStat;
    }
}

// drive cart forward from CARTTESTSTT till CARTTESTEND since ESC started
void testCart(){
    now = millis() - then;
    if(now > CARTSTART && now < CARTSTART+CARTTIME && cartStatus == 0){
        digitalWrite(directionPinLeft, true);
        digitalWrite(directionPinRight, false);
        analogWrite(driverPWMpinLeft, TESTDCSPEED);
        analogWrite(driverPWMpinRight, TESTDCSPEED);
        cartStatus ++;
    }
    else if (now > CARTSTART+CARTTIME && cartStatus != 0){
        analogWrite(driverPWMpinLeft, 0);
        analogWrite(driverPWMpinRight, 0);
    }
}

// Charlie's code for driving cart with bluetooth; modify serial channel (Serial2 or BLUE) as required
//void driveCart(){
//    //USBS.println("driveCart Flag");
//    durationRight = 0;
//    durationLeft = 0;
//    if (Serial.available()) {
//        ch = Serial.read();
//        Serial.println(ch);
//        DirectionLeft = digitalRead(directionPinLeft);
//        DirectionRight = digitalRead(directionPinRight);
//        switch (ch) {
//            //delay(100);
//            case 1:
//                if (DirectionLeft == false) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                if (DirectionRight == true) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                digitalWrite(directionPinLeft, true);
//                driverPWMdutyLeft += dutyIncrement;
//                if (driverPWMdutyLeft > maxPWM) {
//                    driverPWMdutyLeft = maxPWM;
//                }
//                analogWrite(driverPWMpinLeft, 0.4*driverPWMdutyLeft);
//                digitalWrite(directionPinRight, false);
//                driverPWMdutyRight += dutyIncrement;
//                if (driverPWMdutyRight > maxPWM) {
//                    driverPWMdutyRight = maxPWM;
//                }
//                analogWrite(driverPWMpinRight, driverPWMdutyRight);
//            break;
//
//            case 2:
//                if (DirectionLeft == false) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                if (DirectionRight == true) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                digitalWrite(directionPinLeft, true);
//                driverPWMdutyLeft += dutyIncrement;
//                if (driverPWMdutyLeft > maxPWM) {
//                    driverPWMdutyLeft = maxPWM;
//                }
//                analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
//                digitalWrite(directionPinRight, false); 
//                driverPWMdutyRight += dutyIncrement;
//                if (driverPWMdutyRight > maxPWM) {
//                    driverPWMdutyRight = maxPWM;
//                }
//                analogWrite(driverPWMpinRight, driverPWMdutyRight);
//            break;
//
//            case 3:
//                if (DirectionLeft == false) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                if (DirectionRight == true) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                digitalWrite(directionPinLeft, true); 
//                driverPWMdutyLeft += dutyIncrement;
//                if (driverPWMdutyLeft > maxPWM) {
//                    driverPWMdutyLeft = maxPWM;
//                }
//                analogWrite(driverPWMpinLeft, driverPWMdutyLeft);        
//                digitalWrite(directionPinRight, false);        
//                driverPWMdutyRight += dutyIncrement;
//                if (driverPWMdutyRight > maxPWM) {
//                    driverPWMdutyRight = maxPWM;
//                }
//                analogWrite(driverPWMpinRight, 0.4*driverPWMdutyRight);
//            break;
//
//            case 4:
//                if (DirectionLeft == true) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                if (DirectionRight == true) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                digitalWrite(directionPinLeft, false);        
//                driverPWMdutyLeft += dutyIncrement;
//                if (driverPWMdutyLeft > maxPWM) {
//                    driverPWMdutyLeft = maxPWM;
//                }
//                analogWrite(driverPWMpinLeft, driverPWMdutyLeft);        
//                digitalWrite(directionPinRight, false);        
//                driverPWMdutyRight += dutyIncrement;
//                if (driverPWMdutyRight > maxPWM) {
//                    driverPWMdutyRight = maxPWM;
//                }
//                analogWrite(driverPWMpinRight, driverPWMdutyRight);
//            break;
//
//            case 5:
//                driverPWMdutyLeft = zeroSpeed;
//                driverPWMdutyRight = zeroSpeed;
//                analogWrite(driverPWMpinLeft, zeroSpeed);
//                analogWrite(driverPWMpinRight, zeroSpeed);
//            break;
//
//            case 6:
//                // put these cases into functions, e.g. 'forward, reverse, turnLeft, turnRight'
//                if (DirectionLeft == false) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                if (DirectionRight == false) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                digitalWrite(directionPinLeft, true);
//                driverPWMdutyLeft += dutyIncrement;
//                if (driverPWMdutyLeft > maxPWM) {
//                    driverPWMdutyLeft = maxPWM;
//                }
//                analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
//                digitalWrite(directionPinRight, true);
//                driverPWMdutyRight += dutyIncrement;
//                if (driverPWMdutyRight > maxPWM) {
//                    driverPWMdutyRight = maxPWM;
//                }
//                analogWrite(driverPWMpinRight, driverPWMdutyRight);
//            break;
//
//            case 7:
//                if (DirectionLeft == true) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                if (DirectionRight == false) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                digitalWrite(directionPinLeft, false);
//                driverPWMdutyLeft += dutyIncrement;
//                if (driverPWMdutyLeft > maxPWM) {
//                    driverPWMdutyLeft = maxPWM;
//                }
//                analogWrite(driverPWMpinLeft, 0.4*driverPWMdutyLeft);        
//                digitalWrite(directionPinRight, true);        
//                driverPWMdutyRight += dutyIncrement;
//                if (driverPWMdutyRight > maxPWM) {
//                    driverPWMdutyRight = maxPWM;
//                }
//                analogWrite(driverPWMpinRight, driverPWMdutyRight);
//            break;
//
//            case 8:
//                if (DirectionLeft == true) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                if (DirectionRight == false) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                digitalWrite(directionPinLeft, false);
//                driverPWMdutyLeft += dutyIncrement;
//                if (driverPWMdutyLeft > maxPWM) {
//                    driverPWMdutyLeft = maxPWM;
//                }
//                analogWrite(driverPWMpinLeft, driverPWMdutyLeft); 
//                digitalWrite(directionPinRight, true);
//                driverPWMdutyRight += dutyIncrement;
//                if (driverPWMdutyRight > maxPWM) {
//                    driverPWMdutyRight = maxPWM;
//                }
//                analogWrite(driverPWMpinRight, driverPWMdutyRight);
//            break;
//
//            case 9:
//                if (DirectionLeft == true) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                if (DirectionRight == false) {
//                    driverPWMdutyLeft = zeroSpeed; driverPWMdutyRight = zeroSpeed;
//                }
//                digitalWrite(directionPinLeft, false);
//                driverPWMdutyLeft += dutyIncrement;
//                if (driverPWMdutyLeft > maxPWM) {
//                    driverPWMdutyLeft = maxPWM;
//                }
//                analogWrite(driverPWMpinLeft, driverPWMdutyLeft);        
//                digitalWrite(directionPinRight, true);        
//                driverPWMdutyRight += dutyIncrement;
//                if (driverPWMdutyRight > maxPWM) {
//                    driverPWMdutyRight = maxPWM;
//                }
//                analogWrite(driverPWMpinRight, 0.4*driverPWMdutyRight);
//            break;
//
//            case 0:
//                driverPWMdutyLeft -= dutyIncrement;
//                driverPWMdutyRight -= dutyIncrement;
//                if (driverPWMdutyLeft < zeroSpeed) {
//                    driverPWMdutyLeft = zeroSpeed;
//                }
//                if (driverPWMdutyRight < zeroSpeed) {
//                    driverPWMdutyRight = zeroSpeed;
//                }
//                analogWrite(driverPWMpinLeft, driverPWMdutyLeft);
//                analogWrite(driverPWMpinRight, driverPWMdutyRight);
//        }
//
//        timePrev = millis();
//    }
//
//    timeNow = millis();
//    timeChangeCart = timeNow - timePrev;
//    if (timeChangeCart > 500) {
//        ch = 0; analogWrite(driverPWMpinLeft, zeroSpeed);
//        analogWrite(driverPWMpinRight, zeroSpeed);
//    }
//}
