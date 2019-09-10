int receivedInfo;
union ftb {
    float fval;
    byte bval[4];
};
ftb floatToBytes;
float x = 3.141592f;

void setup() {
    Serial.begin(115200);
}

void loop() {
    if(Serial.read() == 1){
        while(Serial.read() >= 0);
        floatToBytes.fval = 3.14159f;
        sendFloat();
    }
}


void sendFloat(){
    Serial.write(floatToBytes.bval[0]);
    Serial.write(floatToBytes.bval[1]);
    Serial.write(floatToBytes.bval[2]);
    Serial.write(floatToBytes.bval[3]);
}

void receiveFloat(){
    Serial.readBytes(floatToBytes.bval, 4);
}
