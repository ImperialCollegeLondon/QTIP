bool CartState = false;
int sendInfo;
int receiveInfo;
int byteSent;
union ftb {
    float fval;
    byte bval[4];
};

ftb floatToBytes;
float x = 3.14159f;
float newfloat;

unsigned long then;
unsigned long now;

#define RESET 30
#define USBS Serial
#define MINI Serial1

void setup() {
    digitalWrite(RESET,HIGH);
    USBS.begin(115200);
    MINI.begin(115200);
}

void loop() {
    MINI.write(1);
    while(!MINI.available());
    receiveFloat();

    USBS.println(floatToBytes.fval, 5);
    USBS.println(floatToBytes.bval[3],HEX);
    USBS.println(floatToBytes.bval[2],HEX);
    USBS.println(floatToBytes.bval[1],HEX);
    USBS.println(floatToBytes.bval[0],HEX);
    if(floatToBytes.fval == 3.14159f){
      USBS.println("true");
    }
    else{
      USBS.println("false");
    }
    while(true);
}

void sendFloat(float fourBytes){
    floatToBytes.fval = fourBytes;
    MINI.write(floatToBytes.bval[0]);
    MINI.write(floatToBytes.bval[1]);
    MINI.write(floatToBytes.bval[2]);
    MINI.write(floatToBytes.bval[3]);
}

void receiveFloat(){
    MINI.readBytes(floatToBytes.bval, 4);
}
