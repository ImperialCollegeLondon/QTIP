//defining Pins names for the code

int pwmr=2;
int pwml=3;

int dirr=30;
int dirl=32;

int encoderValuer=0;
int encoderValuel=0;
void countr(void);
void countl(void);

void setup(){

  Serial.begin(57600);
  pinMode(22,INPUT);
  pinMode(24,INPUT);
  pinMode(26,INPUT);
  pinMode(28,INPUT);
  attachInterrupt(2,countr,FALLING);
  attachInterrupt(3,countl,FALLING);
  encoderValuer=0;
  encoderValuel=0;

}

void loop(){

  analogWrite(pwmr,0);
  analogWrite(pwml,0);
  digitalWrite(dlrr,0);
  digitalWrite(dlrl,0);
  Serial.print("Starting\n");
  
  delay(3000);
  
  Serial.print("Encoder Value=");
  Serial.println(encoderValue);
  
  while(1)

  {
       
    // to print encoder value on the screen Serial.print("Encoder Value="); Serial.println(encoderValue); //Setting value of encoder
    
    //defining the while statement condition if(encoderValue<5000)
    
    break; // loop will break as soon as encoder value reaches 5000 or above digitalWrite(forward,1);

        
    digitalWrite(reverse,0);    
    analogWrite(pwm,150);
  
  }

digitalWrite(dirr,1);
digitalWrite(dirl,1);
analogWrite(pwm,255);

}

void countr(){

  encoderValuer++;

}

void countl(){

  encoderValuel++;

}
