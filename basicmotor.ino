#include <PID_v1.h>

#define ain1 2
#define ain2 3
#define pwma 4
#define aeoa 23
#define aeob 22
#define bin1 8    
#define bin2 9
#define pwmb 10
#define beoa 14
#define beob 15
#define stby 6
#define CIRC 3.14159265359*40.2

int motorTime = 0;
int aRPS = 0;
int bRPS = 0;
volatile int aCount = 0;
volatile int aTurns = 0;
volatile int bCount = 0;
volatile int bTurns = 0;
int tickcount = 180;
//bool stopped = false;

double Input, Output, Setpoint;
Setpoint = 0;
PID myPID(Input, Output, Setpoint, 2.5, 0.1, 0.2, DIRECT);

void aSpin() {
  if (digitalRead(aeoa) == HIGH){
    if (digitalRead(aeob) == LOW){
      aCount--;
    }
    else{
      aCount++;
    }
  }
  else{
    if(digitalRead(aeob) == LOW){
      aCount++;
    } 
    else{
      aCount--;
    }
  }
  /*
  if((digitalRead(aeoa) ^ digitalRead(aeob)) == HIGH){
    aCount+=(1*adir);
  }
  */
}

void bSpin() {
  if (digitalRead(beoa) == HIGH){
    if (digitalRead(beob) == LOW){
      bCount++;
    }
    else{
      bCount--;
    }
  }
  else{
    if(digitalRead(beob) == LOW){
      bCount--;
    } 
    else{
      bCount++;
    }
  }
}

void setup() {
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(aeoa, INPUT);
  pinMode(aeob, INPUT);
  pinMode(pwma, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(beoa, INPUT);
  pinMode(beob, INPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(stby, OUTPUT);
  digitalWrite(stby, HIGH);
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  analogWrite(pwma, 100);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  analogWrite(pwmb, 100);
  attachInterrupt(digitalPinToInterrupt(aeoa),aSpin,CHANGE);
  attachInterrupt(digitalPinToInterrupt(beoa),bSpin,CHANGE);
}

void loop() {
//  digitalWrite(bin1, HIGH);
//  digitalWrite(bin2, LOW);
//  analogWrite(pwmb, 100);
//  Serial.printf("%d\t%d\n",aCount,bCount);
//  if(aCount >= 179 && stopped == false){
//    digitalWrite(stby, LOW);
//    stopped = true;
//  }
  aTurns = aCount/180;
  bTurns = bCount/180;
  delay(1000);
  motorTime++;
  aRPS = aTurns/motorTime;
  bRPS = bTurns/motorTime;
}
