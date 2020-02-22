#include "Arduino.h"
#include "MotorDriver.h"

MotorDriver::MotorDriver(int ain1, int ain2, int pwma, int bin1, int bin2, int pwmb, int stby) {
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwma, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(stby, OUTPUT);
  AIN1 = ain1;
  AIN2 = ain2;
  PWMA = pwma;
  BIN1 = bin1;
  BIN2 = bin2;
  PWMB = pwmb;
  STBY = stby;
}

void MotorDriver::setMotorFront(char mfront, bool standard) {
  if(mfront == 'a'){
    if(standard){
      AFRONT = '1';
    }
    else{
      AFRONT = '2';
    }
  }
  else{
    if(standard){
      BFRONT = '1'; 
    }
    else{
      BFRONT = '2';
    }
  }
}

void MotorDriver::moveForward(uint8_t fspeeda, uint8_t fspeedb) {
  if(AFRONT == '1'){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else{
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  if(BFRONT == '1'){
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else{
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  analogWrite(PWMA, fspeeda);
  analogWrite(PWMB, fspeedb);
}

void MotorDriver::moveReverse(uint8_t rspeeda, uint8_t rspeedb) {
  if(AFRONT == '1'){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  if(BFRONT == '1'){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else{
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMA, rspeeda);
  analogWrite(PWMB, rspeedb);
}

void MotorDriver::motorBrake(char mbrake, bool mode) {
  if(mbrake == 'a'){
    if(!mode){
      analogWrite(PWMA, LOW);
    }
    else{
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      analogWrite(PWMA, LOW);
    }
  }
  else{
    if(!mode){
      analogWrite(PWMB, LOW);
    }
    else{
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, HIGH);
      analogWrite(PWMB, LOW);
    }
  }
}

void MotorDriver::allBrake(bool allmode) {
  if(!allmode){
    analogWrite(PWMA, LOW);
    analogWrite(PWMB, LOW);
  }
  else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, LOW);
  }
}

void MotorDriver::motorStop(char mstop) {
  if (mstop == 'a') {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, HIGH);
  }
  else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, HIGH);
  }
}

void MotorDriver::allStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, HIGH);
}

void MotorDriver::stbyON() {
  digitalWrite(STBY, HIGH);
}

void MotorDriver::stbyOFF() {
  digitalWrite(STBY, LOW);
}
