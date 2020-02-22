#ifndef MotorDriver_h
#define MotorDriver_h

#include "Arduino.h"

class MotorDriver
{
  public:
    MotorDriver(int ain1, int ain2, int pwma, int bin1, int bin2, int pwmb, int stby);
    void setMotorFront(char mfront, bool standard);
    void moveForward(uint8_t fspeeda, uint8_t fspeedb);
    void moveReverse(uint8_t rspeeda, uint8_t rspeedb);
    void pointTurn(char dir, char mright, char mleft);
    void motorBrake(char mbrake, bool mode);
    void allBrake(bool allmode);
    void motorStop(char mstop);
    void allStop();
    void stbyON();
    void stbyOFF();
  private:
    int AIN1;
    int AIN2;
    int PWMA;
    int BIN1;
    int BIN2;
    int PWMB;
    int STBY;
    char AFRONT;
    char BFRONT;
};

#endif
