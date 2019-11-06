#include <PID_v1.h>

/*Motor A Pins*/
#define ain1 2
#define ain2 3
#define pwma 4
#define aeoa 23
#define aeob 22

/*Motor B Pins*/
#define bin1 8
#define bin2 9
#define pwmb 10
#define beoa 14
#define beob 15

/*STBY Pin (aka, the "key" to the motor driver)*/
#define stby 6

/*Circumference of the wheels (based on measured diameter value)*/
#define CIRC 3.14159265359*40.2

/*aCount and bCount measure the number of ticks that the encoder sensors have counted*/
//The number of ticks in 1 rotation are 180
volatile int aCount = 0;
volatile int bCount = 0;

/*For my code, I want motor B to try and match up to motor A's speed, so I will make motor B's speed subject to change*/
//Initially, bSpeed = 100, but this will change as the PID accounts for the difference between motor B's and motor A's speeds
int bSpeed = 100;

/*Here is the PID setup to work with (read the library online for more details)*/
double Input, Output;
double Setpoint = 0.0;
//You can change the constants for Kp, Ki, and Kd based on your results
double Kp = 2.5;
double Ki = 0.1;
double Kd = 0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/*Changes aCount based on the spinning of motor A*/
void aSpin() {
  if (digitalRead(aeoa) == digitalRead(aeob)) {
    aCount++;
  }
  else {
    aCount--;
  }

  //Once motor A makes 1 turn, we'll use the PID
  if (aCount % 180 == 0) {
    Input = bCount - aCount;
    myPID.Compute();

    //The adjustment is the current speed of motor B minus the Output value of the PID controller
    int bAdjust = bSpeed - Output;

    //Remember to constrain your new speed just to make sure that you don't ouput any strange values!
    bSpeed = constrain(bAdjust, 0, 255);
    analogWrite(pwmb, bSpeed);
  }
}

/*Changes bCount based on the spinning of motor B*/
void bSpin() {
  if (digitalRead(beoa) == digitalRead(beob)) {
    bCount--;
  }
  else {
    bCount++;
  }
}

void moveForwardOne() {
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
}

void setup() {
  //Set up pin modes based on whether you're reading or sending values
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

  //Remember to set STBY to high, otherwise your bot won't run
  digitalWrite(stby, HIGH);

  //Here, you're setting the initial direction and speed your motors are gonna go
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  analogWrite(pwma, 100);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  analogWrite(pwmb, bSpeed);

  //The aSpin and bSpin functions will be called each time there is a change in the values of the A channels
  attachInterrupt(digitalPinToInterrupt(aeoa), aSpin, CHANGE);
  attachInterrupt(digitalPinToInterrupt(beoa), bSpin, CHANGE);

  //Turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  Serial.println(Output);
}
