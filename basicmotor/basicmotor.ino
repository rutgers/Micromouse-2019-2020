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
const double CIRC = 3.14159265359*40.2;

/*aCount and bCount measure the number of ticks that the encoder sensors have counted*/
//The number of ticks in 1 rotation are 360
volatile int aCount = 0;
volatile int bCount = 0;

/*For my code, I want motor B to try and match up to motor A's speed, so I will make motor B's speed subject to change*/
//Initially, bSpeed = 40, but this will change as the PID accounts for the difference between motor B's and motor A's speeds
int bSpeed = 40;

/*Here is the PID setup to work with (read the library online for more details)*/
double Input = 0.0, Output = 0.0, Setpoint = 0.0;
/*You can change the constants for Kp, Ki, and Kd based on your testing results*/
double Kp = 6, Ki = 12, Kd = 0.2;
/*Create the PID controller*/
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/*
  Changes bCount based on the spinning of motor B
  Remember to read both channels for full tick count!
*/
void aASpin() {
  if (digitalRead(aeoa) == digitalRead(aeob)) {
    aCount--;
  }
  else {
    aCount++;
  }
}

void aBSpin() {
  if (digitalRead(aeob) == digitalRead(aeoa)) {
    aCount++;
  }
  else {
    aCount--;
  }
}

/*
  Changes bCount based on the spinning of motor B
  Remember to read both channels for full tick count!
*/
void bASpin() {
  if (digitalRead(beoa) == digitalRead(beob)) {
    bCount--;
  }
  else {
    bCount++;
  }
}

void bBSpin() {
  if (digitalRead(beob) == digitalRead(beoa)) {
    bCount++;
  }
  else {
    bCount--;
  }
}

void moveForwardOne() {
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
}

#define blueserial Serial1

void setup() {
  blueserial.begin(38400);
  
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
  digitalWrite(stby, LOW);

  //Here, you're setting the initial direction and speed your motors are gonna go
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  analogWrite(pwma, 40);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  analogWrite(pwmb, bSpeed);

  //The aSpin and bSpin functions will be called each time there is a change in the values of the encoders' A channels
  attachInterrupt(digitalPinToInterrupt(aeoa), aASpin, CHANGE);
  attachInterrupt(digitalPinToInterrupt(aeob), aBSpin, CHANGE);
  attachInterrupt(digitalPinToInterrupt(beoa), bASpin, CHANGE);
  attachInterrupt(digitalPinToInterrupt(beob), bBSpin, CHANGE);

  //Turn the PID on by having it's mode set to AUTOMATIC
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
}

void loop() {
  if(millis() < 2000){
    digitalWrite(stby, LOW);
    return;
  }
  else{
    digitalWrite(stby, HIGH);
  }
  
  //Take the difference in motor count as the input and run the PID function
  Input = bCount - aCount;
  
  myPID.Compute();
  
  //The output is the degree of change needed to match up your input with the setpoint
  //This makes it so that both your motors are at the same tick count
  bSpeed = Output;
  analogWrite(pwmb, bSpeed);
  
  //To check it's working, keep track of the input and setpoint as your bot runs
  //The input should go towards the setpoint and hover around it, indicating that the difference in your motors' tick counts are around 0
  Serial.println(Input);
  delay(10);
}
