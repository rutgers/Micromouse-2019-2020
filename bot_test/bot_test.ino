#include <MotorDriver.h>
#include <Encoder.h>
#include <FastPID.h>
#include <SharpDistSensor.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Wire.h>

#define pwma 10
#define ain2 9
#define ain1 8
#define stby 7
#define bin1 6
#define bin2 5
#define pwmb 4

MotorDriver motors(ain1, ain2, pwma, bin1, bin2, pwmb, stby);

char botDirection = 'N';

Encoder encA(11, 12);
Encoder encB(2, 3);

float Kp = 5, Ki = 20, Kd = 1, Hz = 1000;

FastPID linePID(Kp, Ki, Kd, Hz, 8, false);
FastPID turnRightPID(Kp, Ki, Kd, Hz, 8, false);
FastPID turnLeftPID(Kp, Ki, Kd, Hz, 8, false);

// Define the number of sensors in the array as a constant
const byte nSensors = 3;

// Window size of the median filter (odd number, 1 = no filtering)
const byte medianFilterWindowSize = 5;

//// Define the array of SharpDistSensor objects
//SharpDistSensor irArray[] = {
//  SharpDistSensor(A0, medianFilterWindowSize),
//  SharpDistSensor(A1, medianFilterWindowSize),
//  SharpDistSensor(A2, medianFilterWindowSize),
//};

uint16_t distArray[nSensors];

MPU6050 mpu;

#define INTERRUPT_PIN 23

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  // Serial.begin(115200);

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  // verify connection
  if (mpu.testConnection()) digitalWrite(LED_BUILTIN, HIGH);

  // wait for ready
  delay(2000);

  digitalWrite(LED_BUILTIN, LOW);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-26);
  mpu.setYGyroOffset(22);
  mpu.setZGyroOffset(-1);
  mpu.setXAccelOffset(-1110);
  mpu.setYAccelOffset(1732);
  mpu.setZAccelOffset(1011);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  // Set some parameters for each sensor in array
  //  for (byte i = 0; i < nSensors; i++) {
  //    irArray[i].setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);  // Set sensor model
  //    // Set other parameters as required
  //  }

  motors.motorOrderDefault(true);
  motors.stbyON();
  delay(2000);
}

int countdown = 4;

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  if (countdown == 0) return;
  moveForward();
  turnRight();
  countdown--;
}

void moveForward() {
  encA.write(0);
  encB.write(0);
  motors.setForward();
  int a = 0;
  int b = 0;
  analogWrite(pwma, 50);
  analogWrite(pwmb, 50);
  while (encA.read() < 600) {
    a = encA.read();
    b = encB.read();
    analogWrite(pwmb, linePID.step(0, b - a));
    delay(1);
  }
  motors.allStop();
  encA.write(0);
  encB.write(0);
  delay(500);
}

void turnRight() {
  encA.write(0);
  encB.write(0);
  float yawPos = 0;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yawPos = ypr[0] * 180 / M_PI;
  }
  motors.setRight();
  int a = 0;
  int b = 0;
  float angleCheck = 0;
  analogWrite(pwma, 40);
  analogWrite(pwmb, 40);
  switch (botDirection) {
    case 'N':
      botDirection = 'E';
      angleCheck = 88;
      break;
    case 'E':
      botDirection = 'S';
      angleCheck = 175;
      break;
    case 'S':
      botDirection = 'W';
      angleCheck = -94;
      yawPos = -180;
      break;
    case 'W':
      botDirection = 'N';
      angleCheck = -1;
      break;
    default:
      break;
  }
  while (yawPos < angleCheck) {
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yawPos = ypr[0] * 180 / M_PI;
    }
    a = encA.read();
    b = encB.read();
    analogWrite(pwmb, turnRightPID.step(-a, b));
    delay(1);
  }
  motors.allStop();
  encA.write(0);
  encB.write(0);
  delay(500);
}

void turnLeft() {
  encA.write(0);
  encB.write(0);
  float yawPos = 0;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yawPos = ypr[0] * 180 / M_PI;
  }
  motors.setLeft();
  int a = 0;
  int b = 0;
  float angleCheck = 0;
  analogWrite(pwma, 40);
  analogWrite(pwmb, 40);
  switch (botDirection) {
    case 'N':
      botDirection = 'W';
      angleCheck = -88;
      break;
    case 'W':
      botDirection = 'S';
      angleCheck = -175;
      break;
    case 'S':
      botDirection = 'E';
      angleCheck = 94;
      yawPos = -180;
      break;
    case 'E':
      botDirection = 'N';
      angleCheck = 1;
      break;
    default:
      break;
  }
  while (yawPos > angleCheck) {
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yawPos = ypr[0] * 180 / M_PI;
    }
    a = encA.read();
    b = encB.read();
    analogWrite(pwmb, turnLeftPID.step(-a, b));
    delay(1);
  }
  motors.allStop();
  encA.write(0);
  encB.write(0);
  delay(500);
}
