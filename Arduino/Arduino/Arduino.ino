/*
  Name:		Arduino.ino
  Created:	7/22/2020 12:25:23 PM
  Author:	Eragon
*/

#include <Encoder.h>
#include <kissStepper.h>
#include <Servo.h>

#define swStrL A0
#define swStrR A1

#define stpAlmPin A5
#define estopPin A7
#define mtrSwPin A6

#define vescPin 7
#define throPot A9
#define brakePot A8

kissStepper steerStp(5, 4, 6);
Encoder steerEnc (3 , 2);
Servo vesc;

// global vars for steering position
int currentPos = 0;
static int rackWidth = 0;
int limLstate = 0;
int limRstate = 0;

// global vars for encoder
int encPosition = 0;
float nrmEncPos = 0.0;
const int encoderSens = 680; //full range is 940
long int lastMicros = 0;

// global vars for motor
float throRange = 260;
float brakeRange = 280;
long int lastMicros2 = 0;

// global vars for error handling
long int lastMicros3 = 0;
bool stpAlm = 0;
bool estop = 0;
bool motSw = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(swStrL, INPUT);
  pinMode(swStrL, INPUT);
  pinMode(13, OUTPUT);
  pinMode(A12, INPUT);
  pinMode(A13, INPUT);

  ADCSRA = (ADCSRA & 0xf8) | 0x04;

  steerStp.begin();
  calibrateSteering();
  steerStp.setMaxSpeed(11000);
  steerStp.setAccel(0);

  vesc.attach(vescPin);
  Serial.begin(115200);

  steerEnc.write(0);
  startAll();
}

// the loop function runs over and over again until power down or reset
void loop () {
  useSteering((steerEnc.read()/(float)encoderSens));
  useVesc();
  checkErrors();
}

// finds the center of the steering space
int calibrateSteering() {

  int stpL = 0;
  int stpR = 0;

  steerStp.setMaxSpeed(800);
  steerStp.setAccel(3000);
  steerStp.prepareMove(-30000);

  while (true) {
    limLstate = analogRead(swStrL);
    if (limLstate > 1000) {
      steerStp.stop();
      steerStp.setPos(0);
      break;
    }
    else {
      steerStp.move();
    }
  }

  steerStp.prepareMove(30000);

  while (true) {
    limRstate = analogRead(swStrR);
    if (limRstate > 1000) {
      steerStp.stop();
      stpR = steerStp.getPos();
      break;
    }
    else {
      steerStp.move();
    }
  }

  currentPos = (stpR / 2);
  rackWidth = (stpR);

  steerStp.setPos(currentPos);
  steerStp.setMaxSpeed(10000);
  steerStp.prepareMove(0);

  while (true) {
    steerStp.move();
    if (steerStp.getDistRemaining() == 0) {
      break;
    }
  }

  steerStp.setForwardLimit(rackWidth / 2);
  steerStp.setReverseLimit(-rackWidth / 2);
}

// sets stepper position based on steering wheel position @100Hz
void useSteering(float nrmPos) {

  if ((micros() - lastMicros) >= 10000) {
    lastMicros = micros();
    limLstate = analogRead(swStrL);
    limRstate = analogRead(swStrR);

    int absStepDiff = (nrmPos * (rackWidth / 2)) - steerStp.getPos();

    steerStp.stop();

    //if limit switch on, arrest movement in that direction
    if (limRstate > 1000 && absStepDiff > 0) {
      steerStp.prepareMove(steerStp.getPos());
    }
    else if (limLstate > 1000 && absStepDiff < 0) {
      steerStp.prepareMove(steerStp.getPos());
    }
    else {
      steerStp.prepareMove(nrmPos * (rackWidth / 2));
    }
  }

  steerStp.move();
}

// updates the speed and braking current of motor based on pedals
void useVesc() {
  if ((micros() - lastMicros2) >= 20030) {
    lastMicros2 = micros();

    int throValue = analogRead(throPot);
    int brakeValue = analogRead(brakePot);

    float normThro = ((float)throValue - 572) / throRange; // dont touch these values
    float normBrake = ((float)brakeValue - 512) / brakeRange;

    // clamp norm values
    if (normThro > 1) normThro = 1;
    if (normThro < 0) normThro = 0;
    if (normBrake > 1) normBrake = 1;
    if (normBrake < 0) normBrake = 0;

    // calc pwm output
    if (normBrake > 0.15) {
      int microsec = (normBrake * -500) + 1500;
      vesc.writeMicroseconds(microsec);
    }
    else {
      int microsec = (normThro * 500) + 1500;
      vesc.writeMicroseconds(microsec);
    }

  }
}

void stopMotor() {
  vesc.writeMicroseconds(1500);
  vesc.detach();
}

void stopAll() {
  stopMotor();
  steerStp.disable();
}

void startMotor() {
  if (!vesc.attached()) {
    vesc.attach(vescPin);
  }
}

void startAll() {
  startMotor();
  if (!steerStp.isEnabled()) {
    steerStp.enable();
  }
}

int checkErrors() {
  if ((micros() - lastMicros3) >= 40020) {
    lastMicros3 = micros();

    //get switch states
    int stps = analogRead(stpAlmPin);
    int mtrs = analogRead(mtrSwPin);
    int estops = analogRead(estopPin);

    // if stepper alarms: normally high, open collector
    if (stps < 100 && stpAlm == 0) {
      stpAlm = 1;
      stopMotor();
      Serial.println("Stepper Alarmed");
    }
    // if etop depressed: normally low
    if (estops > 1000 && estop == 0) {
      estop = 1;
      stopAll();
      Serial.println("Estop depressed");
    }
    else if (estops < 100 && estop == 1) {
      estop = 0;
      startAll();
      Serial.println("Estop released");
    }

    // motor switch: normally high
    if (mtrs < 100 && motSw == 0) {
      motSw = 1;
      startMotor();
      Serial.println("Motor On");
    }
    else if (mtrs > 1000 && motSw == 1) {
      motSw = 0;
      stopMotor();
      Serial.println("Motor Off");
    }
  }
}
