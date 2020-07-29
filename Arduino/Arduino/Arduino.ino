/*
 Name:		Arduino.ino
 Created:	7/22/2020 12:25:23 PM
 Author:	Eragon
*/

#include <VescUart.h>
#include <datatypes.h>
#include <crc.h>
#include <buffer.h>
#include <Encoder.h>
#include <kissStepper.h>

#define swStrL A0
#define swStrR A1

#define throPot A2
#define brakePot A3

kissStepper steerStp(5, 4, 6);
Encoder steerEnc (2 , 3);
VescUart vesc;

// global vars for steering position
const int trim = 0;
int currentPos = 0;
const int rackWidth = 2000;
int limLstate = 0;
int limRstate = 0;
/*const int swStrL = 97;
const int swStrR = 96;*/

// global vars for encoder
long int encPosition = 0;
float nrmEncPos = 0;
const int encoderSens = 900;
long int lastMicros = 0;

// global vars for motor
int maxErpm = 9900;
int maxBrakeCurrent = 20;

// the setup function runs once when you press reset or power the board
void setup() {
	pinMode(swStrL, INPUT);
	pinMode(swStrL, INPUT);
	pinMode(13, OUTPUT);

	ADCSRA = (ADCSRA & 0xf8) | 0x04;

	Serial.begin(115200);
	Serial1.begin(115200);

	steerStp.begin();
	steerStp.setAccel(60000);
	calibrateSteering();
	steerEnc.write(0);
	steerStp.setMaxSpeed(18000);

	vesc.setSerialPort(&Serial1);

	Serial.println("setup complete");
}

// the loop function runs over and over again until power down or reset
void loop() {
	useSteering();
	useVesc();
}

// this function finds the center of the steering space
int calibrateSteering() {

	int stpL = 0;
	int stpR = 0;
	
	steerStp.setMaxSpeed(5000);
	steerStp.prepareMove(-20000);

	while (true) {
		limLstate = analogRead(swStrL);
		if (limLstate > 1000) {
			steerStp.stop();
			stpL = steerStp.getPos();
			steerStp.setReverseLimit(stpL + 5);
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
			steerStp.setForwardLimit(stpR - 5);
			break;
		}
		else {
			steerStp.move();
		}
	}

	currentPos = ((stpR - stpL) / 2 + trim);
	steerStp.setPos(currentPos);

	steerStp.prepareMove(0);

	while (true) {
		limLstate = analogRead(swStrL);
		limRstate = analogRead(swStrR);
		if (limRstate > 1000 || limLstate > 1000) {
			steerStp.stop();
			return(1);
		}
		if (steerStp.getDistRemaining() == 0) {
			return(0);
		}
		else {
			steerStp.move();
		}
	}
}

// sets stepper position based on steering wheel position
void useSteering() {

	if ((micros() - lastMicros) >= 20000) {
		lastMicros = micros();
		limLstate = analogRead(swStrL);
		limRstate = analogRead(swStrR);

		encPosition = steerEnc.read();

		if (encPosition > encoderSens) {
			encPosition = encoderSens;
		}
		else if (encPosition < -encoderSens) {
			encPosition = -encoderSens;
		}
		nrmEncPos = (encPosition / (float)encoderSens);

		steerStp.stop();

		if (limRstate > 1000 && steerStp.getDistRemaining() > 0) {
			steerStp.prepareMove(steerStp.getPos());
		}
		else if (limLstate > 1000 && steerStp.getDistRemaining() < 0) {
			steerStp.prepareMove(steerStp.getPos());
		}
		else {
			steerStp.prepareMove(nrmEncPos * rackWidth);
		}
	}

	steerStp.move();
}

// updates the speed and braking current of motor based on pedals
void useVesc() {
	if ((micros() - lastMicros) >= 25000) {
		int throValue = analogRead(throPot);
		int brakeValue = analogRead(brakePot);

		float normThro = (float)throValue / 1024.0;
		float normBrake = (float)brakeValue / 1024.0;

		if (normThro > 1) {
			normThro = 1;
		}
		if (normThro < 0) {
			normThro = 0;
		}
		if (normBrake > 1) {
			normBrake = 1;
		}
		if (normBrake < 0) {
			normBrake = 0;
		}

		float ERPM = maxErpm * normThro;
		float brakeCurrent = maxBrakeCurrent * normBrake;

		if (brakeCurrent > 1) {
			vesc.setRPM(0);
			vesc.setBrakeCurrent(brakeCurrent);
		}
		else {
			vesc.setRPM(ERPM);
		}

	}
}