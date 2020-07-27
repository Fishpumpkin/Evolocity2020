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

kissStepper steerStp(5, 4, 6);
Encoder steerEnc (2 , 3);

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
long int encOldPosition = 0;
float nrmEncPos = 0;
const int encoderSens = 900;

// the setup function runs once when you press reset or power the board
void setup() {
	pinMode(swStrL, INPUT);
	pinMode(swStrL, INPUT);
	pinMode(13, OUTPUT);

	ADCSRA = (ADCSRA & 0xf8) | 0x04;

	Serial.begin(115200);
	steerStp.begin();
	steerStp.setAccel(30000);
	calibrateSteering();
	steerEnc.write(0);
	steerStp.setMaxSpeed(12000);
	Serial.println("setup complete");
}

// the loop function runs over and over again until power down or reset
void loop() {
	useSteering();
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

	currentPos = ((stpR - stpL) / 2);
	steerStp.setPos(currentPos);

	steerStp.prepareMove(trim);

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
	// get steering wheel position
	encPosition = steerEnc.read();

	limLstate = analogRead(swStrL);
	limRstate = analogRead(swStrR);

	if (encPosition != encOldPosition) {
		// run when steering wheel pos changes
		if (encPosition > encoderSens) {
			encPosition = encoderSens;
		}
		else if (encPosition < -encoderSens) {
			encPosition = -encoderSens;
		}
		encOldPosition = encPosition;
		nrmEncPos = (encPosition / (float)encoderSens);

		steerStp.stop();
	}

	// if the linear rail hits a contact switch and still has distance to go,
	// cull any movement in that direction
	// else operate normally

	if (limRstate > 1000) {
		if (steerStp.getDistRemaining() > 0) {
			steerStp.prepareMove(steerStp.getPos());
		}
		else {
			steerStp.prepareMove(nrmEncPos * rackWidth);
		}
	}
	else {
		steerStp.prepareMove(nrmEncPos * rackWidth);
	}

	steerStp.move();
}