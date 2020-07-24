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

#define swStrL 7
#define swStrR 8
#define steerPot A0

kissStepper steerStp(4, 5, 6);

Encoder steerEnc (2 , 3);

// global vars for steering position
int trim = 0;
int currentPos = 0;

// global vars for encoder
long int encOldPosition = 0;
float nrmEncPos = 0;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	steerStp.begin();
	steerStp.setAccel(100);
	calibrateSteering();
	steerEnc.write(0);
}

// the loop function runs over and over again until power down or reset
void loop() {
	
}

// this function finds the center of the steering space
int calibrateSteering() {

	int stpL = 0;
	int stpR = 0;

	steerStp.prepareMove(-1500);
	steerStp.setMaxSpeed(500);

	while (true) {
		if (digitalRead(swStrL)) {
			steerStp.stop();
			stpL = steerStp.getPos();
			steerStp.setPos(0);
			break;
		}
		else {
			steerStp.move();
		}
	}

	steerStp.prepareMove(2500);

	while (true) {
		if (digitalRead(swStrR)) {
			steerStp.stop();
			stpR = steerStp.getPos();
			break;
		}
		else {
			steerStp.move();
		}
	}

	currentPos = ((stpR + stpL) / 2);
	steerStp.setPos(currentPos);

	steerStp.prepareMove(0 + trim);

	while (true) {
		if (digitalRead(swStrR) || digitalRead(swStrL)) {
			steerStp.stop();
			break;
		}
		if (steerStp.getDistRemaining() == 0) {
			return(0);
		}
		else {
			steerStp.move();
		}
		return(1);
	}
}

int useSteering() {
	
}

void getSteerWhlPos() {
	long int encPosition = steerEnc.read();
	if (encPosition != encOldPosition) {
		encOldPosition = encPosition;
		nrmEncPos = (encPosition / 300.0);
		Serial.println(nrmEncPos);
	}
}