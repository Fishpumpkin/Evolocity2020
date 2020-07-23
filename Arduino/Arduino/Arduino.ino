/*
 Name:		Arduino.ino
 Created:	7/22/2020 12:25:23 PM
 Author:	Eragon
*/

#include <kissStepper.h>

#define swStrL 5
#define swStrR 6

kissStepper steerStp(2, 3, 4);

// global vars for steering position;
int trim = 0;
int currentPos = 0;

// the setup function runs once when you press reset or power the board
void setup() {
	steerStp.begin();
	steerStp.setAccel(100);
}

// the loop function runs over and over again until power down or reset
void loop() {
	
}

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