/*
 Name:		Arduino.ino
 Created:	7/22/2020 12:25:23 PM
 Author:	Eragon
*/

#include <kissStepper.h>
#include <chrono>

#define swStrL 5
#define swStrR 6

kissStepper steerStp(2, 3, 4);

// the setup function runs once when you press reset or power the board
void setup() {
	steerStp.begin();
	steerStp.setAccel(100);
}

// the loop function runs over and over again until power down or reset
void loop() {
	
}

void calibrateSteering() {
	int currentStp = 0;
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
}