/*
 * GyroCTRL.h
 *
 *  Created on: 5 feb. 2017
 *      Author: oscar
 */

#ifndef GYROCTRL_H_
#define GYROCTRL_H_

#include "Arduino.h"
#include "Pinout.h"


#include "lib/PIDController/PID.h"

#include "src/Motors/Motors.h"
#include "src/Gyroscope/Gyroscope.h"

Gyroscope gyro(INTERRUPT_GYRO);

#define DUTY 0.35

Motors motors((char []) {PWM1A, PWM1B}, (char []) {PWM2A, PWM2B}, DUTY);

double yaw0 = 0;

double inputError(void) {
	return yaw0 - gyro.getYaw();
}

void outputError(double error) {
	motors.move(error, 1.0);
}

PID Pid(inputError, outputError);

void setup() {
delay(1000);
	gyro.init();
Serial.begin(9600);
	for (int i = 0; i < 10; i++) {
		gyro.check();
		delay(10);
	}


	while(digitalRead(BUTTON)) {}

	yaw0 = 0;

	for (int i = 0; i < 50; i++) {

		gyro.check();

		motors.move((yaw0 - gyro.getYaw())*4, 1.0);
	}


}
long time = millis();
int l = 0;
void loop() {

	gyro.check();

	//Pid.check();

	motors.move((yaw0 - gyro.getYaw())*4, 1.0);

	if (millis()-time > 1000) {
		switch(l%4) {
		case 0:
			yaw0 = 0;
			break;
		case 1:
			yaw0 = 1.5;
			break;
		case 2:
			yaw0 = 3.1;
			break;
		case 3:
			yaw0 = -1.5;
			break;

		}
		time = millis();
		l++;
	}
	Serial.println(gyro.getYaw());
	delay(10);
}

#endif /* GYROCTRL_H_ */
