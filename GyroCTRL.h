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

#define MAX_V 80
Motors motors(new DoublePwm(PWM1A, PWM1B), new DoublePwm(PWM2A, PWM2B), MAX_V);

double yaw0 = 0;

double inputError(void) {
	return yaw0 - gyro.getYaw();
}

void outputError(double error) {
	motors.fullFwd(error, 1.0);
}

PID Pid(inputError, outputError);

void setup() {

	gyro.init();

	for (int i = 0; i < 10; i++) {
		gyro.check();
		delay(10);
	}

	yaw0 = gyro.getYaw();

	Pid.setKp(0.01);

}

void loop() {

	gyro.check();

	Pid.check();

}

#endif /* GYROCTRL_H_ */
