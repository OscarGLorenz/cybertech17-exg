
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


#include "PIDController/PID.h"

#include "Motors/Motors.h"
#include "Gyroscope/Gyroscope.h"

Gyroscope gyro(INTERRUPT_GYRO);


double yaw0 = 0;

void setup() {
Serial.begin(9600);
Serial.println(gyro.getYaw());
gyro.init();

	for (int i = 0; i < 1000; i++) {
		gyro.check();
		delay(10);
		Serial.println(gyro.getYaw());

	}


}
void loop() {

	gyro.check();

	Serial.println(gyro.getYaw());
	delay(10);
}

#endif /* GYROCTRL_H_ */
