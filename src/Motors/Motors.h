/*
 * Motors.h
 *
 *  Created on: 2 oct. 2016
 *      Author: oscar
 */

#ifndef MOTORS_H_
#define MOTORS_H_
#include "Arduino.h"

class Motors {
public:

	//Dos vectores de dos componentes para los pines de cada motor, maxima velocidad en tanto por uno
	Motors(char * motorsA, char * motorsB, float maxSpeed) {
		mA[0] = motorsA[0];
		mA[1] = motorsA[1];
		mB[0] = motorsB[0];
		mB[1] = motorsB[1];
		vMax = constrain(255.0 * maxSpeed, 0, 255);
	}

	//Mover por diferencia de velocidades, dir de -1 a 1 y speed de 0 a 1
	void move(double dir, double speed) {
		dir = constrain(dir, -1.0, 1.0);
		speed = constrain(speed, 0.0, 1.0);
		int	k1 = vMax * speed * ( (dir <= 0) ? (1.0 - dir) : 1 );
		int	k2 = vMax * speed * ( (dir >= 0) ? (1.0 - dir) : 1 );
		analogWrite(mA[0], 0);
		analogWrite(mA[1], k1);
		analogWrite(mB[0], 0);
		analogWrite(mB[1], k2);
	}

private:
	char mA[2];
	char mB[2];
	char vMax;
};

#endif /* MOTORS_H_ */
