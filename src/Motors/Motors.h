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
	Motors(char * motorsA, char * motorsB, unsigned char maxSpeed) {
		mA[0] = motorsA[0];
		mA[1] = motorsA[1];
		mB[0] = motorsB[0];
		mB[1] = motorsB[1];
		vMax = maxSpeed;
	}

	//Mover por diferencia de velocidades, dir de -1 a 1 y speed de 0 a 1
	void move(double dir, double speed) {

		int	k1 = vMax * speed * ( (dir <= 0) ? (1.0 + dir) : 1 );
		int	k2 = vMax * speed * ( (dir >= 0) ? (1.0 - dir) : 1 );

		if (speed < 0) {
			analogWrite(mA[0], -k1);
			analogWrite(mA[1], 0);
			analogWrite(mB[0], -k2);
			analogWrite(mB[1], 0);
		} else {
			analogWrite(mA[0], 0);
			analogWrite(mA[1], k1);
			analogWrite(mB[0], 0);
			analogWrite(mB[1], k2);
		}
	}

	//Mover por diferencia de velocidades, dir de -1 a 1 y speed de 0 a 1
	//Se permite sentido inverso
	void rotate(double speed) {
		int	k1 =  speed * vMax;
		int	k2 = -k1;

		analogWrite(mA[0], (k1 <= 0) ? 0 : abs(k1));
		analogWrite(mA[1], (k1 >= 0) ? 0 : abs(k1));
		analogWrite(mB[0], (k2 <= 0) ? 0 : abs(k2));
		analogWrite(mB[1], (k2 >= 0) ? 0 : abs(k2));
	}

		void smoothRotate(double speed, bool right, double coef) {
		  int k1 = vMax * speed * (right ? 1+coef : 1);
		  int k2 = vMax * speed * (!right ? 1+coef : 1);

			if (speed < 0) {
				analogWrite(mA[0], -k1);
				analogWrite(mA[1], 0);
				analogWrite(mB[0], -k2);
				analogWrite(mB[1], 0);
			} else {
				analogWrite(mA[0], 0);
				analogWrite(mA[1], k1);
				analogWrite(mB[0], 0);
				analogWrite(mB[1], k2);
			}
		}

	void setMax(unsigned char max) {
			vMax = max;
	}

private:
	char mA[2];
	char mB[2];
	unsigned char vMax;
};

#endif /* MOTORS_H_ */
