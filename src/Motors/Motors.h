/*
 * Motors.h
 *
 *  Created on: 2 oct. 2016
 *      Author: oscar
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <stdint.h>
#include "PWMOutput.h"

class Motors {
public:

	//Motors(uint8_t pinLeft1, uint8_t pinLeft2, uint8_t pinRight1, uint8_t pinRight2, double maxV);
	Motors(PWMOutput * const outputLeft, PWMOutput * const outputRight, double maxV);

	/* Dir acotado de -1.0 a 1.0, negativo izquierdas, 0.0 recto, 1.0 derecha
	 * Speed acotado de -1 a 1.0, negativo marcha atrás, positivo macrha adelante
	 */
	void fullFwd(double dir, double speed);

	//void fwdBck(double dir,double speed);
	void fwdBck(bool toLeft);

	void halt(void);

	~Motors();

private:
//	uint8_t _pinLeft1;
//	uint8_t _pinLeft2;
//	uint8_t _pinRight1;
//	uint8_t _pinRight2;
	PWMOutput * _outputLeft;
	PWMOutput * _outputRight;
	double _maxV;
};

#endif /* MOTORS_H_ */
