/*
 * Motors.cpp
 *
 *  Created on: 2 oct. 2016
 *      Author: oscar
 */

#include "Motors.h"
#include "Arduino.h"

//Motors::Motors(uint8_t pinLeft1, uint8_t pinLeft2, uint8_t pinRight1, uint8_t pinRight2, double maxV) :
//	_pinLeft1(pinLeft1), _pinLeft2(pinLeft2), _pinRight1(pinRight1),_pinRight2(pinRight2),_maxV(maxV) {
//	pinMode(_pinLeft1,OUTPUT);
//	pinMode(_pinLeft2,OUTPUT);
//	pinMode(_pinRight1,OUTPUT);
//	pinMode(_pinRight2,OUTPUT);
//}

Motors::Motors(PWMOutput * const outputLeft, PWMOutput * const outputRight,
		double maxV) :
		_outputLeft(outputLeft), _outputRight(outputRight), _maxV(maxV) {
}

//void Motors::fullFwd(double dir, double speed) {
//
//	double ddpl = 255.0*fabs(speed)*(double) (_maxV)* ((dir <= 0) ? 1.0+dir : 1.0);
//	double ddpr = 255.0*fabs(speed)*(double) (_maxV)* ((dir >= 0) ? 1.0-dir : 1.0);
//
//	analogWrite(_pinLeft1,127+ ((speed > 0.0) ? ddpl/2 : 0-ddpl/2));
//	analogWrite(_pinLeft2,127+ ((speed < 0.0) ? ddpl/2 : 0-ddpl/2));
//
//	analogWrite(_pinRight1,127+ ((speed > 0.0) ? ddpr/2 : 0-ddpr/2));
//	analogWrite(_pinRight2,127+ ((speed < 0.0) ? ddpr/2 : 0-ddpr/2));
//}
void Motors::fullFwd(double dir, double speed) {

	double ddpl = 255.0 * speed * (double) (_maxV)
			* ((dir <= 0.0) ? 1.0 + dir : 1.0);
	double ddpr = 255.0 * speed * (double) (_maxV)
			* ((dir >= 0.0) ? 1.0 - dir : 1.0);

	_outputLeft->write(ddpl);
	_outputRight->write(ddpr);
}

//void Motors::fwdBck(double dir, double speed) {
//
//	double ddpl = 255.0*fabs(speed)*(double) (_maxV)*((dir <= 0) ? dir : dir);
//	double ddpr = 255.0*fabs(speed)*(double) (_maxV)*((dir >= 0) ? -dir : -dir);
//
//
//	analogWrite(_pinLeft1,127+ ((speed > 0.0) ? ddpl/2 : 0.0-ddpl/2));
//	analogWrite(_pinLeft2,127+ ((speed < 0.0) ? ddpl/2 : 0.0-ddpl/2));
//
//	analogWrite(_pinRight1,127+ ((speed > 0.0) ? ddpr/2 : 0.0-ddpr/2));
//	analogWrite(_pinRight2,127+ ((speed < 0.0) ? ddpr/2 : 0.0-ddpr/2));
//}

void Motors::fwdBck(bool toLeft) {

	_outputLeft->write((toLeft) ? 255 : -255);
	_outputRight->write((toLeft) ? -255 : 255);
}

//void Motors::halt(void) {
//
//	analogWrite(_pinLeft1,127);
//	analogWrite(_pinLeft2,127);
//
//	analogWrite(_pinRight1,127);
//	analogWrite(_pinRight2,127);
//}

void Motors::halt(void) {
	_outputLeft->write(0);
	_outputRight->write(0);
}

Motors::~Motors() {
	delete _outputLeft;
	delete _outputRight;
}

