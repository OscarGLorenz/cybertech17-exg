/*
 * PWMOutput.cpp
 *
 *  Created on: 3 oct. 2016
 *      Author: oscar
 */

#include "PWMOutput.h"
#include <stdint.h>
#include "Arduino.h"

DoublePwm::DoublePwm(const uint8_t in1, const uint8_t in2) :
		_in1(in1), _in2(in2) {
	pinMode(_in1, OUTPUT);
	pinMode(_in2, OUTPUT);
}

void DoublePwm::write(const int16_t ddp) const {
	analogWrite(_in1, 127-ddp/2);
	analogWrite(_in2, 127+ddp/2);
}

PwmDir::PwmDir(const uint8_t pin, const uint8_t dir) :
		_pin(pin), _dir(dir) {
	pinMode(_pin, OUTPUT);
	pinMode(_dir, OUTPUT);
}

void PwmDir::write(const int16_t ddp) const {
	analogWrite(_pin, abs(ddp)*2);
	digitalWrite(_dir, ((ddp >= 0) ? HIGH : LOW));
}

PwmDoubleDir::PwmDoubleDir(const uint8_t pin, const uint8_t dir1,
		const uint8_t dir2) :
		_pin(pin), _dir1(dir1), _dir2(dir2) {
	pinMode(_pin, OUTPUT);
	pinMode(_dir1, OUTPUT);
	pinMode(_dir2, OUTPUT);
}

void PwmDoubleDir::write(const int16_t ddp) const {
	analogWrite(_pin, abs(ddp)*2);
	digitalWrite(_dir1, ((ddp >= 0) ? HIGH : LOW));
	digitalWrite(_dir2, ((ddp >= 0) ? LOW : HIGH));
}
