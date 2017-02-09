/*
 * PWMOutput.h
 *
 *  Created on: 3 oct. 2016
 *      Author: oscar
 */

#ifndef PWMOUTPUT_H_
#define PWMOUTPUT_H_

#include <stdint.h>
#include "Arduino.h"

class PWMOutput {
public:
	virtual void write(const int16_t ddp) const = 0;
	virtual ~PWMOutput() {}
};

class DoublePwm : public PWMOutput {
public:
	DoublePwm(const uint8_t in1, const uint8_t in2);
	//Override
	void write(const int16_t ddp) const;
private:
	uint8_t _in1;
	uint8_t _in2;
};

class PwmDir : public PWMOutput {
public:
	PwmDir(const uint8_t pin, const uint8_t dir);
	//Override
	void write(const int16_t ddp) const;
private:
	uint8_t _pin;
	uint8_t _dir;
};

class PwmDoubleDir : public PWMOutput {
public:
	PwmDoubleDir(const uint8_t pin, const uint8_t dir1, const uint8_t dir2);
	//@Override
	void write(const int16_t ddp) const;
private:
	uint8_t _pin;
	uint8_t _dir1;
	uint8_t _dir2;
};


#endif /* PWMOUTPUT_H_ */
