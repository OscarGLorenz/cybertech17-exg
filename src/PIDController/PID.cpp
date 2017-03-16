#include "PID.h"
#include "Arduino.h"
PID::PID(double (*input)(void),void (*output)(double), double maxDerivative, double maxIntegral, double timing) {
	in = input;
	out = output;
	_ki = 0;
	_kp = 0;
	_kd = 0;
	lastError = 0;
	constrainedKd = maxDerivative;
	constrainedKi = maxIntegral;
	sumShaft = 0;
	dt = timing;
}

void PID::setKp(double kp) {
	_kp = kp;
}

void PID::setKd(double kd) {
	_kd = kd;
}

void PID::setKi(double ki) {
	_ki = ki;
}

double PID::getKp(void) {
	return _kp;
}

double PID::getKd(void) {
	return _kd;
}

double PID::getKi(void) {
	return _ki;
}


void PID::check(void) {
	double error = in();

	double integral = 0;
	double derivative = 0;
	double proportional = 0;
	proportional = error*_kp;
	sumShaft += error * dt;
	if (abs(constrainedKi) > abs(sumShaft*_ki)){
		integral = sumShaft*_ki;
	} else {
		integral = ((sumShaft*_ki >= 0) ? 1 : -1 ) * constrainedKi;
	}

	if (abs(constrainedKd) > abs(_kd*(error-lastError))) {
		derivative = _kd*(error-lastError)/dt;
	} else {
		derivative = ((_kd*(error-lastError) >= 0) ? 1 : -1 ) * constrainedKd;
	}

	lastError = error;

	out(integral+proportional+derivative);

	lastTime = millis();
}
