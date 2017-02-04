#include "PID.h"

PID::PID(double (*input)(void),void (*output)(double), double maxDerivative, double maxIntegra) {
	in = input;
	out = output;
	_ki = 0;
	_kp = 0;
	_kd = 0;
	lastError = 0;
	constrainedKd = maxDerivative;
	constrainedKi = maxIntegral;
	sumShaft = 0;
}

void PID::setKp(int kp) {
	_kp = kp;


void PID::setKd(int kd) {
	_kd = kd;
}

void PID::setKi(int ki) {
	_ki = ki;
}

int PID::getKp(void) {
	return _kp;
}

int PID::getKd(void) {
	return _kd;
}

int PID::getKi(void) {
	return _ki;
}

void PID::check(void) {
	double error = in();

	double integral = 0;
	double derivative = 0;
	double proportional = 0;
	proportional = error*_kp;
	sumShaft += error;
	if (constrainedKi > sumShaft*_ki/proportional){
		integral = sumShaft*_ki;
	} else {
		integral = proportional*constrainedKi;
	}



	if (constrainedKd > _kd*(error-lastError)/proportional) {
		derivative = _kd*(error-lastError);
	} else {
		derivative = proportional*constrainedKd;
	}

	lastError = error;



	out(integral+proportional+derivative);

}

