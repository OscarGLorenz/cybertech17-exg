#include "PID.h"
#include "Arduino.h"
PID::PID(double (*input)(void),void (*output)(double), double maxIntegral, double timing) {
	in = input;
	out = output;
	_ki = 0;
	_kp = 0;
	_kd = 0;
	_kf = 1;
	lastError = 0;
	constrainedKi = maxIntegral;
	sumShaft = 0;
	dt = timing;
	deadZone = 0;
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

void PID::setKf(double kf) {
	_kf = kf;
}

void PID::setDeadZone(double value) {
	deadZone = value;
}

void PID::resetSumShaft() {
	sumShaft = 0;
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

double PID::getKf(void) {
	return _kf;
}

void PID::check(void) {
	double error = in();

	double integral = 0;
	double derivative = 0;
	double proportional = 0;

	proportional = error*_kp;

	sumShaft += error * dt;

	if (abs(constrainedKi) > abs(sumShaft*_ki)){
		integral = ((sumShaft*_ki >= 0) ? 1 : -1 ) * deadZone + sumShaft*_ki;
	} else {
		sumShaft = -sumShaft;
	}

	error = lastError + _kf * (error - lastError);
  derivative = _kd*(error-lastError)/dt;
	lastError = error;

	out(integral+proportional+derivative);

}
