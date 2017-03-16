#ifndef PID_H
#define PID_H

class PID {
public:
	PID(double (*input)(void),void (*output)(double), double maxDerivative = 10E6, double maxIntegral = 10E6, double timing = 0.002);

	void setKp(double kp);

	void setKd(double kd);

	void setKi(double ki);
	double getKp(void);

	double getKd(void);

	double getKi(void);

	void check(void);

private:
	double (*in)(void);
	void (*out)(double);

	double _ki;
	double _kp;
	double _kd;

	double lastError;
	double sumShaft;

	double constrainedKd;
	double constrainedKi;

	double dt;
	
	unsigned long lastTime;

};

#endif
