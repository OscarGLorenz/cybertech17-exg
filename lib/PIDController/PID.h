#ifndef PID_H
#define PID_H

class PID {
public:
	PID(int (*input)(void),void (*output)(int), double maxDerivative = 10E6, double maxIntegral = 10E6);

	void setKp(int kp);

	void setKd(int kd);

	void setKi(int ki);
	int getKp(void);

	int getKd(void);

	int getKi(void);

	void check(void);

private:
	int (*in)(void);
	void (*out)(int);

	double _ki;
	double _kp;
	double _kd;

	double lastError;
	double sumShaft;

	double constrainedKd;
	double constrainedKi;

};

#endif
