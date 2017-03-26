#ifndef PID_H
#define PID_H

class PID {
public:
	PID(double (*input)(void),void (*output)(double), double maxIntegral = 10E6, double timing = 0.002);

	void setKp(double kp);

	void setKd(double kd);

	void setKi(double ki);

	void setKf(double kf);

	double getKp(void);

	double getKd(void);

	double getKi(void);

	double getKf(void);

	void check(void);

private:
	double (*in)(void);
	void (*out)(double);

	double _ki;
	double _kp;
	double _kd;

	double lastError;
	double sumShaft;

	double _kf;
	double constrainedKi;

	double dt;

};

#endif
