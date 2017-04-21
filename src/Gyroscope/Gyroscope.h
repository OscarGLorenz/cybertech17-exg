/*
* Gyroscope.h
*
*  Created on: 9 feb. 2017
*      Author: oscar
*/

#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_

#include "Arduino.h"

#include "../../lib/MPU6050/MPU6050_6Axis_MotionApps20.h"

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
	mpuInterrupt = true;
}


double toDeg(double x) {return x*180.0/M_PI;}
double toRad(double x) {return x/180.0*M_PI;}


class Angle {
	double angle;
public:
	Angle(double value) {
		angle = value;
	}
	double get() const {
		return angle;
	}
	Angle operator-(const Angle& other) const {
		double angulo = angle - other.get();
		if(angulo < -M_PI) {
			return 2.0*M_PI+angulo;
		} else if(angulo > M_PI) {
			return -2.0*M_PI+angulo;
		}

		return angulo;
	}
	Angle operator+(const Angle& other) const {
		if(angle + other.get() > M_PI) {
			return -2*M_PI + angle + other.get();
		} else if (angle + other.get() < -M_PI){
			return 2*M_PI + angle + other.get();
		}
		return angle + other.get();
	}
	Angle& operator=(const Angle& other) {
		angle = other.get();
		return *this;
	}
	Angle& operator=(const double& other) {
		angle = other;
		return *this;
	}
};
//CODIGO PARA PROBAR "ANUGLOS"
// Angle a(0);
// Angle b(0);
//
// int main() {
//
// 	double alpha, beta;
// 	while(1) {
// 	cin >> alpha;
// 	cin >> beta;
// 	a = toRad(alpha);
// 	b = toRad(beta);
//
// 	cout << "A+B" << endl;
// 	cout << toDeg((a+b).get()) << endl;
// 	cout << "A-B" << endl;
// 	cout << toDeg((a-b).get())  << endl;
// 	cout << "B-A" << endl;
// 	cout << toDeg((b-a).get())  << endl;
//
// 	cout << endl << endl << endl << endl;
//
// 	}
//
// }


class Gyroscope : MPU6050{
public:
	Gyroscope(char interrupt_pin) {
		interrupt_gyro = interrupt_pin;
	}

	void init() {
		// join I2C bus (I2Cdev library doesn't do this automatically)
		Wire.begin();
		Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

		// initialize device
		Serial.println(F("Initializing I2C devices..."));
		initialize();
		pinMode(interrupt_gyro, INPUT);

		// verify connection
		Serial.println(F("Testing device connections..."));
		Serial.println(
			testConnection() ?
			F("MPU6050 connection successful") :
			F("MPU6050 connection failed"));

			// load and configure the DMP
			Serial.println(F("Initializing DMP..."));
			devStatus = dmpInitialize();

			// supply your own gyro offsets here, scaled for min sensitivity

			setXAccelOffset(-2121);
			setYAccelOffset(-86);
			setZAccelOffset(1444);

			setXGyroOffset(-195);
			setYGyroOffset(8);
			setZGyroOffset(52);

			// make sure it worked (returns 0 if so)
			if (devStatus == 0) {
				// turn on the DMP, now that it's ready
				Serial.println(F("Enabling DMP..."));
				setDMPEnabled(true);

				// enable Arduino interrupt detection
				Serial.println(
					F(
						"Enabling interrupt detection (Arduino external interrupt 0)..."));
						attachInterrupt(digitalPinToInterrupt(interrupt_gyro), dmpDataReady,
						RISING);
						mpuIntStatus = getIntStatus();

						// set our DMP Ready flag so the main loop() function knows it's okay to use it
						Serial.println(F("DMP ready! Waiting for first interrupt..."));
						dmpReady = true;

						// get expected DMP packet size for later comparison
						packetSize = dmpGetFIFOPacketSize();
					} else {
						// ERROR!
						// 1 = initial memory load failed
						// 2 = DMP configuration updates failed
						// (if it's going to break, usually the code will be 1)
						Serial.print(F("DMP Initialization failed (code "));
						Serial.print(devStatus);
						Serial.println(F(")"));
					}
				}



				void check() {
					// if programming failed, don't try to do anything
					if (!dmpReady)
					return;

					// wait for MPU interrupt or extra packet(s) available
					if (!mpuInterrupt && fifoCount < packetSize)
					return;
					//Salta de la función y permite ejecutar el resto del programa

					// reset interrupt flag and get INT_STATUS byte
					mpuInterrupt = false;
					mpuIntStatus = getIntStatus();

					// get current FIFO count
					fifoCount = getFIFOCount();

					// check for overflow (this should never happen unless our code is too inefficient)
					if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
						// reset so we can continue cleanly
						resetFIFO();
						Serial.println(F("FIFO overflow!"));

						// otherwise, check for DMP data ready interrupt (this should happen frequently)
						// Updates
					} else if (mpuIntStatus & 0x02) {
						// wait for correct available data length, should be a VERY short wait
						while (fifoCount < packetSize)
						fifoCount = getFIFOCount();

						// read a packet from FIFO
						getFIFOBytes(fifoBuffer, packetSize);

						// track FIFO count here in case there is > 1 packet available
						// (this lets us immediately read more without waiting for an interrupt)
						fifoCount -= packetSize;

						dmpGetQuaternion(&q, fifoBuffer);
						dmpGetEuler(euler, &q);
						dmpGetGravity(&gravity, &q);
						dmpGetYawPitchRoll(ypr, &q, &gravity);
						dmpGetAccel(&aa, fifoBuffer);
						dmpGetLinearAccel(&aaReal, &aa, &gravity);
					}
				}

				double getYaw() {
					return ypr[0];
				}

				double getPitch() {
					return ypr[1];
				}

				double getRoll() {
					return ypr[2];
				}

				Angle getAlpha() {
					return Angle(euler[0]);
				}

				Angle getBeta() {
					return Angle(euler[1]);
				}

				Angle getGamma() {
					return Angle(euler[2]);
				}

				Quaternion q;           // [w, x, y, z]         quaternion container
				VectorInt16 aa;         // [x, y, z]            accel sensor measurements
				VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements
				VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
				VectorFloat gravity;    // [x, y, z]            gravity vector
				float euler[3];         // [psi, theta, phi]    Euler angle container
				float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

			private:
				bool dmpReady = false;  // set true if DMP init was successful
				uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
				uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
				uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
				uint16_t fifoCount;     // count of all bytes currently in FIFO
				uint8_t fifoBuffer[64]; // FIFO storage buffer



				char interrupt_gyro;

				uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00,
				'\r', '\n' };

			};
			#endif /* GYROSCOPE_H_ */
