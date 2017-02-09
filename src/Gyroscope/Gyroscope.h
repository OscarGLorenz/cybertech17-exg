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
#include "Wire.h"

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

class Gyroscope {
public:
	Gyroscope(char interrupt_pin);

	void init();
	void check();

private:
	MPU6050 mpu;

	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	char interrupt_gyro;

	uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00,
			'\r', '\n' };

};

#endif /* GYROSCOPE_H_ */