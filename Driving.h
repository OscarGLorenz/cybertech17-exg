/*
 * Driving.h
 *
 *  Created on: 6 feb. 2017
 *      Author: oscar
 */

#ifndef DRIVING_H_
#define DRIVING_H_


#include "Arduino.h"
#include "Pinout.h"

#include "Wire.h"

#include "lib/MPU6050/MPU6050_6Axis_MotionApps20.h"
#include "lib/Motors/Motors.h"



//---------MPU6050 CONFIGS---------\\

MPU6050 mpu;

bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00,
		'\r', '\n' };

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

//---------MPU6050 CONFIGS---------\\




//---------Motors CONFIGS---------\\

#define MAX_V 80
Motors motors(new DoublePwm(PWM1A,PWM1B), new DoublePwm(PWM2A,PWM2B), MAX_V);

//---------Motors CONFIGS---------\\


void setup() {

//---------MPU6050 SETUP---------\\

	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_GYRO, INPUT);

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(
			mpu.testConnection() ?
					F("MPU6050 connection successful") :
					F("MPU6050 connection failed"));

	// wait for ready
	Serial.println(
			F("\nSend any character to begin DMP programming and demo: "));
	while (Serial.available() && Serial.read())
		; // empty buffer
	while (!Serial.available())
		;                 // wait for data
	while (Serial.available() && Serial.read())
		; // empty buffer again

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(
				F(
						"Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_GYRO), dmpDataReady,
				RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

//---------MPU6050 SETUP---------\\


}
long times = millis();

void loop() {
	// if programming failed, don't try to do anything
	if (!dmpReady)
		return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {
		// other program behavior stuff here
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		float dir = Serial.parseFloat()/100;
		float speed = Serial.parseFloat()/100;

		motors.fullFwd(dir, speed);

		if (millis() - times >= 100) {
			Serial.println(ypr[0]);
			times = millis();
		}
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
// Updates
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		Serial.println(ypr[0] * 180 / M_PI);

		// blink LED to indicate activity
		blinkState = !blinkState;
		pinMode(BLUE_LED, OUTPUT);
		digitalWrite(BLUE_LED, blinkState);
	}
}


#endif /* DRIVING_H_ */
