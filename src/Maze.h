
/*
* GyroCTRL.h
*
*  Created on: 5 feb. 2017
*      Author: oscar
*/

#ifndef GYROCTRL_H_
#define GYROCTRL_H_

#include "Arduino.h"
#include "Pinout.h"


#include "PIDController/PID.h"

#include "Motors/Motors.h"
#include "Gyroscope/Gyroscope.h"

#define toDeg(x) x*180.0/M_PI
#define toRad(x) x/180.0*M_PI

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"



Gyroscope gyro(INTERRUPT_GYRO);

void limitedSerial(String value, unsigned long limit) {
  static unsigned long timing = millis();
  if (millis()- timing > limit) {
    timing = millis();
    Serial.println(value);
  }
}

#define DUTY 25
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);

void setup() {
  Serial.begin(9600);
  gyro.init();

while(millis() < 5000) {
  gyro.check();
delay(2);
}

//Calibrar Gyro
  // while(1) {
  //   gyro.check();
  //   String str = String(String(gyro.getAlpha()) + " " + String(gyro.getBeta()) \
  //   + " " + String(gyro.getGamma()) + " " + String(gyro.getAcc().z));
  //   limitedSerial(str, 250);
  // }

}
void loop() {
  motors.move(0, 1.0);
  gyro.check();

  if(analogRead(FRONT_SHARP) > 200) {
    gyro.check();
    motors.rotate(0,0);
    delay(500);

    motors.rotate(0, -1.0);
    delay(1000);
    motors.rotate(1, 1.0);

    double gyros = gyro.getAlpha();
    while(abs(gyros-gyro.getAlpha()) < M_PI/2.0) {
      gyro.check();
      limitedSerial(String(toDeg(abs(gyros-gyro.getAlpha()))), 250);
    }
    motors.rotate(0,0);
    delay(500);
    motors.move(0, 1.0);

    delay(3000);

  }

}

#endif /* GYROCTRL_H_ */
