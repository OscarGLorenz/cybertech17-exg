
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
#include "Global.h"

#include "Debug/Debug.h"
#include "PIDController/PID.h"

#include "Motors/Motors.h"
#include "Gyroscope/Gyroscope.h"

//Motores
#define DUTY 255
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);

//Giroscopio
Gyroscope gyro(INTERRUPT_GYRO);

//Variable ángulo, se usa como referncia en varias tareas
Angle yaw0(0);


#define RIGHT true
#define LEFT false

//PID STRAIGHT
//Lectura de error de ir recto
double input() {
  return (yaw0-gyro.getAlpha()).get();
}
//Salida de PID de ir recto
void output(double dir) {
  motors.move(-constrain(dir,-1.0,1.0), 0.20);
}
#define STRAIGHT_KP 1.2
#define STRAIGHT_KI 0
#define STRAIGHT_KD 0.02 //10
PID straight(input,output,1,1);
//PID STRAIGHT




//PID ROTATION
//Lectura de error de girar
double inputR() {
  return (yaw0-gyro.getAlpha()).get();
}
//Salida de PID de girar
void outputR(double dir) {
  motors.rotate(1.0, -constrain(dir,-0.4,0.4));
}
#define ROTATION_KP 0.55
#define ROTATION_KI 0
#define ROTATION_KD 0.025
PID rotation(inputR,outputR,1,1);
//PID ROTATION



#define WAIT_GYROSCOPE 4000

void setup() {
  start();

  //Configuración PIDs
  straight.setKp(STRAIGHT_KP);
  straight.setKd(STRAIGHT_KD);

  rotation.setKp(ROTATION_KP);
  rotation.setKd(ROTATION_KD);

  //Inicio giroscopio, esperamos para que se estabilice la salida
  gyro.init();
  while(millis() < WAIT_GYROSCOPE) {
    gyro.check();
    delay(2);
  }
  ready();
  //Ángulo inicial
  yaw0 = gyro.getAlpha();

  //flag();
}

#define THRESHOLD_FRONT 180
#define THRESHOLD_RIGHT 200
#define THRESHOLD_LEFT 200

#define CHECKCOUNT 10
#define RAD_TOLERANCE 0.08

#define TIME_BACK 500
#define TIME_STOP 500
#define TIME_FORWARD 500

void turn(bool isRight) {

  //Arrancamos
  motors.move(0, 0.20);
  delay(TIME_FORWARD);

  //Paramos
  motors.rotate(0,0);
  delay(TIME_STOP);

  //Sumamos 90º
  if(isRight) {
    yaw0 = yaw0 + M_PI_2;
  } else {
    yaw0 = yaw0 - M_PI_2;
  }

  //Giro, si conseguimos el ángulo deseado y lo leemos CHECKCOUNT veces salimos del while
  unsigned long auxtime = millis();
  unsigned int c = 0;
  while(c < CHECKCOUNT && millis()-auxtime < 1200) {
    //Salida por pantalla de ángulo actual y objetivo
    //limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 250);
    //Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));

    //Lectura giroscopio
    gyro.check();

    //Ejecutamos PID rotación
    rotation.check();

    delay(2);

    //Si el error es menor que RAD_TOLERANCE sumamos
    if(abs((yaw0 - gyro.getAlpha()).get()) < RAD_TOLERANCE) c++;
  }

  //Paramos
  motors.rotate(0,0);
  delay(TIME_STOP);

  //Arrancamos
  motors.move(0, 0.20);
  delay(TIME_FORWARD);

}

void loop() {
  //Salida por pantalla de ángulo actual y objetivo
  //limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()) + " " + String((yaw0 - gyro.getAlpha()).get()), 250);
  //Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));

  //Leer giroscopio
  gyro.check();

  //Ejecutar PID Recto
  straight.check();

  //Si nos encontramos una pared
  if(analogRead(RIGHT_SHARP) < THRESHOLD_RIGHT) {
    turn(RIGHT);
  } else if (analogRead(FRONT_SHARP) > THRESHOLD_FRONT){
    if(analogRead(RIGHT_SHARP) < THRESHOLD_LEFT) {
      turn(LEFT);
    } else {
      turn(LEFT);
      turn(LEFT);
    }
  }

  delay(50);

}

#endif /* GYROCTRL_H_ */
