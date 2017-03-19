
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
  motors.move(-constrain(dir,-1.0,1.0),0.3);
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

#define THRESHOLD_FRONT 150
#define THRESHOLD_RIGHT 200
#define THRESHOLD_LEFT 200

#define CHECKCOUNT 10
#define RAD_TOLERANCE 0.08

#define TIME_BACK 500
#define TIME_STOP 500
#define TIME_FORWARD 500

bool dir = 0;
void loop() {
  //Salida por pantalla de ángulo actual y objetivo
  //limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()) + " " + String((yaw0 - gyro.getAlpha()).get()), 250);
  //Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));

  //Leer giroscopio
  gyro.check();

  //Ejecutar PID Recto
  straight.check();

  if (filterRead(FRONT_SHARP,10,10) > THRESHOLD_FRONT) {
    motors.rotate(0,0);
    delay(500);
    motors.move(0,-0.3);
    delay(500);
    motors.rotate(1, (dir) ? 0.25 : -0.25);
    dir = !dir;
    delay(150);
    motors.move(0,0.3);
    delay(500);
  }

}
