
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

#include "Debug/Debug.h"
#include "PIDController/PID.h"

#include "Motors/Motors.h"
#include "Gyroscope/Gyroscope.h"

//Motores
#define DUTY 40
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);

//Giroscopio
Gyroscope gyro(INTERRUPT_GYRO);

//Variable ángulo, se usa como referncia en varias tareas
Angle yaw0(0);



//PID STRAIGHT
//Lectura de error de ir recto
double input() {
  return (yaw0-gyro.getAlpha()).get();
}
//Salida de PID de ir recto
void output(double dir) {
  motors.move(-constrain(dir,-1.0,1.0), 1.0);
}
#define STRAIGHT_KP 1.2
#define STRAIGHT_KI 0
#define STRAIGHT_KD 10
PID straight(input,output,1,1);
//PID STRAIGHT




//PID ROTATION
//Lectura de error de girar
double inputR() {
  return (yaw0-gyro.getAlpha()).get();
}
//Salida de PID de girar
void outputR(double dir) {
  motors.rotate(1.0, -constrain(dir,-1.0,1.0));
}
#define ROTATION_KP 1.2
#define ROTATION_KI 0.01
#define ROTATION_KD 20
PID rotation(inputR,outputR,1,1);
//PID ROTATION



#define LOW_VOTAGE 750
#define WAIT_GYROSCOPE 4000

void setup() {
  //LECTURA DEL VOTAJE DE LA LIPO
   int LiPo = analogRead(A1);
   //Valores medidos 7.5V 900 y 7.8V 940
   //Valor límite 6.25 750
   if (LiPo <= LOW_VOTAGE) {
     pinMode(BLUE_LED, OUTPUT);
     for (int i = 0; i < 50; i++) {
       digitalWrite(BLUE_LED,HIGH);
       delay(50);
       digitalWrite(BLUE_LED,LOW);
       delay(50);
     }
   }
   //LECTURA DEL VOTAJE DE LA LIPO

   delay(50);
   Serial.begin(9600);

   //MENSAJE INICIAL
   if (LiPo <= LOW_VOTAGE) {
     Serial.println("STATUS: LOW VOLTAGE");
   } else {
     Serial.println("STATUS: READY");
   }
   Serial.print("VOLTAGE: ");
   Serial.print(LiPo/900.0*7.5);
   Serial.println("V");
   //MENSAJE INICIAL

  //Configuración PIDs
  straight.setKp(STRAIGHT_KP);
  straight.setKd(STRAIGHT_KD);

  rotation.setKp(ROTATION_KP);
  rotation.setKi(0.01);
  rotation.setKd(ROTATION_KD);

  //Inicio giroscopio, esperamos para que se estabilice la salida
  gyro.init();
  while(millis() < WAIT_GYROSCOPE) {
    gyro.check();
    delay(2);
  }

  //Ángulo inicial
  yaw0 = gyro.getAlpha();
}

#define THRESHOLD_FRONT 180

#define CHECKCOUNT 30
#define RAD_TOLERANCE 0.06

#define TIME_BACK 500
#define TIME_STOP 500
#define TIME_FORWARD 500

void loop() {

  //Salida por pantalla de ángulo actual y objetivo
  limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()) + " " + String((yaw0 - gyro.getAlpha()).get()), 250);

  //Leer giroscopio
  gyro.check();

  //Ejecutar PID Recto
  straight.check();

  delay(2);


  //Si nos encontramos una pared
  if(analogRead(FRONT_SHARP) > THRESHOLD_FRONT) {

    //Paramos
    motors.rotate(0,0);
    delay(TIME_STOP);

    //Un poco marcha atrás
    motors.move(0, -1.0);
    delay(TIME_BACK);

    //Sumamos 90º
    yaw0 = yaw0 + M_PI_2;

    //Giro, si conseguimos el ángulo deseado y lo leemos CHECKCOUNT veces salimos del while
    int c = 0;
    while(c < CHECKCOUNT) {
      //Salida por pantalla de ángulo actual y objetivo
      limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 250);

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
    motors.move(0, 1.0);
    delay(TIME_FORWARD);
  }

  delay(2);

}

#endif /* GYROCTRL_H_ */
