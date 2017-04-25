
#include "Arduino.h"
#include "Pinout.h"
#include "Global.h"

#include "Debug/Debug.h"
#include "PIDController/PID.h"

#include "Motors/Motors.h"
#include "Gyroscope/Gyroscope.h"

//Motores
#define DUTY 100
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);
Sharps sharps((unsigned char []) {FRONT_SHARP, LEFT_SHARP, BACK_SHARP, RIGHT_SHARP}, 0.8);

//Giroscopio
Gyroscope gyro(INTERRUPT_GYRO);

//Variable ángulo, se usa como referncia en varias tareas
Angle yaw0(0);

#define STRAIGHT_KP 4
#define STRAIGHT_KI 0.12
#define STRAIGHT_KD 0
#define STRAIGHT_SAT 1
PID straight(
  []()-> double {return (yaw0-gyro.getAlpha()).get();},
  [](double dir){motors.move(constrain(dir,-1.0,1.0),STRAIGHT_SAT);},
  1);//PID STRAIGHT
//PID ROTATION
//Lectura de error de girar
#define ROTATION_KP 0.35
#define ROTATION_KI 0.01
#define ROTATION_KD 0.01
#define ROTATION_KF 0.80
#define ROTATION_SAT 1
#define ROTATION_DEAD 0.11
#define ROTATION_THRESHOLD 0.15
PID rotation(
  []()-> double {return (yaw0-gyro.getAlpha()).get();},
  [](double dir){motors.rotate(-constrain(dir,-ROTATION_SAT,ROTATION_SAT));},
  0.15);
  bool right = true;

  void setup() {

    start();

    //Configuración PIDs
    straight.setKp(STRAIGHT_KP);
    straight.setKi(STRAIGHT_KI);
    straight.setKd(STRAIGHT_KD);

    rotation.setKp(ROTATION_KP);
    rotation.setKi(ROTATION_KI);
    rotation.setKd(ROTATION_KD);
    rotation.setKf(ROTATION_KF);
    rotation.setDeadZone(ROTATION_DEAD);


  //Inicio giroscopio, esperamos para que se estabilice la salida
  gyro.init();


  right = digitalRead(BUTTON);

  while(millis() < 4000) {
    gyro.check();
    sharps.check();
    delay(2);
  }
  ready();
  //Ángulo inicial
  yaw0 = gyro.getAlpha();

  //flag();
}


  void testTurn(double value,bool right) {
  long int tiempos = millis();

    yaw0 = yaw0 + value;
    //Angle yawr = yaw0;
    delay(2);

    int c = 0;
    while(c < 30 ) {
    //  yawr = gyro.getAlpha();
      //Salida por pantalla de ángulo actual y objetivo
      limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 10);

      //Lectura giroscopio
      gyro.check();

      //Ejecutamos PID rotación
      rotation.check();

      if(abs((yaw0 - gyro.getAlpha()).get()) < ROTATION_THRESHOLD) c++;
      if(millis() - tiempos > 2000) break;
      delay(2);

    }

    rotation.resetSumShaft();

  }

void loop() {

  sharps.check();
  //Leer giroscopio
  gyro.check();

  //Ejecutar PID Recto
  straight.check();

  if (sharps.get(Dirs::Front) > 250)  {

    if (right) {
      testTurn(M_PI_4/2.0, true);
    } else {
      testTurn(-M_PI_4/2.0, false);
    }

   long int times = millis();
    while(millis()-times < 600) {
      sharps.check();
      //Leer giroscopio
      gyro.check();

      //Ejecutar PID Recto
      straight.check();
      delay(2);
    }

    if (!right) {
      testTurn(M_PI_4/2.0, true);
    } else {
      testTurn(-M_PI_4/2.0, false);
    }
    right = !right;
  }

  delay(2);

}
