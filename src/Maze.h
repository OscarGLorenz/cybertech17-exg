#include "Arduino.h"

#include "Pinout.h"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "DynamicStructures/Queue.h"
#include "Gyroscope/Gyroscope.h"

#include "Global.h"

#include "QTRSensors.cpp"

#include "Debug/Debug.h"

Sharps sharps((unsigned char []) {FRONT_SHARP, LEFT_SHARP, BACK_SHARP, RIGHT_SHARP}, 0.2);

//Giroscopio
Gyroscope gyro(INTERRUPT_GYRO);



//Variable ángulo, se usa como referncia en varias tareas
Angle yaw0(0);



//Motores
#define DUTY 200
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);



//Sensor de línea
QTRSensorsRC qtrrc((unsigned char[]) {PIN1_QTR,PIN2_QTR, PIN3_QTR, PIN4_QTR,
  PIN5_QTR, PIN6_QTR, PIN7_QTR,PIN8_QTR}, 8, 2500);
  unsigned int sensorValues[8];
  int position = 3500;

  //PID ROTATION
  //Lectura de error de girar
  #define ROTATION_KP 0.25
  #define ROTATION_KI 0
  #define ROTATION_KD 0
  #define ROTATION_KF 0.80
  #define ROTATION_SAT 0.20
  PID rotation(
    []()-> double {return (yaw0-gyro.getAlpha()).get();},
    [](double dir){motors.rotate(-constrain(dir,-ROTATION_SAT,ROTATION_SAT));},
    1);
  //PID ROTATION

//PID STRAIGHT
#define STRAIGHT_KP 1.2
#define STRAIGHT_KI 0
#define STRAIGHT_KD 0.02
#define STRAIGHT_SAT 0.20
PID straight(
  []()-> double {return (yaw0-gyro.getAlpha()).get();},
  [](double dir){motors.move(constrain(dir,-1.0,1.0), STRAIGHT_SAT);},
  1);
//PID STRAIGHT


#define THRESHOLD_FRONT 180
#define THRESHOLD_RIGHT 200
#define THRESHOLD_LEFT 200

void turn(double value) {
  yaw0 = gyro.getAlpha();
  yaw0 = yaw0 + value;

  //Giro, si conseguimos el ángulo deseado y lo leemos CHECKCOUNT veces salimos del while
  int c = 0;
  while(c < 40) {
    //Salida por pantalla de ángulo actual y objetivo
    limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 10);
    //Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));

    //Lectura giroscopio
    gyro.check();

    //Ejecutamos PID rotación
    rotation.check();

    delay(2);

    //Si el error es menor que RAD_TOLERANCE sumamos
    if(abs((yaw0 - gyro.getAlpha()).get()) < 0.04) c++;
  }

  motors.move(0,0);
}



void setup() {
  start();

  //Configuración PIDs
  straight.setKp(STRAIGHT_KP);
  straight.setKd(STRAIGHT_KD);

  rotation.setKp(ROTATION_KP);
  rotation.setKi(ROTATION_KI);
  rotation.setKd(ROTATION_KD);
  rotation.setKf(ROTATION_KF);

  //Inicio giroscopio, esperamos para que se estabilice la salida


  ready();

  gyro.init();

  yaw0 = gyro.getAlpha();

  while(millis() < 4000) {
    gyro.check();
    delay(2);
    yaw0 = gyro.getAlpha();

    Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));
  }
  yaw0 = gyro.getAlpha();

  for(int i=0; i < 4; i++) {
    turn(M_PI_2);
    delay(1000);}
  for(int i=0; i < 4; i++) {
    turn(-M_PI_2);
    delay(1000);}
  ready();
  exit(0);
}

void loop() {

  // //Leer giroscopio
  // gyro.check();
  //
  // //Ejecutar PID Recto
  // straight.check();
  //
  // sharps.check();
  //
  // if(sharps.get(Dirs::Right) < THRESHOLD_RIGHT) {
  //   turn(M_PI_2);
  // } else if (sharps.get(Dirs::Front) > THRESHOLD_FRONT){
  //   if(sharps.get(Dirs::Left) < THRESHOLD_LEFT) {
  //     turn(-M_PI_2);
  //   } else {
  //     turn(M_PI);
  //   }
  // }
  //
  // delay(2);

}
