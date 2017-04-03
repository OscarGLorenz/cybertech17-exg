#include "Arduino.h"

#include "Pinout.h"

#include "QTRSensors.cpp"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "DynamicStructures/Queue.h"
#include "Gyroscope/Gyroscope.h"
#include "Mapper/Mapper.h"
#include "Global.h"
#include "Debug/Debug.h"


//Iterator maze();

Sharps sharps((unsigned char []) {FRONT_SHARP, LEFT_SHARP, BACK_SHARP, RIGHT_SHARP}, 0.8);

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
  // #define ROTATION_KP 0.25
  // #define ROTATION_KI 4
  // #define ROTATION_KD 0.05
  // #define ROTATION_KF 0.80
  // #define ROTATION_SAT 0.20
  // PID rotation(
  //   []()-> double {return (yaw0-gyro.getAlpha()).get();},
  //   [](double dir){motors.rotate(-constrain(dir,-ROTATION_SAT,ROTATION_SAT));},
  //   0.15);
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


#define THRESHOLD_FRONT 350
#define THRESHOLD_RIGHT 70
#define THRESHOLD_LEFT 70

void turn(double value,bool right) {

  motors.move(0,0);
  delay(500);

  yaw0 = gyro.getAlpha();
  yaw0 = yaw0 + value;
  delay(2);

  if(right) {
    while(abs((yaw0 - gyro.getAlpha()).get()) > 0.11) {
      gyro.check();
      sharps.check();
      motors.rotate(-0.12);
      delay(2);
    }
  } else {
    while(abs((yaw0 - gyro.getAlpha()).get()) > 0.08) {
      gyro.check();
      sharps.check();
      motors.rotate(0.14);
      delay(2);
    }
  }


  // motors.move(0,0.15);
  // delay(500);


  // int c = 0, d = 0;
  // long int timer = millis();
  // while(millis() - timer < 250000 && c < 30) {
  //   //Salida por pantalla de ángulo actual y objetivo
  //   limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 10);
  //
  //   //Lectura giroscopio
  //   gyro.check();
  //
  //   //Ejecutamos PID rotación
  //   rotation.check();
  //
  //   delay(2);
  //
  //   if(abs((yaw0 - gyro.getAlpha()).get()) < 0.05) c++;
  //   if(abs((yaw0 - gyro.getAlpha()).get()) < 0.4) d++;
  //
  //   if(d == 2) {
  //     motors.move(0,0);
  //     delay(500);
  //     ready();
  //     rotation.setKp(0.010);
  //     rotation.setKi(0.005);
  //     rotation.setKd(0.01);
  //   }
  //
  // }
  //
  // rotation.setKp(ROTATION_KP);
  // rotation.setKi(0);
  // rotation.setKd(ROTATION_KD);
  // rotation.resetSumShaft();
  //
  // motors.move(0,0);

  // motors.move(0,0.2);
  // delay(500);
}



void setup() {
  start();

  //Configuración PIDs
  straight.setKp(STRAIGHT_KP);
  straight.setKd(STRAIGHT_KD);

  // rotation.setKp(ROTATION_KP);
  // rotation.setKi(ROTATION_KI);
  // rotation.setKd(ROTATION_KD);
  // rotation.setKf(ROTATION_KF);
  // rotation.setDeadZone(0.10);
  //Inicio giroscopio, esperamos para que se estabilice la salida


  ready();
  Serial.begin(9600);

  gyro.init();

  yaw0 = gyro.getAlpha();

  while(millis() < 4000) {
    gyro.check();
    delay(2);
    yaw0 = gyro.getAlpha();
    sharps.check();
  //  Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));
  }
  yaw0 = gyro.getAlpha();


}

void loop() {

  //Leer giroscopio
  gyro.check();

  //Ejecutar PID Recto
  straight.check();

  sharps.check();

  static long int noRepetir = 0;
if(millis() - noRepetir > 500) {}
  if(sharps.get(Dirs::Right) < THRESHOLD_RIGHT) {
    if(sharps.get(Dirs::Front) < THRESHOLD_RIGHT && sharps.get(Dirs::Left) < THRESHOLD_LEFT) {
      motors.move(0,0.15);
      delay(1000);
      motors.move(0,0);
      while(1) {ready();delay(250);}
    }
      motors.move(0,0.15);
      delay(400);
      turn(M_PI_2,true);
      noRepetir = millis();
      //maze.move(Dir::RIGHT)

  } else if (sharps.get(Dirs::Front) > THRESHOLD_FRONT){
    if(sharps.get(Dirs::Left) < THRESHOLD_LEFT) {
      yaw0 = gyro.getAlpha();
      motors.move(0,-0.15);
      delay(350);
      turn(-M_PI_2,false);
    } else {
      yaw0 = gyro.getAlpha();
      motors.move(0,-0.15);
      delay(350);
      if(sharps.get(Dirs::Left) > sharps.get(Dirs::Right)) {
        turn(M_PI_2,true);
      } else {
        turn(M_PI_2,false);
      }
    }

  }
  delay(2);

}
