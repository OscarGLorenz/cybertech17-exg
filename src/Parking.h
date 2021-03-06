#include "Arduino.h"

#include "Pinout.h"

#include "QTRSensors.cpp"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "DynamicStructures/Queue.h"
#include "Gyroscope/Gyroscope.h"
#include "Global.h"
#include "Debug/Debug.h"
//8-00V
Sharps sharps((unsigned char []) {FRONT_SHARP, LEFT_SHARP, BACK_SHARP, RIGHT_SHARP}, 0.8);

//Giroscopio
Gyroscope gyro(INTERRUPT_GYRO);



//Variable ángulo, se usa como referncia en varias tareas
Angle yaw0(0);

#define DUTY 200
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);



//Sensor de línea
QTRSensorsRC qtrrc((unsigned char[]) {PIN1_QTR,PIN2_QTR, PIN3_QTR, PIN4_QTR,
  PIN5_QTR, PIN6_QTR, PIN7_QTR,PIN8_QTR}, 8, 2500);
  unsigned int sensorValues[8];
  int position = 3500;


    //PID ROTATION
    //Lectura de error de girar
    #define ROTATION_KP 0.3
    #define ROTATION_KI 0
    #define ROTATION_KD 0.08
    #define ROTATION_KF 0.80
    #define ROTATION_SAT 0.20
    #define ROTATION_DEAD 0.11
    #define ROTATION_THRESHOLD 0.04
    PID rotation(
      []()-> double {return (yaw0-gyro.getAlpha()).get();},
      [](double dir){motors.rotate(-constrain(dir,-ROTATION_SAT,ROTATION_SAT));},
      0.15);
    //PID ROTATION



  //PID STRAIGHT
  #define STRAIGHT_KP 1.2
  #define STRAIGHT_KI 0.005
  #define STRAIGHT_KD 0.02
  #define STRAIGHT_SAT 0.20
  PID straight(
    []()-> double {return (yaw0-gyro.getAlpha()).get();},
    [](double dir){motors.move(constrain(dir,-1.0,1.0), STRAIGHT_SAT);},
    1);
  //PID STRAIGHT

  //PID STRAIGHT
  #define BACK_KP 1.2
  #define BACK_KI 0.005
  #define BACK_KD 0.02
  #define BACK_SAT 0.20
  PID back(
    []()-> double {return (yaw0-gyro.getAlpha()).get();},
    [](double dir){motors.move(-constrain(dir,-1.0,1.0), -BACK_SAT);},
    1);
  //PID STRAIGHT


  void testTurn(double value,bool right) {

  long int tiempos = millis();
    motors.move(0,0);
    delay(100);

    yaw0 = yaw0 + value;
    //Angle yawr = yaw0;
    delay(2);

    Serial.println("Al" + String(gyro.getAlpha().get()) + " yaw0" + String(yaw0.get()) + " yaw0-al" + String((gyro.getAlpha() - yaw0).get()));

    int c = 0;
    while(c < 5) {
      gyro.check();
      rotation.check();

      if(abs((yaw0 - gyro.getAlpha()).get()) < ROTATION_THRESHOLD) c++;
      if(millis() - tiempos > 8000) break;
      if(abs((yaw0 - gyro.getAlpha()).get()) < 0.02) {
        motors.rotate((value > 0) ? -0.2 : 0.2); delay(20); break;
      }
      delay(2);
    }

    rotation.resetSumShaft();

    motors.move(0,0);
    delay(100);
  }


  void setup() {

    start();

    //Configuración PIDs
    straight.setKp(STRAIGHT_KP);
    straight.setKi(STRAIGHT_KI);
    straight.setKd(STRAIGHT_KD);

    //Configuración PIDs
    back.setKp(BACK_KP);
    back.setKi(BACK_KI);
    back.setKd(BACK_KD);

    rotation.setKp(ROTATION_KP);
    rotation.setKi(ROTATION_KI);
    rotation.setKd(ROTATION_KD);
    rotation.setKf(ROTATION_KF);
    rotation.setDeadZone(ROTATION_DEAD);

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

    ready();
    flag();

  }

#define THRESHOLD_RIGHT 250//200
#define THRESHOLD_BACK 210
#define FRONT_DELAY 310
  void loop() {
    //Ejecutar PID Recto
    gyro.check();

    sharps.check();
    straight.check();
    if(sharps.get(Dirs::Right) < THRESHOLD_RIGHT) {
      yaw0 = gyro.getAlpha();
      long int times = millis();
      while (millis() - times < FRONT_DELAY) {
        gyro.check();
        straight.check();
        sharps.check();
        delay(2);
      }

      testTurn(-M_PI_2, false);



      for(int i = 0; i < 10; i++) {
        sharps.check();
        delay(2);
      }

      sharps.check();

      while (sharps.get(Dirs::Back) < THRESHOLD_BACK) {
        gyro.check();
        back.check();
        sharps.check();
        delay(2);
      }

      testTurn(M_PI_2, true);

      exit(0);
    }

    delay(2);
    //return;


  }
