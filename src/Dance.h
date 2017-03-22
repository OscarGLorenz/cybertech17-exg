#include "Arduino.h"

#include "Pinout.h"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "DynamicStructures/Queue.h"
#include "Gyroscope/Gyroscope.h"

#include "Global.h"

#include "QTRSensors.cpp"

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



  unsigned long rottime = 0;
  Angle yawLast(0);

  //PID ROTATION
  //Lectura de error de girar
  double inputR() {
    return (yaw0-gyro.getAlpha()).get();

  }
  //Salida de PID de girar
  void outputR(double dir) {
    motors.move(1, constrain(dir,-0.4,0.4));
  }
  #define ROTATION_KP 0.5
  #define ROTATION_KI 4
  #define ROTATION_KD 0.02
  PID rotation(inputR,outputR,1,1);
  //PID ROTATION




  //PID STRAIGHT
  //Lectura de error de ir recto
  double inputS() {
    return (yaw0-gyro.getAlpha()).get();
  }
  //Salida de PID de ir recto
  void outputS(double dir) {
    motors.move(-constrain(dir,-1.0,1.0), -0.3);
  }
  #define STRAIGHT_KP 1.2
  #define STRAIGHT_KI 0
  #define STRAIGHT_KD 0.05 //10
  PID straight(inputS,outputS,1,1);
  //PID STRAIGHT




  //PID SIGUELINEAS
  double input(void) {
    return (position - 3500.0)/3500.0;
  }

  void output(double dir) {
    motors.move(-constrain(dir,-1.0,1.0), 0.2);
  }
  #define LINE_KP 2
  #define LINE_KI 0
  #define LINE_KD 0.02
  PID pid(input,output);
  //PID SIGUELINEAS




  //MOVIMIENTOS
  void turn(double value) {
    yaw0 = gyro.getAlpha();
    yaw0 = yaw0 + value;

    //Giro, si conseguimos el ángulo deseado y lo leemos CHECKCOUNT veces salimos del while
    int c = 0;
    while(c < 40) {
      //Salida por pantalla de ángulo actual y objetivo
      //limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 250);
      Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));

      //Lectura giroscopio
      gyro.check();

      //Ejecutamos PID rotación
      rotation.check();

      delay(2);

      //Si el error es menor que RAD_TOLERANCE sumamos
      if(abs((yaw0 - gyro.getAlpha()).get()) < 0.02) c++;
    }

    motors.move(0,0);
  }

  void goBack() {
    yaw0 = gyro.getAlpha();

    double distance = 0;
    double velocity = 0;
    while(abs(distance) < 0.25) {
      gyro.check();

      straight.check();

      velocity+=gyro.aa.x/175.0*0.002;
      distance+=velocity*0.002;
      delay(2);
    }
    motors.rotate(0,0);

  }
  //MOVIMIENTOS




  //CONTADOR DE LÍNEAS
  Queue<int> mov; //Guardamos movimientos

  void execute() {
    delay(500);

    QueueIterator<int> itr_mov = mov.getIterator(); //Iterador de movimientos

    while(itr_mov.hasNext()) { //Iteramos por los movimientos y los hacemos
      switch(itr_mov.next()) {
        case 1:
        turn(M_PI);
        break;
        case 2:
        goBack();
        break;
        case 3:
        turn(M_PI_2);
        break;
        case 4:
        turn(-M_PI_2);
        break;
      }
    }

    while(1) {motors.move(0,0);} //Detenemos el robot

  }

  void read() {//Contar lineas
    static bool overSlash = false; //Variable para situacion de linea

    static unsigned long ref = 0; //Ultima lectura
    static int countmoves = 0; //Grupo de lineas

    int sum = sensorValues[2] + sensorValues[5]; //Sumar sensores 3 y 6

    if(mov.size() >= 6) { //¿Hemos visto 6 grupos ya?
      motors.rotate(0,0); //Parar
      execute(); //Ejecutar movimientos
    }

    if (millis()-ref > 300 && countmoves >= 1) { //¿Hemos abandonado un grupo de lineas?
      mov.pushBack(countmoves); //Registramos el número
      Serial.println(countmoves);
      countmoves = 0;
    }

    if (!overSlash) { //¿Estamos sobre la linea?
      if (sum > 1500) { //Si su suma es grande estamos sobre una linea

        overSlash = true; //Ahora estamos sobre la línea

        Serial.println(millis()); //Milisegundos actuales

        if(countmoves == 0) { //Nuevo grupo de lineas
          ref = millis();
          countmoves = 1;
          return;
        }

        countmoves++; //Sumamos una linea

        ref = millis();

      }
    } else {
      if (sum < 1500) { //Si salimos de la línea ponemos que estamos fuera
        overSlash = false;
      }
    }
  }
  //CONTADOR DE LÍNEAS





  void setup(){
    start();//Nivel bateria, +++



    //CONSTANTES PID
    straight.setKp(STRAIGHT_KP);
    straight.setKd(STRAIGHT_KD);

    pid.setKp(LINE_KP);
    pid.setKd(LINE_KD);

    rotation.setKp(ROTATION_KP);
    rotation.setKi(ROTATION_KI);
    rotation.setKd(ROTATION_KD);
    //CONSTANTES PID



    //CALIBRACIÓN
    // Serial.println("QTR-8RC Calibration");
    // delay(500);
    // for (int i = 0; i < 100; i++) {
    //   qtrrc.calibrate();
    // }
    // Serial.println("Calibration finished");
    //CALIBRACIÓN

    ready();

    //Inicio giroscopio
    gyro.init();
    //Ángulo inicial

    yaw0 = gyro.getAlpha();

    while(millis() < 4000) {
      gyro.check();
      delay(2);
      yaw0 = gyro.getAlpha();

      Serial.println(String(gyro.getAlpha().get()) + " " + String(yaw0.get()));
    }
    yaw0 = gyro.getAlpha();

    ready();

    // for(int i=0; i < 4; i++) {
    //   turn(M_PI_2);
    //   delay(1000);}
    // for(int i=0; i < 4; i++) {
    //   turn(-M_PI_2);
    //   delay(1000);}
    // ready();
    // exit(0);
  }

  void loop() {
    position = (int) qtrrc.readLine(sensorValues); //Leer línea
    pid.check(); //Ejecutar pid linea
    read(); //Leer líneas cruzadas

    gyro.check();//Leer giroscopio

    //delay(2);
  }
