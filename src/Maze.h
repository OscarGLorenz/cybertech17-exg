#include "Arduino.h"
#include "EEPROM.h"
#include "Pinout.h"

#include "QTRSensors.cpp"

#include "Motors/Motors.h"
#include "PIDController/PID.h"
#include "DynamicStructures/Queue.h"
#include "Gyroscope/Gyroscope.h"
#include "Mapper/Mapper.h"
#include "Global.h"
#include "Debug/Debug.h"

Maze mazeObj;
MazeIterator mazeItr(mazeObj.getStart());

Sharps sharps((unsigned char []) {FRONT_SHARP, LEFT_SHARP, BACK_SHARP, RIGHT_SHARP}, 0.8);

//Giroscopio
Gyroscope gyro(INTERRUPT_GYRO);

//Variable ángulo, se usa como referncia en varias tareas
Angle yaw0(0);

Queue<char> giros;

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
  #define ROTATION_KP 0.35
  #define ROTATION_KI 0
  #define ROTATION_KD 0.005
  #define ROTATION_KF 0.80
  #define ROTATION_SAT 0.20
  #define ROTATION_DEAD 0.11
  #define ROTATION_THRESHOLD 0.05
  PID rotation([]()-> double {return (yaw0-gyro.getAlpha()).get();}, [](double dir){motors.rotate(-constrain(dir,-ROTATION_SAT,ROTATION_SAT));}, 0.15);
  //PID ROTATION

  //PID STRAIGHT
  #define STRAIGHT_KP 1.2
  #define STRAIGHT_KI 0.002
  #define STRAIGHT_KD 0.02
  #define STRAIGHT_SAT 0.20
  PID straight([]()-> double {return (yaw0-gyro.getAlpha()).get();}, [](double dir){motors.move(constrain(dir,-1.0,1.0), STRAIGHT_SAT);}, 1);
  //PID STRAIGHT

  #define LATERAL_KP 0.001
  #define LATERAL_KI 0
  #define LATERAL_KD 0.001
  PID lateral([]() -> double {return sharps.get(Dirs::Right) - sharps.get(Dirs::Left);}, [](double dir){motors.move(-constrain(dir,-1.0,1.0), STRAIGHT_SAT);}, 1);



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

    motors.move(0,0);
    delay(500);

  }

  void testTurn(double value,bool right) {
    long int tiempos = millis();
    motors.move(0,0);
    delay(500);

    yaw0 = gyro.getAlpha();
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
      if(millis() - tiempos > 4000) break;
      delay(2);

    }

    rotation.resetSumShaft();

    motors.move(0,0);
    delay(500);
  }

  #define THRESHOLD_FRONT 300
  #define THRESHOLD_RIGHT 80
  #define THRESHOLD_LEFT 80

  void knownMaze(Queue<Dirs> dirs) {
    QueueIterator<Dirs> itr = dirs.getIterator();
    while(itr.hasNext()) {
      Dirs now = itr.next();
      sharps.check();
      long int times;

      switch (now) {
        case Dirs::Front:

        while (sharps.get(Dirs::Left) > THRESHOLD_LEFT && sharps.get(Dirs::Right) > THRESHOLD_RIGHT ) {
          gyro.check();
          straight.check();
          delay(2);
          sharps.check();
        }

        times = millis();
        while(millis() - times < 1000) {
          straight.check();
          delay(2);
        }


        break;


        case Dirs::Left:

        while (sharps.get(Dirs::Left) > THRESHOLD_LEFT) {
          gyro.check();
          straight.check();
          delay(2);
          sharps.check();
        }

        motors.move(0,0.15);
        delay(400);
        testTurn(-M_PI_2, false);

        times = millis();
        while(millis() - times < 1000) {
          straight.check();
          delay(2);
        }

        break;


        case Dirs::Right:

        while (sharps.get(Dirs::Right) > THRESHOLD_RIGHT) {
          gyro.check();
          straight.check();
          delay(2);
          sharps.check();
        }

        motors.move(0,0.15);
        delay(400);
        testTurn(M_PI_2, true);

        times = millis();
        while(millis() - times < 1000) {
          straight.check();
          delay(2);
        }

        break;


        case Dirs::Back:

        while (sharps.get(Dirs::Right) < THRESHOLD_FRONT) {
          gyro.check();
          straight.check();
          delay(2);
          sharps.check();
        }

        testTurn(M_PI, true);

        break;
      }

      delay(2);
    }
  }

  void store(Queue<Dirs> dirs) {
    EEPROM.put(0x40,dirs.size());
    for (size_t i = 0; i < dirs.size(); i++) {
      EEPROM.put(0x40+i*2,dirs.get(i));
    }
  }

  Queue<Dirs> restore() {
    Queue<Dirs> dirs;
    Dirs aux;
    size_t size = EEPROM.get(0x40,size);
    for (size_t i = 0; i < size; i++) {
      dirs.pushBack(EEPROM.get(0x40+i*2,aux));
    }
    return dirs;
  }

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

    if(!analogRead(BUTTON)) {
      knownMaze(restore());
    }
  }

  void loop() {

    gyro.check();
    straight.check();
    sharps.check();
    delay(2);

    static long int noRepetir = 0;
    bool isExit = true;
    if(millis() - noRepetir > 1000) {
      if(sharps.get(Dirs::Right) < THRESHOLD_RIGHT) {
        sharps.check();

        Serial.println(String(sharps.get(Dirs::Right)) + " " + String(sharps.get(Dirs::Left)) + " " + String(sharps.get(Dirs::Front)) );

        motors.move(0,0.15);
        delay(400);

        if(sharps.get(Dirs::Front) < 40 && sharps.get(Dirs::Left) < 40 && sharps.get(Dirs::Right) < 40) {
          motors.move(0,0);
          delay(800);
          if (isExit) {
            giros.pushBack('X');
            mazeItr.move(Dirs::Front, Type::EXIT);

            giros.pushBack('B');
            mazeItr.move(Dirs::Back, Type::NORMAL);

            if(sharps.get(Dirs::Left) > sharps.get(Dirs::Right)) {
              testTurn(M_PI,true);
            } else {
              testTurn(M_PI,false);
            }

            noRepetir = millis();

            isExit = !isExit;
          } else {
            giros.pushBack('E');
            mazeItr.move(Dirs::Front, Type::ENTRY);

            QueueIterator<char> itr = giros.getIterator();
            while(itr.hasNext()) {
              Serial.printtab(itr.next());
            }
            Queue<Dirs> dirs = mazeObj.solve();
            store(dirs);
            motors.rotate(0);
            while(1) {delay(250);}
          }



        } else {

          testTurn(M_PI_2,true);
          noRepetir = millis();
          giros.pushBack('R');
          mazeItr.move(Dirs::Right, Type::NORMAL);
        }
      } else if (sharps.get(Dirs::Front) > THRESHOLD_FRONT){
        if(sharps.get(Dirs::Left) < THRESHOLD_LEFT) {
          yaw0 = gyro.getAlpha();
          motors.move(0,-0.15);
          delay(350);
          testTurn(-M_PI_2,false);
          giros.pushBack('L');
          mazeItr.move(Dirs::Left, Type::NORMAL);

        } else {
          yaw0 = gyro.getAlpha();
          motors.move(0,-0.15);
          delay(350);
          giros.pushBack('B');
          mazeItr.move(Dirs::Back, Type::NORMAL);

          if(sharps.get(Dirs::Left) > sharps.get(Dirs::Right)) {
            testTurn(M_PI,true);
          } else {
            testTurn(M_PI,false);
          }
        }
        noRepetir = millis();

      } else if(sharps.get(Dirs::Left) < THRESHOLD_LEFT && millis() - noRepetir > 800) {
        giros.pushBack('F');
        mazeItr.move(Dirs::Front, Type::NORMAL);
        noRepetir = millis();
      }
    }

    delay(2);
  }
