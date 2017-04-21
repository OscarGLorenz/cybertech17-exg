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
  #define STRAIGHT_KI 0.02
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
    Serial.println(value);
    //yaw0 = gyro.getAlpha();
    yaw0 = yaw0 + value;
    //Angle yawr = yaw0;

    Serial.println("Ang " + String(yaw0.get()) + " " + String(gyro.getAlpha().get()) + " " + String((yaw0-gyro.getAlpha()).get()));
    delay(2);

    int c = 0;
    while(c < 30 ) {
      //  yawr = gyro.getAlpha();
      //Salida por pantalla de ángulo actual y objetivo
      //limitedSerial(String(gyro.getAlpha().get()) + " " + String(yaw0.get()), 10);

      //Lectura giroscopio
      gyro.check();

      //Ejecutamos PID rotación
      rotation.check();

      if(abs((yaw0 - gyro.getAlpha()).get()) < ROTATION_THRESHOLD) c++;
      if(millis() - tiempos > 4000 && abs((yaw0 - gyro.getAlpha()).get()) < 0.1) break;
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
          gyro.check();
          straight.check();
          delay(2);
          sharps.check();
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
        while(millis() - times < 800) {
          gyro.check();
          straight.check();
          delay(2);
          sharps.check();
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
        while(millis() - times < 500) {
          gyro.check();
          straight.check();
          delay(2);
          sharps.check();
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
    EEPROM.put(0x40,(uint8_t) dirs.size());

    QueueIterator<Dirs> itrDirs = dirs.getIterator();
    int i = 1;
    while(itrDirs.hasNext()) {
      char c = 'A';
      switch(itrDirs.next()) {
        case Dirs::Front:
        c = 'F';
        break;
        case Dirs::Left:
        c = 'L';
        break;
        case Dirs::Right:
        c = 'R';
        break;
        case Dirs::Back:
        c = 'B';
        break;
      }
      EEPROM.put(0x40+i*sizeof(char),c);
      i++;
    }
  }

  Queue<Dirs> restore() {
    Queue<Dirs> dirs;
    uint8_t size = EEPROM.get(0x40,size);
    char c;

    for (uint8_t i = 0; i < size; i++) {
      Dirs aux;
      switch(EEPROM.get(0x40+(i+1)*sizeof(char),c)) {
        case 'F':
          aux = Dirs::Front;
        break;
        case 'L':
          aux = Dirs::Left;
        break;
        case 'R':
          aux = Dirs::Right;
        break;
        case 'B':
          aux = Dirs::Back;
        break;
      }
      dirs.pushBack(aux);
    }

    QueueIterator<Dirs> itrDirs = dirs.getIterator();
    while(itrDirs.hasNext()) {
      char c = '\0';
      switch(itrDirs.next()) {
        case Dirs::Front:
        c = 'F';
        break;
        case Dirs::Left:
        c = 'L';
        break;
        case Dirs::Right:
        c = 'R';
        break;
        case Dirs::Back:
        c = 'B';
        break;
      }
      Serial.printtab(c);
    }
    Serial.println();

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

    //Queue<Dirs> queue;
    //Dirs dirs[] = {Dirs::Right,Dirs::Right,Dirs::Left,Dirs::Right,Dirs::Left,Dirs::Left,Dirs::Right};
    //queue.add(dirs, 7);

  //  store(queue);
  //  restore();
//if(!digitalRead(BUTTON)) knownMaze(restore());
//return;

  //    knownMaze(queue);
      //knownMaze(restore());
  //    motors.rotate(0);
    //exit(0);

  }

  void loop() {

    gyro.check();
    straight.check();
    sharps.check();

    static long int noRepetir = 0;
    bool isExit = true;
    if(millis() - noRepetir > 800) {
      limitedSerial("Sha " + String(sharps.get(Dirs::Right)) + " " + String(sharps.get(Dirs::Left)) + " " + String(sharps.get(Dirs::Front)) ,150);
      if(sharps.get(Dirs::Right) < THRESHOLD_RIGHT) {
        motors.move(0,0.15);
        unsigned long timer = millis();
        while(millis() - timer < 300) {
           sharps.check();
           delay(2);
        }

        if(sharps.get(Dirs::Front) < 40 && sharps.get(Dirs::Right) < 40) {
          for(int i = 0; i < 10; i++) {sharps.check(); delayMicroseconds(500);}
        }


         if(sharps.get(Dirs::Front) < 40 && sharps.get(Dirs::Left) < 40 && sharps.get(Dirs::Right) < 40) {
           motors.move(0,0);
           delay(800);
           if (isExit) {
               giros.pushBack('X');
               mazeItr.move(Dirs::Front, Type::EXIT);

               giros.pushBack('B');
               mazeItr.move(Dirs::Back, Type::NORMAL);

              testTurn(M_PI_2,true);
              testTurn(M_PI_2,true);

              noRepetir = millis();

              isExit = false;
              Serial.println("HE ZALIDO");

           } else {
               giros.pushBack('E');
               mazeItr.move(Dirs::Front, Type::ENTRY);

               motors.rotate(0);
               delay(50);
               Serial.println("RESULTS.....");

               QueueIterator<char> itr = giros.getIterator();
               while(itr.hasNext()) {
                Serial.printtab(itr.next());
              }

              Serial.println("SOLVING MAZE.....");

              Queue<Dirs> dirs = mazeObj.solve();
              QueueIterator<Dirs> itrDirs = dirs.getIterator();
              while(itrDirs.hasNext()) {
                char c = '\0';
                switch(itrDirs.next()) {
                  case Dirs::Front:
                  c = 'F';
                  break;
                  case Dirs::Left:
                  c = 'L';
                  break;
                  case Dirs::Right:
                  c = 'R';
                  break;
                  case Dirs::Back:
                  c = 'B';
                  break;
                }
                Serial.printtab(c);
              }
              Serial.println();

              store(dirs);

              Serial.println("RESTORING DATA.....");
              Queue<Dirs> checkDirs = restore();
              itrDirs = checkDirs.getIterator();
              while(itrDirs.hasNext()) {
                char c = '\0';
                switch(itrDirs.next()) {
                  case Dirs::Front:
                  c = 'F';
                  break;
                  case Dirs::Left:
                  c = 'L';
                  break;
                  case Dirs::Right:
                  c = 'R';
                  break;
                  case Dirs::Back:
                  c = 'B';
                  break;
                }
                Serial.printtab(c);
              }
              Serial.println();



               while(1) {delay(250);}
           }



         } else {
           if (sharps.get(Dirs::Front) > THRESHOLD_FRONT){
             yaw0 = gyro.getAlpha();
             motors.move(0,-0.15);
             delay(350);
           }

             testTurn(M_PI_2,true);
             noRepetir = millis();
            giros.pushBack('R');
            mazeItr.move(Dirs::Right, Type::NORMAL);
         }
      } else if (sharps.get(Dirs::Front) > THRESHOLD_FRONT){
        yaw0 = gyro.getAlpha();
        motors.move(0,-0.15);
        delay(350);

        if(sharps.get(Dirs::Left) < THRESHOLD_LEFT) {
          testTurn(-M_PI_2,false);
          giros.pushBack('L');
          mazeItr.move(Dirs::Left, Type::NORMAL);

        } else {
          giros.pushBack('B');
          mazeItr.move(Dirs::Back, Type::NORMAL);

          if(sharps.get(Dirs::Left) > sharps.get(Dirs::Right)) {
            testTurn(M_PI_2,true);
            testTurn(M_PI_2,true);
          } else {
            testTurn(-M_PI_2,false);
            testTurn(-M_PI_2,false);
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
