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
MazeIterator mazeItr(&mazeObj);

Sharps sharps((unsigned char []) {FRONT_SHARP, LEFT_SHARP, BACK_SHARP, RIGHT_SHARP}, 0.6);
Gyroscope gyro(INTERRUPT_GYRO);
Angle yaw0(0);

Queue<char> giros;
Queue<Type> checkpoints;

#define DUTY 200
Motors motors((char []) {PWM1B, PWM1A}, (char []) {PWM2B, PWM2A}, DUTY);

QTRSensorsRC qtrrc((unsigned char[]) {PIN1_QTR,PIN2_QTR, PIN3_QTR, PIN4_QTR, PIN5_QTR, PIN6_QTR, PIN7_QTR,PIN8_QTR}, 8, 2500);
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
#define STRAIGHT_KP 4
#define STRAIGHT_KI 0.12
#define STRAIGHT_KD 0.05
#define STRAIGHT_SAT 0.20
PID straight([]()-> double {return (yaw0-gyro.getAlpha()).get();}, [](double dir){motors.move(constrain(dir,-1.0,1.0), STRAIGHT_SAT);}, 1);
//PID STRAIGHT

void showDir(Queue<Movement> dirs) {
  QueueIterator<Movement> itrDirs = dirs.getIterator();
  while(itrDirs.hasNext()) {
    Movement mov = itrDirs.next();
    char c = '\0';
    if (mov.type == Type::CHECK1) {
      c = '1';
    } else if (mov.type == Type::CHECK1) {
      c = '2';
    } else {
      switch(mov.dir) {
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
  }
  Serial.println();
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

#define THRESHOLD_FRONT 400
#define THRESHOLD_RIGHT 80
#define THRESHOLD_LEFT 80

void queHay(MazeIterator &itrMaze) {
    if(itrMaze.actual()->type == Type::NORMAL)
        Serial.print("NORMAL ");
    else if(itrMaze.actual()->type == Type::ENTRY)
        Serial.print("ENTRY ");
    else if(itrMaze.actual()->type == Type::CHECK1)
        Serial.print("CHECK1 ");
    else if(itrMaze.actual()->type == Type::CHECK2)
        Serial.print("CHECK2 ");
    else if(itrMaze.actual()->type == Type::CHECK3)
        Serial.print("CHECK3 ");
    else if(itrMaze.actual()->type == Type::CHECK4)
        Serial.print("CHECK4 ");

    Serial.printtab(itrMaze.actual()->id);

    if(itrMaze.actual()->front != nullptr)
        Serial.print('F');
    if(itrMaze.actual()->left != nullptr)
        Serial.print('L');
    if(itrMaze.actual()->right != nullptr)
        Serial.print('R');
    if(itrMaze.actual()->back != nullptr)
        Serial.print('B');
}

void knownMaze(Queue<Movement> movs) {
  QueueIterator<Movement> itr = movs.getIterator();
  long int noRepetir = millis();
  itr.next(); // NOS SALTAMOS EL PRIMERO QUE ES UN FRONT
  while(itr.hasNext()) {
    BRK
    Movement move = itr.next();
    Dirs now = move.dir;
    sharps.check();
    long int times = millis();

    while (millis() - noRepetir < 800) {
      gyro.check();
      straight.check();
      delay(2);
      sharps.check();
    }

    if (move.type != Type::NORMAL) {
      gyro.check();
      straight.check();
      delay(2);
      sharps.check();
      continue;
    }

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
      delay(200);
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
      delay(200);
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
      if(sharps.get(Dirs::Left) > sharps.get(Dirs::Right)) {
        testTurn(M_PI_2,true);
        testTurn(M_PI_2,true);
      } else {
        testTurn(-M_PI_2,false);
        testTurn(-M_PI_2,false);
      }
      break;
    }
    noRepetir = millis();
    delay(2);
  }
}

void read() {//Contar lineas
  static bool overSlash = false; //Variable para situacion de linea

  static unsigned long ref = 0; //Ultima lectura
  static int countmoves = 0; //Grupo de lineas

  int sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += sensorValues[i]; //Sumar sensores 3 y 6
  }

  if (millis()-ref > 300 && countmoves >= 1) { //¿Hemos abandonado un grupo de lineas?
    switch (countmoves) {
      // case 1: mazeItr.move(Dirs::Front, Type::CHECK1); if(!checkpoints.has(Type::CHECK1)) break;
      // case 2: mazeItr.move(Dirs::Front, Type::CHECK2); if(!checkpoints.has(Type::CHECK2)) break;
      // case 3: mazeItr.move(Dirs::Front, Type::CHECK3); if(!checkpoints.has(Type::CHECK3)) break;
      // case 4: mazeItr.move(Dirs::Front, Type::CHECK4); if(!checkpoints.has(Type::CHECK4)) break;;
      // case 5: mazeItr.move(Dirs::Front, Type::CHECK5); if(!checkpoints.has(Type::CHECK5)) break;
      //if(checkpoints.size() >= 5) TERMINOSE Y GUARDAR ABAJO ESTÁ
      case 1:
      giros.pushBack('1');
      mazeItr.move(Dirs::Front, Type::CHECK1);
      motors.rotate(0);
      delay(50);

      Serial.println("RESULTS.....");
      QueueIterator<char> itr = giros.getIterator();
      while(itr.hasNext()) {
        Serial.printtab(itr.next());
      }


Serial.end();
      while(1) {
        if(!digitalRead(BUTTON)) {
          Serial.begin(9600);
          Serial.println("SOLVING MAZE.....");
          MazeIterator mazeItr2(&mazeObj);
          for(; mazeItr2.actual()->type != Type::CHECK1; mazeItr2.movePriority(true)) {
               queHay(mazeItr2); Serial.println();
           }
           queHay(mazeItr2);
           Serial << endl;
          Queue<Movement> movs = mazeObj.solver();
          showDir(movs);

          Serial << "BIGOTITOS SABE LABERINTO" << endl;
          ready();

          long int times = millis();
          while(millis() - times < 4000) {
            gyro.check();
            delay(2);
            yaw0 = gyro.getAlpha();
            sharps.check();
            limitedSerial("Sha " + String(sharps.get(Dirs::Right)) + " " + String(sharps.get(Dirs::Left)) + " " + String(sharps.get(Dirs::Front)) ,150);
          }

          yaw0 = gyro.getAlpha();

          flag();
          knownMaze(movs);

          while(1) {
            gyro.check();
            delay(2);
            yaw0 = gyro.getAlpha();
            sharps.check();
            limitedSerial("Sha " + String(sharps.get(Dirs::Right)) + " " + String(sharps.get(Dirs::Left)) + " " + String(sharps.get(Dirs::Front)) ,150);
          }

        }
        delay(2);
      }
    }

    Serial.println(countmoves);
    countmoves = 0;
  }

  if (!overSlash) { //¿Estamos sobre la linea?
    if (sum > 4000) { //Si su suma es grande estamos sobre una linea

      overSlash = true; //Ahora estamos sobre la línea
      countmoves++; //Sumamos una linea
      ref = millis();

      Serial.println(millis()); //Milisegundos actuales
    }
  } else {
    if (sum < 4000) { //Si salimos de la línea ponemos que estamos fuera
      overSlash = false;
    }
  }
}
//CONTADOR DE LÍNEAS
long int times = millis();
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
  delay(100);
  yaw0 = gyro.getAlpha();

  unsigned int * max = (unsigned int *) malloc(sizeof(unsigned int)*8);
  unsigned int * min = (unsigned int *) malloc(sizeof(unsigned int)*8);
  memcpy(max, maxLine, sizeof(unsigned int)*8);
  memcpy(min, minLine, sizeof(unsigned int)*8);

  qtrrc.calibratedMaximumOn = max;
  qtrrc.calibratedMinimumOn = min;
  Serial.begin(9600);

  while(millis() - times < 4000) {
    gyro.check();
    delay(2);
    yaw0 = gyro.getAlpha();
    sharps.check();
    limitedSerial("Sha " + String(sharps.get(Dirs::Right)) + " " + String(sharps.get(Dirs::Left)) + " " + String(sharps.get(Dirs::Front)) ,150);
  }

  yaw0 = gyro.getAlpha();
  flag();
  mazeItr.move(Dirs::Front, Type::NORMAL);

}

void loop() {
  limitedSerial("Sha " + String(sharps.get(Dirs::Right)) + " " + String(sharps.get(Dirs::Left)) + " " + String(sharps.get(Dirs::Front)) ,150);
  sharps.check();

  gyro.check();

  straight.check();

  position = (int) qtrrc.readLine(sensorValues); //Leer línea
  read(); //Leer líneas cruzadas


  static long int noRepetir = 0;
  if(millis() - noRepetir > 800) {
    limitedSerial("Sha " + String(sharps.get(Dirs::Right)) + " " + String(sharps.get(Dirs::Left)) + " " + String(sharps.get(Dirs::Front)) ,150);
    if(sharps.get(Dirs::Right) < THRESHOLD_RIGHT) {
      motors.move(0,0.15);

      unsigned long timer = millis();
      while(millis() - timer < 300) {
        sharps.check();
        delay(1);
      }

      if (sharps.get(Dirs::Front) > THRESHOLD_FRONT){
        yaw0 = gyro.getAlpha();
        motors.move(0,-0.15);
        delay(300);
      }

      testTurn(M_PI_2,true);
      noRepetir = millis();
      giros.pushBack('R');
      Serial.println('R');
      mazeItr.move(Dirs::Right, Type::NORMAL);

    } else if (sharps.get(Dirs::Front) > THRESHOLD_FRONT){
      yaw0 = gyro.getAlpha();
      motors.move(0,-0.15);
      delay(300);

      if(sharps.get(Dirs::Left) < THRESHOLD_LEFT) {
        testTurn(-M_PI_2,false);
        giros.pushBack('L');
        Serial.println('L');

        mazeItr.move(Dirs::Left, Type::NORMAL);

      } else {
        giros.pushBack('B');
        Serial.println('B');

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

    } else if(sharps.get(Dirs::Left) < THRESHOLD_LEFT  && millis() - noRepetir > 1200 && sharps.get(Dirs::Front) < 100 && (sharps.get(Dirs::Right) > 115)) {
      giros.pushBack('F');
      Serial.println('F');

      mazeItr.move(Dirs::Front, Type::NORMAL);
      noRepetir = millis();
    }
  }

  delay(2);
}
