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

Queue<char> giros;

Queue<Type> checkpoints;



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

void showDir(Queue<Movement> dirs) {
  QueueIterator<Movement> itrDirs = dirs.getIterator();
  while(itrDirs.hasNext()) {
    Movement mov = itrDirs.next();
    char c = '\0';
    if (mov.type == Type::CHECK1) {
      c = '1';
    } else if (mov.type == Type::CHECK2) {
      c = '2';
    } else if (mov.type == Type::CHECK3) {
      c = '3';
    } else if (mov.type == Type::CHECK4) {
      c = '4';
    } else if (mov.type == Type::CHECK5) {
      c = '5';
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

//CONTADOR DE LÍNEAS
long int times = millis();

#define FR Dirs::Front, Type::NORMAL
#define LF Dirs::Left, Type::NORMAL
#define RG Dirs::Right, Type::NORMAL
#define BK Dirs::Back, Type::NORMAL
#define CK1 Dirs::Front, Type::CHECK1
#define CK2 Dirs::Front, Type::CHECK2
#define CK3 Dirs::Front, Type::CHECK3
#define CK4 Dirs::Front, Type::CHECK4

void setup() {

  //start();
  Serial.begin(9600);
  mazeItr.move(FR);

  mazeItr.move(RG);
  mazeItr.move(FR);

  mazeItr.move(RG);
  mazeItr.move(RG);
  mazeItr.move(BK);
  mazeItr.move(LF);
  mazeItr.move(LF);

  mazeItr.move(RG);
  mazeItr.move(RG);
  mazeItr.move(BK);
  mazeItr.move(FR);

  mazeItr.move(BK);
  mazeItr.move(RG);
  mazeItr.move(RG);
  mazeItr.move(FR);
  mazeItr.move(RG);
  mazeItr.move(CK1);

  MazeIterator mazeItr2(&mazeObj);
  for(; mazeItr2.actual()->type != Type::CHECK1; mazeItr2.movePriority(true)) {
       queHay(mazeItr2); Serial.println();
   }
   queHay(mazeItr2);
   Serial.println();
  showDir(mazeObj.solver());
  
  Serial << "test " << " tusa " << 47 << ' ' << 6.8 << endl;

}

void loop() {

}
