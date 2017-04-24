#ifndef MAPPER_H_
#define MAPPER_H_

#include "Arduino.h"
#include "../DynamicStructures/Queue.h"

enum class Type {NORMAL = 0, ENTRY=-1, CHECK1 = 1, CHECK2 = 2, CHECK3 = 3, CHECK4 = 4, CHECK5 = 5};

#ifndef DIRS_
#define DIRS_
enum class Dirs {Front = 0, Left = 1, Back = 2, Right = 3};
#endif

class Movement {
public:
  Movement(Type t, Dirs d) {
    type = t;
    dir = d;
  }
  Movement() {
    type = Type::NORMAL;
    dir = Dirs::Front;
  }
  Type type;
  Dirs dir;
};

class Cell { //Una celda
public:
  //Crea la celda, dada una dirección y otra celda, enlaza,
  //el puntero de la dirección dada a la celda dada, establece
  //el tipo de celda y le da un id
  Cell(Dirs from, Cell * fromCell, Type t, int index);
  void change(Dirs dir, Cell * cell); //Cambia el puntero de la dirección dada a otro dado
  Cell * point(Dirs dir); //Devuelve la dirección del puntero según la dirección dada
  int connections(); //Devuelve el número de punteros no nulos
  int id; //Id de la celda
  Type type; //Tipo de celda
  //Punteros a celda en cada una de las direcciones
  Cell * front;
  Cell * left;
  Cell * right;
  Cell * back;
};

class Maze;

class MazeIterator {
public:
  MazeIterator(Maze *m, Dirs head = Dirs::Front); //Nuevo iterador, celda seleccionada, direccion seleccionada
  MazeIterator(MazeIterator &itr); //Recorre el laberinto segun la regla especificada
  Dirs movePriority(bool rightWise); //Moviento manual, si no hay celda, la crea;
  void move(Dirs dir, Type type); //+180º a heading
  void reverse(void); //-90º a heading
  void ckWise(void); //+90º a heading
  void cntWise(void); //Devuelve numero de conexiones en la celda actual
  int connections(); //Se mueve segun prioridad y elimina a su paso
  void deleteAndMove(bool rightWise); //Usa heading y la dirección dada para devolver, la
  //dirección relativa
  Dirs relative(Dirs after, Dirs before);  //Devuelve el id de la celda actual
  int id(); //Devuelve el tipo de celda actual
  Type type(); //Devuelve la dirección
  Dirs head(); //Devuelve la celda actual
  Cell * actual();
  Maze * getMaze();

private:
  Cell * point(Dirs dir);  //A que celda se apunta en esa dirección
  Cell * now;
  Dirs heading; //Dirección del iterador
  Maze * maze; //Celda actual
};

//Clase Maze, administra las celdas
class Maze {
public:
  Maze();
  Maze(Maze &maze);
  Queue<Movement> solver();
  Queue<Movement> solve(Type start, Type goal);
  Cell * getStart(); //Devuelve la dirección de la primera celda
  int count;  //Contador de celdas
private:
  Cell * first; //Primera Celda
  void autoFree(MazeIterator itr);

};
#endif
