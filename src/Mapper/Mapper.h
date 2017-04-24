#ifndef MAPPER_H_
#define MAPPER_H_

#include "Arduino.h"
#include "../Global.h"
#include "../DynamicStructures/Queue.h"

enum class Type {NORMAL = 0, ENTRY=-1, CHECK1 = 1, CHECK2 = 2, CHECK3 = 3, CHECK4 = 4, CHECK5 = 5};

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

Dirs ckWise(Dirs dir) {
    switch (dir) {
    case Dirs::Front:
        return Dirs::Right;
    case Dirs::Left:
        return Dirs::Front;
    case Dirs::Right:
        return Dirs::Back;
    case Dirs::Back:
        return Dirs::Left;
    }
    return Dirs::Front;
}

Dirs cntCkWise(Dirs dir) {
    switch (dir) {
    case Dirs::Front:
        return Dirs::Left;
    case Dirs::Left:
        return Dirs::Back;
    case Dirs::Right:
        return Dirs::Front;
    case Dirs::Back:
        return Dirs::Right;
    }
    return Dirs::Front;
}

Dirs reverse(Dirs dir) {
    switch (dir) {
    case Dirs::Front:
        return Dirs::Back;
    case Dirs::Left:
        return Dirs::Right;
    case Dirs::Right:
        return Dirs::Left;
    case Dirs::Back:
        return Dirs::Front;
    }
    return Dirs::Front;
}

//Una celda
class Cell {
    public:
        //Crea la celda, dada una dirección y otra celda, enlaza,
        //el puntero de la dirección dada a la celda dada, establece
        //el tipo de celda y le da un id
        Cell(Dirs from, Cell * fromCell, Type t, int index);

        //Cambia el puntero de la dirección dada a otro dado
        void change(Dirs dir, Cell * cell);

        //Devuelve la dirección del puntero según la dirección dada
        Cell * point(Dirs dir);

        //Devuelve el número de punteros no nulos
        int connections();

        //Id de la celda
        int id;

        //Tipo de celda
        Type type;

        //Punteros a celda en cada una de las direcciones
        Cell * front;
        Cell * left;
        Cell * right;
        Cell * back;
};

class Maze;

class MazeIterator {
    public:
        //Nuevo iterador, celda seleccionada, direccion seleccionada
        MazeIterator(Maze *m, Dirs head = Dirs::Front);

        MazeIterator(MazeIterator &itr);

        //Recorre el laberinto segun la regla especificada
        Dirs movePriority(bool rightWise);

        //Moviento manual, si no hay celda, la crea;
        void move(Dirs dir, Type type);

        //+180º a heading
        void reverse(void);

        //-90º a heading
        void ckWise(void);
        //+90º a heading
        void cntWise(void);

        //Devuelve numero de conexiones en la celda actual
        int connections();

        //Se mueve segun prioridad y elimina a su paso
        void deleteAndMove(bool rightWise);

        //Usa heading y la dirección dada para devolver, la
        //dirección relativa
        Dirs relative(Dirs after, Dirs before);

        //Devuelve el id de la celda actual
        int id();
        //Devuelve el tipo de celda actual
        Type type();

        //Devuelve la dirección
        Dirs head();
        //Devuelve la celda actual
        Cell * actual();

        Maze * getMaze();

    private:
        //A que celda se apunta en esa dirección
        Cell * point(Dirs dir);

        //Celda actual
        Cell * now;

        //Dirección del iterador
        Dirs heading;

        Maze * maze;
};


//Clase Maze, administra las celdas
class Maze {
    public:
        Maze();

        Maze(Maze &maze);

        Queue<Movement> solver();

		Queue<Movement> solve(Type start, Type goal);

        //Devuelve la dirección de la primera celda
        Cell * getStart();

        //Contador de celdas
        int count;
    private:
        //Primera Celda
        Cell * first;

        //Elimina una rama en la direccion señalada
        void autoFree(MazeIterator itr);

};
#endif
