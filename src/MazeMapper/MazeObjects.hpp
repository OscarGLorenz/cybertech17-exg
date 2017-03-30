#ifndef MazeObjects_hpp
#define MazeObjects_hpp

#include <Arduino.h>

//Direcciones
enum Dir {
	FRONT, LEFT, RIGHT, BACK

};

//Tipos de Celda
enum Type {
	NORMAL, EXIT, ENTRY, CHECK1, CHECK2, CHECK3, CHECK4
};

//ArrayList para cualquier objeto/tipo

template<class T> class ArrayList {
public:
	//Asigna a lenght y array 0
	ArrayList();

	//Libera el espacio
	~ArrayList();

	//Añade al final
	void add(T t);

	//Modifica el elemento especificado, si i >= size modifica el último elemento, si i < 0 el primero
	 T &operator[](const int i);

	//Retorna el elemento especificado, si i >= size devuelve el último elemento, si i < 0 el primero
	const T &operator[](const int i) const;

	//Retorna true si el elemento existe en el ArrayList, debe tener definido
	//el operador == si es un objeto
	bool has(T t);

	//Devuelve el número de elementos
	int size();

private:
	//Dirección del ArrayList
	T * array;

	//Numero de Elementos
	int lenght;
};

//Una celda
class Cell {
public:
	//Crea la celda, dada una dirección y otra celda, enlaza,
	//el puntero de la dirección dada a la celda dada, establece
	//el tipo de celda y le da un id
	Cell(Dir from, Cell * fromCell, Type type);

	//Cambia el puntero de la dirección dada a otro dado
	void change(Dir dir, Cell * cell);

	//Devuelve la dirección del puntero según la dirección dada
	Cell * point(Dir dir);

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

class Iterator {
public:
	//Nuevo iterador, celda inicial, direccion inicial
	Iterator();

	//Nuevo iterador, celda seleccionada, direccion seleccionada
	Iterator(Dir head, Cell * cell);

	//Recorre el laberinto segun la regla especificada
	Dir movePriority(bool rightWise);

	//Moviento manual, si no hay celda, la crea;
	void move(Dir dir, Type type);

	//+180º a heading
	void reverse();

	//-90º a heading
	void ckWise();

	//+90º a heading
	void cntWise();

	//Devuelve numero de conexiones en la celda actual
	int connections();

	//Se mueve segun prioridad y elimina a su paso
	void deleteAndMove(bool rightWise);

	//Usa heading y la dirección dada para devolver, la
	//dirección relativa
	Dir relative(Dir p);

	//Devuelve el id de la celda actual
	int id();

	//Devuelve el tipo de celda actual
	Type type();

	//Devuelve la dirección
	Dir head();

	//Devuelve la celda actual
	Cell * actual();
private:
	//A que celda se apunta en esa dirección
	Cell * point(Dir dir);

	//Celda actual
	Cell * now;

	//Dirección del iterador
	Dir heading;
};


//Clase Maze, administra las celdas
class Maze {
public:
	//Devuelve un arraylist con las direcciones para salir
	//lo más directo posible del laberinto
	static ArrayList<Dir> solve();

	//Devuelve la dirección de la primera celda
	static Cell * getStart();

	//Da la dirección siguiente a la dada en dirección
	//a las agujas del reloj
	static Dir ckWise(Dir dir);

	//Da la dirección siguiente a la dada en dirección
	//contraria a las agujas del reloj
	static Dir cntCkWise(Dir dir);

	//Da la dirección contraria a la dada
	static Dir reverse(Dir dir);

	//Contador de celdas
	static int count;
private:
	//Primera Celda
	static Cell * first;

	//Elimina una rama en la direccion señalada
	static void autoFree(Iterator itr);


};

#endif
