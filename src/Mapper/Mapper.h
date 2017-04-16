#include "Arduino.h"
#include "../Global.h"
#include "../DynamicStructures/Queue.h"

enum class Type {
	NORMAL, EXIT, ENTRY, CHECK1, CHECK2, CHECK3, CHECK4
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
	Cell(Dirs from, Cell * fromCell, Type type) {
    front = nullptr;
    left = nullptr;
    right = nullptr;
    back = nullptr;
    switch (from) {
  	case Dirs::Front:
  		front = fromCell;
  		break;
  	case Dirs::Left:
  		left = fromCell;
  		break;
  	case Dirs::Right:
  		right = fromCell;
  		break;
  	case Dirs::Back:
  		back = fromCell;
  		break;
  	}
  }

	//Cambia el puntero de la dirección dada a otro dado
	void change(Dirs dir, Cell * cell) {
    switch (dir) {
    case Dirs::Front:
      front = cell;
      break;
    case Dirs::Left:
      left = cell;
      break;
    case Dirs::Right:
      right = cell;
      break;
    case Dirs::Back:
      back = cell;
      break;
    }
  }

	//Devuelve la dirección del puntero según la dirección dada
	Cell * point(Dirs dir) {
    switch (dir) {
    case Dirs::Front:
      return front;
      break;
    case Dirs::Left:
      return left;
      break;
    case Dirs::Right:
      return right;
      break;
    case Dirs::Back:
      return back;
      break;
    }
    return 0;
  }
	//Devuelve el número de punteros no nulos
	int connections() {
  	int n = 0;
  	n += (right != nullptr) ? 1 : 0;
  	n += (left != nullptr) ? 1 : 0;
  	n += (front != nullptr) ? 1 : 0;
  	n += (back != nullptr) ? 1 : 0;
  	return n;
  }

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


class MazeIterator {
public:

	//Nuevo iterador, celda seleccionada, direccion seleccionada
	MazeIterator(Cell * cell, Dirs head = Dirs::Front) {
    heading = head;
    now = cell;
  }

	//Recorre el laberinto segun la regla especificada
	Dirs movePriority(bool rightWise) {
    if (rightWise) {
      switch (heading) {
      case Dirs::Front:
        if (now->right != nullptr) {
          heading = Dirs::Right;
        } else if (now->front != nullptr) {
          heading = Dirs::Front;
        } else if (now->left != nullptr) {
          heading = Dirs::Left;
        } else {
          heading = Dirs::Back;
        }
        break;
      case Dirs::Left:
        if (now->front != nullptr) {
          heading = Dirs::Front;
        } else if (now->left != nullptr) {
          heading = Dirs::Left;
        } else if (now->back != nullptr) {
          heading = Dirs::Back;
        } else {
          heading = Dirs::Right;
        }
        break;
      case Dirs::Right:
        if (now->back != nullptr) {
          heading = Dirs::Back;
        } else if (now->right != nullptr) {
          heading = Dirs::Right;
        } else if (now->front != nullptr) {
          heading = Dirs::Front;
        } else {
          heading = Dirs::Left;
        }
        break;
      case Dirs::Back:
        if (now->left != 0) {
          heading = Dirs::Left;
        } else if (now->back != 0) {
          heading = Dirs::Back;
        } else if (now->right != 0) {
          heading = Dirs::Right;
        } else {
          heading = Dirs::Front;
        }
      }
    } else {
      switch (heading) {
      case Dirs::Front:
        if (now->left != nullptr) {
          heading = Dirs::Left;
        } else if (now->front != nullptr) {
          heading = Dirs::Front;
        } else if (now->right != nullptr) {
          heading = Dirs::Right;
        } else {
          heading = Dirs::Back;
        }
        break;
      case Dirs::Left:
        if (now->back != nullptr) {
          heading = Dirs::Back;
        } else if (now->left != nullptr) {
          heading = Dirs::Left;
        } else if (now->front != nullptr) {
          heading = Dirs::Front;
        } else {
          heading = Dirs::Right;
        }
        break;
      case Dirs::Right:
        if (now->front != nullptr) {
          heading = Dirs::Front;
        } else if (now->right != nullptr) {
          heading = Dirs::Right;
        } else if (now->back != nullptr) {
          heading = Dirs::Back;
        } else {
          heading = Dirs::Left;
        }
        break;
      case Dirs::Back:
        if (now->right != nullptr) {
          heading = Dirs::Right;
        } else if (now->back != nullptr) {
          heading = Dirs::Back;
        } else if (now->left != nullptr) {
          heading = Dirs::Left;
        } else {
          heading = Dirs::Front;
        }
      }
    }
    now = now->point(heading);
    return relative(heading);
  }

	//Moviento manual, si no hay celda, la crea;
	void move(Dirs dir, Type type) {
    switch (dir) {
    case Dirs::Right:
      ckWise();
      break;
    case Dirs::Left:
      cntWise();
      break;
    case Dirs::Back:
      reverse();
      break;
    case Dirs::Front:
      break;
    }
    if (now->point(heading) == nullptr) {
      now->change(heading, new Cell(::reverse(heading), now, type));
    }
    now = now->point(heading);
  }

	//+180º a heading
	void reverse(void) {
    heading = ::reverse(heading);
  }

	//-90º a heading
	void ckWise(void) {
    heading = ::ckWise(heading);
  }

	//+90º a heading
	void cntWise(void) {
    heading = ::cntCkWise(heading);
  }

	//Devuelve numero de conexiones en la celda actual
	int connections() {
    return now->connections();
  }

	//Se mueve segun prioridad y elimina a su paso
	void deleteAndMove(bool rightWise) {
    Cell * aux = now;
    movePriority(rightWise);
    delete aux;
  }

	//Usa heading y la dirección dada para devolver, la
	//dirección relativa
	Dirs relative(Dirs p) {
    switch (heading) {
    case Dirs::Front:
      return p;
    case Dirs::Right:
      return ::cntCkWise(p);
    case Dirs::Back:
      return ::reverse(p);
    case Dirs::Left:
      return ::ckWise(p);
    }
    return Dirs::Front;
  }

	//Devuelve el id de la celda actual
	int id() {
  	return now->id;
  }

	//Devuelve el tipo de celda actual
	Type type() {
  	return now->type;
  }

	//Devuelve la dirección
	Dirs head() {
  	return heading;
  }

	//Devuelve la celda actual
	Cell * actual() {
  	return now;
  }

private:
	//A que celda se apunta en esa dirección
	Cell * point(Dirs dir);

	//Celda actual
	Cell * now;

	//Dirección del iterador
	Dirs heading;
};


//Clase Maze, administra las celdas
class Maze {
public:
  Maze() {
    count = 0;
    first  = new Cell(Dirs::Back, 0, Type::ENTRY);
  }

	//Devuelve un arraylist con las direcciones para salir
	//lo más directo posible del laberinto
	Queue<Dirs> solve() {
    Queue<Dirs> dirs;
    Queue<int> ids;
    MazeIterator itr(getStart());
    ids.pushBack(itr.id());

    while(itr.type() != Type::EXIT) {
      itr.movePriority(true);
      ids.pushBack(itr.id());
      if (itr.connections() > 2 && ids.has(itr.id())) {
        Serial.println("DELETING");
        itr.reverse();
        autoFree(itr);
        Serial.println("END DELETE");
      }
    }

    itr.reverse();
    itr.movePriority(true);
    Queue<int> ids2;

    while (itr.type() != Type::ENTRY) {
      itr.movePriority(true);
      ids2.pushBack(itr.id());
      if (itr.connections() > 2 && ids.has(itr.id())) {
        Serial.println("DELETING");
        itr.reverse();
        autoFree(itr);
        Serial.println("END DELETE");
      }
    }

    itr.reverse();

    for (int i = 0; itr.type() != Type::EXIT; i++) {
  		dirs.pushBack(itr.movePriority(true));
  	}

  	return dirs;
  }

	//Devuelve la dirección de la primera celda
	Cell * getStart() {
    return first;
  }

	//Contador de celdas
	int count;
private:
	//Primera Celda
	Cell * first;

	//Elimina una rama en la direccion señalada
	void autoFree(MazeIterator itr) {
    while (itr.connections() == 2) {
      itr.movePriority(true);
    }
    while (itr.connections() < 3 && itr.type() == Type::NORMAL) {
      itr.deleteAndMove(true);
    }
  }

};
