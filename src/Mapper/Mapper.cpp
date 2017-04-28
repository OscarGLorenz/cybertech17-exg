#include "Mapper.h"

Dirs clockWise(Dirs dir) {
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

Dirs cntClockWise(Dirs dir) {
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

Dirs reverseD(Dirs dir) {
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

Cell::Cell(Dirs from, Cell * fromCell, Type t, int index) {
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
    type = t;
    id = index;
}

//Cambia el puntero de la dirección dada a otro dado
void Cell::change(Dirs dir, Cell * cell) {
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
Cell * Cell::point(Dirs dir) {
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
int Cell::connections() {
    int n = 0;
    n += (right != nullptr) ? 1 : 0;
    n += (left != nullptr) ? 1 : 0;
    n += (front != nullptr) ? 1 : 0;
    n += (back != nullptr) ? 1 : 0;
    return n;
}

//Nuevo iterador, celda seleccionada, direccion seleccionada
MazeIterator::MazeIterator(Maze * m, Dirs head) {
    heading = head;
    maze = m;
    now = m->getStart();
}

Maze * MazeIterator::getMaze() {
	return maze;
}

MazeIterator::MazeIterator(MazeIterator &itr) {
	heading = itr.head();
    maze = itr.getMaze();
    now = itr.actual();
}

//Recorre el laberinto segun la regla especificada
Dirs MazeIterator::movePriority(bool rightWise) {
    Dirs preHeading = heading;
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
    return relative(heading,preHeading);
}

//Moviento manual, si no hay celda, la crea;
void MazeIterator::move(Dirs dir, Type type) {
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
        now->change(heading, new Cell(reverseD(heading), now, type, ++(maze->count)));
    }
    now = now->point(heading);
}

//+180º a heading
void MazeIterator::reverse(void) { heading = reverseD(heading); }

//-90º a heading
void MazeIterator::ckWise(void) {
    heading = clockWise(heading);
}

//+90º a heading
void MazeIterator::cntWise(void) {
    heading = cntClockWise(heading);
}

//Devuelve numero de conexiones en la celda actual
int MazeIterator::connections() {
    return now->connections();
}

//Se mueve segun prioridad y elimina a su paso
void MazeIterator::deleteAndMove(bool rightWise) {
    Cell * aux = now;
    movePriority(rightWise);

    if (aux == actual()->right)
		actual()->change(Dirs::Right, nullptr);
	if (aux == actual()->front)
		actual()->change(Dirs::Front, nullptr);
	if (aux == actual()->left)
		actual()->change(Dirs::Left, nullptr);
	if (aux == actual()->back)
		actual()->change(Dirs::Back, nullptr);

    delete aux;
}

//Usa heading y la dirección dada para devolver, la
//dirección relativa
Dirs MazeIterator::relative(Dirs after, Dirs before) {
    if (after == before)
        return Dirs::Front;
    if (after == clockWise(before))
        return Dirs::Right;
    if (after == cntClockWise(before))
        return Dirs::Left;
    if (after == reverseD(before))
        return Dirs::Back;
    return Dirs::Front;
}

//Devuelve el id de la celda actual
int MazeIterator::id() {
    return now->id;
}

//Devuelve el tipo de celda actual
Type MazeIterator::type() {
    return now->type;
}

//Devuelve la dirección
Dirs MazeIterator::head() {
    return heading;
}

//Devuelve la celda actual
Cell * MazeIterator::actual() {
    return now;
}

Maze::Maze() {
    count = 0;
    first  = new Cell(Dirs::Back, 0, Type::ENTRY, 0);
}

//Devuelve la dirección de la primera celda
Cell * Maze::getStart() {
    return first;
}


//Elimina una rama en la direccion señalada
void Maze::autoFree(MazeIterator itr) {
    while (itr.connections() == 2) {
        itr.movePriority(true);
    }
    while (itr.connections() < 3) {
        itr.deleteAndMove(true);
    }
}

//Copiar laberinto
Maze::Maze(Maze &maze) : Maze() {
    MazeIterator copy(&maze);
    MazeIterator paste(this);
    do {
        Dirs dir = copy.movePriority(true);
        paste.move(dir,copy.actual()->type);
    } while (copy.actual()->type != Type::ENTRY);
}

Queue<Movement> Maze::solve(Type start, Type goal) {
	Maze copyMaze(*this);

	MazeIterator killer(&copyMaze);

	Dirs dir; Type type;
	 if (start != Type::ENTRY) {
	 	do {
	 		type = killer.actual()->type;
	 	} while (type != start);
	 }

	 MazeIterator solver(killer);

	do {
		killer.movePriority(true);
	  type = killer.actual()->type;
	  if (type != goal && killer.actual()->connections() == 1) {
	 		while (killer.connections() == 1)	killer.deleteAndMove(true);
	 	}
 } while (type != goal);

	Queue<Movement> moves;

	do {
		dir = solver.movePriority(true);
		type = solver.actual()->type;
		moves.pushBack(Movement(type,dir));
	} while (type != goal);

	return moves;
}

Queue<Movement> Maze::solver() {
    Queue<Movement> moves;

    moves.appendQueue(solve(Type::ENTRY,Type::CHECK1));
    //moves.appendQueue(solve(Type::CHECK1,Type::CHECK2));
    //moves.appendQueue(solve(Type::CHECK2,Type::CHECK3));
    //moves.appendQueue(solve(Type::CHECK3,Type::CHECK4));
    //moves.appendQueue(solve(Type::CHECK4,Type::CHECK5));

    return moves;
}
