#include "MazeObjects.hpp"

template<class T> ArrayList<T>::ArrayList() {
	lenght = 0;
	array = 0;
}

template<class T> ArrayList<T>::~ArrayList() {
	free(array);
}

template<class T> void ArrayList<T>::add(T t) {
	lenght++;
	array = (T *) realloc(array, lenght * sizeof(T));
	array[lenght - 1] = t;
}

template<class T> T& ArrayList<T>::operator[](const int i) {
	if (i < 0) {
		return array[0];
	} else if (i > lenght - 1) {
		return array[lenght - 1];
	} else {
		return array[i];
	}
}

template<class T> const T& ArrayList<T>::operator[](const int i) const {
	if (i < 0) {
		return array[0];
	} else if (i > lenght - 1) {
		return array[lenght - 1];
	} else {
		return array[i];
	}
}

template<class T> bool ArrayList<T>::has(T t) {
	for (int i = 0; i < lenght; i++) {
		if (t == array[i]) {
			return true;
		}
	}
	return false;
}

template<class T> int ArrayList<T>::size() {
	return lenght;
}

template class ArrayList<int> ;
template class ArrayList<Dir> ;

Cell::Cell(Dir from, Cell * fromCell, Type type) {
	front = 0;
	left = 0;
	right = 0;
	back = 0;
	switch (from) {
	case FRONT:
		front = fromCell;
		break;
	case LEFT:
		left = fromCell;
		break;
	case RIGHT:
		right = fromCell;
		break;
	case BACK:
		back = fromCell;
		break;
	}
	this->id = Maze::count++;
	this->type = type;
}

void Cell::change(Dir dir, Cell * cell) {
	switch (dir) {
	case FRONT:
		front = cell;
		break;
	case LEFT:
		left = cell;
		break;
	case RIGHT:
		right = cell;
		break;
	case BACK:
		back = cell;
		break;
	}
}

Cell * Cell::point(Dir dir) {
	switch (dir) {
	case FRONT:
		return front;
		break;
	case LEFT:
		return left;
		break;
	case RIGHT:
		return right;
		break;
	case BACK:
		return back;
		break;
	}
	return 0;
}

int Cell::connections() {
	int n = 0;
	n += (right != 0) ? 1 : 0;
	n += (left != 0) ? 1 : 0;
	n += (front != 0) ? 1 : 0;
	n += (back != 0) ? 1 : 0;
	return n;
}

Iterator::Iterator() {
	heading = FRONT;
	now = Maze::getStart();
}

Iterator::Iterator(Dir head, Cell * cell) {
	heading = head;
	now = cell;
}

Dir Iterator::movePriority(bool rightWise) {
	if (rightWise) {
		switch (heading) {
		case FRONT:
			if (now->right != 0) {
				heading = RIGHT;
			} else if (now->front != 0) {
				heading = FRONT;
			} else if (now->left != 0) {
				heading = LEFT;
			} else {
				heading = BACK;
			}
			break;
		case LEFT:
			if (now->front != 0) {
				heading = FRONT;
			} else if (now->left != 0) {
				heading = LEFT;
			} else if (now->back != 0) {
				heading = BACK;
			} else {
				heading = RIGHT;
			}
			break;
		case RIGHT:
			if (now->back != 0) {
				heading = BACK;
			} else if (now->right != 0) {
				heading = RIGHT;
			} else if (now->front != 0) {
				heading = FRONT;
			} else {
				heading = LEFT;
			}
			break;
		case BACK:
			if (now->left != 0) {
				heading = LEFT;
			} else if (now->back != 0) {
				heading = BACK;
			} else if (now->right != 0) {
				heading = RIGHT;
			} else {
				heading = FRONT;
			}
		}
	} else {
		switch (heading) {
		case FRONT:
			if (now->left != 0) {
				heading = LEFT;
			} else if (now->front != 0) {
				heading = FRONT;
			} else if (now->right != 0) {
				heading = RIGHT;
			} else {
				heading = BACK;
			}
			break;
		case LEFT:
			if (now->back != 0) {
				heading = BACK;
			} else if (now->left != 0) {
				heading = LEFT;
			} else if (now->front != 0) {
				heading = FRONT;
			} else {
				heading = RIGHT;
			}
			break;
		case RIGHT:
			if (now->front != 0) {
				heading = FRONT;
			} else if (now->right != 0) {
				heading = RIGHT;
			} else if (now->back != 0) {
				heading = BACK;
			} else {
				heading = LEFT;
			}
			break;
		case BACK:
			if (now->right != 0) {
				heading = RIGHT;
			} else if (now->back != 0) {
				heading = BACK;
			} else if (now->left != 0) {
				heading = LEFT;
			} else {
				heading = FRONT;
			}
		}
	}
	now = now->point(heading);
	return relative(heading);
}

void Iterator::move(Dir dir, Type type) {
	switch (dir) {
	case RIGHT:
		heading = Maze::ckWise(heading);
		break;
	case LEFT:
		heading = Maze::cntCkWise(heading);
		break;
	case BACK:
		heading = Maze::reverse(heading);
		break;
	case FRONT:
		break;
	}
	if (now->point(heading) == 0) {
		now->change(heading, new Cell(Maze::reverse(heading), now, type));
	}
	now = now->point(heading);
}

void Iterator::reverse() {
	heading = Maze::reverse(heading);
}

void Iterator::ckWise() {
	heading = Maze::ckWise(heading);
}

void Iterator::cntWise() {
	heading = Maze::cntCkWise(heading);
}

int Iterator::connections() {
	return now->connections();
}

void Iterator::deleteAndMove(bool rightWise) {
	Cell * aux = now;
	movePriority(rightWise);
	delete aux;
}

Dir Iterator::relative(Dir p) {
	switch (heading) {
	case FRONT:
		return p;
	case RIGHT:
		return Maze::cntCkWise(p);
	case BACK:
		return Maze::reverse(p);
	case LEFT:
		return Maze::ckWise(p);
	}
	return FRONT;
}

int Iterator::id() {
	return now->id;
}

Type Iterator::type() {
	return now->type;
}

Dir Iterator::head() {
	return heading;
}

Cell * Iterator::actual() {
	return now;
}

int Maze::count = 0;
Cell * Maze::first = new Cell(BACK, 0, ENTRY);

ArrayList<Dir> Maze::solve() {
	ArrayList<Dir> dirs;
	ArrayList<int> ids;
	Iterator itr;
	ids.add(itr.id());
	while (itr.type() != EXIT) {
		itr.movePriority(true);
		ids.add(itr.id());
		if (itr.connections() > 2 && ids.has(itr.id())) {
			Serial.println("DELETING");
			itr.reverse();
			autoFree(itr);
			Serial.println("END DELETE");
		}
	}

	itr.reverse();
	itr.movePriority(true);
	ArrayList<int> ids2;

	while (itr.type() != ENTRY) {
		itr.movePriority(true);
		ids2.add(itr.id());
		if (itr.connections() > 2 && ids2.has(itr.id())) {
			Serial.println("DELETING");
			itr.reverse();
			autoFree(itr);
			Serial.println("END DELETE");
		}
	}

	itr.reverse();

	// if(first != itr.actual()) {
	// 	Serial.println("ISLAND DETECTED");
	// 	Iterator itrEntry;
	// 	while(itrEntry.actual() != itr.actual()) {
	// 		itrEntry.movePriority(true);
	// 		itr.movePriority(true);
	// 	}
	// 	itr.reverse();
	// 	autoFree(itr);
	// 	Serial.println("ISLAND DELETED, PATHS MERGED");
	// }

	for (int i = 0; itr.type() != EXIT; i++) {
		dirs.add(itr.movePriority(true));
	}

	ids2.~ArrayList();
	return dirs;
}

Cell * Maze::getStart() {
	return first;
}

void Maze::autoFree(Iterator itr) {
	while (itr.connections() == 2) {
		itr.movePriority(true);
	}
	while (itr.connections() < 3) {
		itr.deleteAndMove(true);
	}
}

Dir Maze::ckWise(Dir dir) {
	switch (dir) {
	case FRONT:
		return RIGHT;
	case LEFT:
		return FRONT;
	case RIGHT:
		return BACK;
	case BACK:
		return LEFT;
	}
	return FRONT;
}

Dir Maze::cntCkWise(Dir dir) {
	switch (dir) {
	case FRONT:
		return LEFT;
	case LEFT:
		return BACK;
	case RIGHT:
		return FRONT;
	case BACK:
		return RIGHT;
	}
	return FRONT;
}

Dir Maze::reverse(Dir dir) {
	switch (dir) {
	case FRONT:
		return BACK;
	case LEFT:
		return RIGHT;
	case RIGHT:
		return LEFT;
	case BACK:
		return FRONT;
	}
	return FRONT;
}
