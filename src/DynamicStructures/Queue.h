/*
 * Queue.h
 *
 * Copyright 2017 oscar <oscar@oscar-HP>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */


#ifndef QUEUE_H
#define QUEUE_H

#include <stddef.h>
//Declaraciones para que no haya problemas de orden en el propio archivo,
//que no se use una cosa antes de que se haya declarado
template<class T> class Queue;
template<class T> class NodeQueue;
template<class T> class QueueIterator;

//Clase cola dinámica
template<class T> class Queue {
	//Clase amiga para que QueueIterator pueda acceder a first, first es privado
	friend class QueueIterator<T>;

private:
	//Primer nodo
	NodeQueue<T> * first;

	//Último nodo
	NodeQueue<T> * last;

	//Longitud
	size_t lenght;
public:
	//Crear cola
	Queue() {
		first = nullptr;
		last = nullptr;
		lenght = 0;
	}

	//Devuelve el tamaño
	size_t size(void) {
		return lenght;
	}

	//Añade un elemento al final
	void pushBack(T object) {
		if (first == nullptr) {
			//Si es el primero elemento en añadirse, el primer y último
			//elemento son el mismo
			last = new NodeQueue<T>(object);
			first = last;
		} else {
			//Si no, se enlaza con el último elemento el nuevo y
			//avanzamos a este nuevo
			last->_next = new NodeQueue<T>(object);
			last = last->_next;
		}
		//Incrementa longitud
		lenght++;
	}

	//Devuelve el elemento en la posición dada, DESACONSEJADO PARA ITERAR
	T get(size_t index) {
		//Obtiene un iterador
		QueueIterator<T> itr = getIterator();
		size_t i = 0;
		//Usa el iterador hasta que se haya iterado el mismo numero de veces
		//que el pedido y devuelve este valor
		while (itr.hasNext()) {
			if (i == index)
				return itr.next();
			else
				itr.next();
			i++;
		}
		return itr.next();
	}

	bool has(T object) {
		//Obtiene un iterador
		QueueIterator<T> itr = getIterator();

		//Usa el iterador hasta que se haya iterado el mismo numero de veces
		//que el pedido y devuelve este valor
		while (itr.hasNext()) {
			if (itr.next() == object)
				return true;
		}
		return false;
	}

	//Devuelve un iterador
	QueueIterator<T> getIterator() {
		return QueueIterator<T>(this);
	}

	void add(T * array, size_t dim) {
		for (size_t i = 0; i < dim; i++) {
			pushBack(array[i]);
		}
	}
};

//Nodo, no se usa directamente
template<class T> class NodeQueue {
public:
	//Por defecto creas un nodo sin siguiente
	NodeQueue(T data, NodeQueue<T> * next = nullptr) {
		_data = data;
		_next = next;
	}

	//Valor
	T _data;

	//Siguiente nodo
	NodeQueue<T> * _next;
};

//Clase para iterar eficientemente se obtiene con getIterator,
//se usa con la estructura while(ITR.hasNext()) { A = ITR.next();}
template<class T> class QueueIterator {
private:
	//Nodo actual
	NodeQueue<T>  * node;

	//Se ha usado por primera vez o no
	bool first;
public:
	//Crea un iterador
	QueueIterator(Queue<T> * queue) {
		node = queue->first;
		first = true;
	}

	//Mira si hay siguiente nodo, si es la primera vez mira el primero
	bool hasNext() {
		if (first) {
			return node != nullptr;
		} else
			return node->_next != nullptr;
	}

	//Devuelve el valor del siguiente nodo y avanza una posición
	//En caso de ser la primera vez devuelve el primer nodo
	T next() {
		if (first) {
			first = false;
		} else {
			if(node->_next != nullptr);
			node = node->_next;
		}
		return node->_data;
	}
};

/* EJEMPLO
    //Cola de enteros
	Queue<int> queue;

	//Añadir números
	queue.pushBack(4);
	queue.pushBack(8);
	queue.pushBack(9);

	//Método típico para iterar, ineficiente
	for (size_t i = 0; i < queue.size(); i++) {
		Serial,println(queue.get(i));
	}

	//Crear iterador, método eficiente para iterar
	QueueIterator<int> itr = queue.getIterator();

	while (itr.hasNext()) {
		Serial,println(itr.next());
	}
*/

#endif
