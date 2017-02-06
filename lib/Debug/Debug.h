/*
 * Debug.h
 *
 *  Created on: 19 sept. 2016
 *      Author: oscar
 *	
 */

#ifndef SRC_DEBUG_H_
#define SRC_DEBUG_H_



/* DEBUG es una macro que muestra el valor y nombre de algún valor o variable
 * por Serial
 * Ejemplo:
 *   int x = 45;
 *   DEBUG(x)
 *   Por serial pondría "x = 45"
*/
#define DEBUG(x) Serial.print(#x); Serial.print(" = "); Serial.println(x);

/*
 *  BRK es una macro que imprime por Serial la linea en la que se encuentra
 *  muy útil para hacer debug y ver si el programa llega o no a cierto punto
 *  Ejemplo:
 *    1. if(true)
 *    2.   BRK
 *    3. else
 *    4.   BRK
 *  Por serial solo pondría "Code reached line No.2"
 */
#define BRK Serial.print("Reached Line #"); Serial.println(__LINE__);




#endif /* SRC_DEBUG_H_ */