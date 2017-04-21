/*
 * Debug.h
 *
 *  Created on: 19 sept. 2016
 *      Author: oscar
 *
 */

#ifndef SRC_DEBUG_H_
#define SRC_DEBUG_H_

#include "Arduino.h"

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

/*
 * Esta macro se usa como println pero escribe una tabulación en vez de salto
 * de línea
 */
#define printtab(x) print(x); Serial.print("\t");

//Variable global para contar tiempo
static unsigned long TIMING = millis();

/*
 * Con esta macro podemos limitar la salida por pantalla a un tiempo LIMIT
 */
#define limitedSerial(STR, LIMIT) { if (millis()- TIMING > LIMIT) { \
                                    TIMING = millis(); \
                                    Serial.println(STR); \
                                    }}\

#endif /* SRC_DEBUG_H_ */
