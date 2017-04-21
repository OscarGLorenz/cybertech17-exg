/*
 * Pinout.h
 *
 *  Created on: 4 feb. 2017
 *      Author: oscar
 */

#ifndef PINOUT_H_
#define PINOUT_H_

#define BLUE_LED	1
#define BUTTON	0

#define PIN1_QTR	3
#define PIN2_QTR	4
#define PIN3_QTR	5
#define PIN4_QTR	7
#define PIN5_QTR	8
#define PIN6_QTR	12
#define PIN7_QTR	13
#define PIN8_QTR	A0

#define RIGHT_SHARP  A6
#define FRONT_SHARP  A7
#define LEFT_SHARP A2
#define BACK_SHARP  A3

#define INTERRUPT_GYRO	2

#define PWM1A	9
#define PWM1B	6
#define PWM2A	10
#define PWM2B	11

#define BAT_VOLTAGE A1

#define MIN_LINE (unsigned char []) {96,48,88,88,88,48,8,48};
#define MAX_LINE (unsigned char []) {2500,1488,1800,1940,960,840,8,1812};


#endif /* PINOUT_H_ */
