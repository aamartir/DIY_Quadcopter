#ifndef _ESC_H_
#define _ESC_H

#include "Quad.h"
#include <avr/interrupt.h>

#define MOTORS 4
#define MOTOR_MIN 15000
#define MOTOR_MAX 28000 
#define QUAD_HOVER 20000
#define MOTOR_MAX_MINUS_MIN (MOTOR_MAX - MOTOR_MIN)

#define MOTORS_75_PERCENT (2*(MOTOR_MAX - MOTOR_MIN)/3 + MOTOR_MIN)
#define MOTORS_DEAD_ZONE 200

extern volatile unsigned short motorIndexA;
extern volatile unsigned short motorIndexB;

/* Clock ticks:   (15000)                  (30000)
 *                   __________________________
 *                   |           |            |
 *                   |           |            |
 *
 * Pulse width:     1ms         1.5ms        2ms
 */

extern volatile unsigned int motorPWM[MOTORS];

#endif
