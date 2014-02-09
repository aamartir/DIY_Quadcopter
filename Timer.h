#ifndef _TIMER_H_
#define _TIMER_H_

#include "Quad.h"

typedef void (*FPTR)(void);

enum
{
  LED_TIMER,
  RADIO_TIMER,
  SPEAKER_TIMER,
  MOTOR_TIMER,
  DEBUG_TIMER,
  
  MAX_TIMERS
};

#define LED_TIMER_PERIOD     1    /* milliseconds */
#define RADIO_TIMER_PERIOD   1000 /* milliseconds */

/* Timer object creates events that execute (interrupt based) every so often */
class Timer
{
private:
  uint8 id;
public:
  Timer();
  Timer(FPTR fptr);

  void setTimerVal(uint16 val);
  void clearTimer(void);
  uint16 getTimerValue(void);

  /* Public access variables */
  volatile uint16 val;
  FPTR fptr;
  static uint8 timersActive;
};

extern Timer led_timer;
extern Timer radio_timer;
extern Timer speaker_timer;
extern Timer motor_timer;
extern Timer debug_timer;

extern void timer_routine();
extern void led_timer_handler(void);
extern void radio_timer_handler(void);
extern void speaker_timer_handler(void);
extern void motor_timer_handler(void);
extern void debug_timer_handler(void);

extern Timer *Timers[];
extern uint16 TIMER_ISR_LAST_MS;

#endif




