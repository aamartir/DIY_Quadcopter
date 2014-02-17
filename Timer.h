#ifndef _TIMER_H_
#define _TIMER_H_

#include "Quad.h"

typedef void (*FPTR)(void);

enum
{
  LED_TIMER,
  RADIO_TIMER,
  MOTOR_TIMER,
  
  MAX_TIMERS
};

/* milliseconds */
#define LED_TIMER_PERIOD     1   
#define RADIO_TIMER_PERIOD   100 
#define MOTOR_TIMER_PERIOD   20

/* Timer object creates events that execute (interrupt based) every so often */
class Timer
{
private:
  uint8_t id;
public:
  Timer();
  Timer(FPTR fptr);

  void setTimerVal(uint16_t val);
  void clearTimer(void);
  uint16_t getTimerValue(void);

  /* Public access variables */
  volatile uint16_t val;
  FPTR fptr;
  static uint8_t timersActive;
};

extern Timer led_timer;
extern Timer radio_timer;
extern Timer motor_timer;

extern void timer_routine();
extern void led_timer_handler(void);
extern void radio_timer_handler(void);
extern void motor_timer_handler(void);

extern Timer *Timers[];
extern uint16_t TIMER_ISR_LAST_MS;

#endif




