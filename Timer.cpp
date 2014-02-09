#include "Timer.h"
#include "Quad.h"
#include "LEDManager.h"
#include "Radio_PCINT.h"

#include <stdio.h>

uint8 debug_timer_state;

Timer *Timers[] = 
{
  &led_timer, 
  &radio_timer,
  &speaker_timer, 
  &motor_timer,
  &debug_timer
};

Timer::Timer()
{
  fptr = NULL;
}

Timer::Timer(FPTR ptr)
{
  fptr = ptr;
}

void Timer::setTimerVal(uint16 newVal)
{
  val = newVal;
}

void Timer::clearTimer(void)
{
  val = 0;
}

uint16 Timer::getTimerValue(void)
{
  return val; 
}

void led_timer_handler(void)
{
  /* Execute the current value in the pattern */
  gpio_value(led_manager.getCurrentLEDStateValue(), led_manager.getLEDpin());
  
  if(led_manager.hasNextStateInPattern() || led_manager.patternRepeats())
  {
    led_timer.setTimerVal(led_manager.getCurrentLEDStateTime());
    led_manager.nextLEDStateInPattern(); /* Go to next state in pattern or roll-over */
  }
}

/* Under Construction */
void radio_timer_handler(void)
{
  /* When radio_timer expires, we have to check 
   * if we have received a new radio frame. If
   * no frame has been received, land quad. If
   * GPS present, we can activate Go Home feature
   * and then land.  */
  
  /* State machine */
  switch( radio.getState() )
  {
    case RADIO_SIGNAL_LOST:
      /* Here, we configure autoland mode, or return to HOME 
       * Need GPS for this. For now, just autoland. Decrease motor speeds
       * slowly. We can use the accelerometer's z axis and integrate it
       * to get some feedback on the speed of descent.
       */
      
      break;
      
    default:
      radio.setState( RADIO_SIGNAL_LOST );
  }
  
  /* Reset timer */
  radio_timer.setTimerVal( RADIO_TIMER_PERIOD );
}

void debug_timer_handler(void)
{
  if(debug_timer_state)
    gpio_clear(BLUE_LED_PIN);
  else
    gpio_set(BLUE_LED_PIN);
  
  debug_timer_state = !debug_timer_state;
  //debug_timer.setTimerVal( 1 );
}

/* TODO */
void speaker_timer_handler()
{
  
}

void motor_timer_handler()
{
  
}
