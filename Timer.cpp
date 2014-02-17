#include "Timer.h"
#include "Quad.h"
#include "ESC.h"
#include "LEDManager.h"
#include "Radio_PCINT.h"

#include <stdio.h>

Timer *Timers[] = 
{
  &led_timer, 
  &radio_timer,
  &motor_timer,
};

Timer::Timer()
{
  fptr = NULL;
}

Timer::Timer(FPTR ptr)
{
  fptr = ptr;
}

void Timer::setTimerVal(uint16_t newVal)
{
  val = newVal;
}

void Timer::clearTimer(void)
{
  val = 0;
}

uint16_t Timer::getTimerValue(void)
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

/* Handles low-level radio signal processing.
   Higher level functions are handled within main-loop */
void radio_timer_handler(void)
{
  /* When radio_timer expires, we have to check 
   * if we have received a new radio frame. If
   * no frame has been received, land quad. If
   * GPS present, we can activate Go Home feature
   * and then land.  */
  
  /* State machine */
  if( (radio.state & RADIO_SIGNAL_LOST) == 0 )
  {
    /* Here, we configure autoland mode, or return to HOME 
     * Need GPS for this. For now, just autoland. Decrease motor speeds
     * slowly. We can use the accelerometer's z axis and integrate it
     * to get some feedback on the speed of descent.
     */ 
     
    gpio_set( RED_LED_PIN );
    SET_BIT( radio.state, RADIO_SIGNAL_LOST );
  }
}

/* Do not have any altitude feedback as of now. Will have to decrease throttle slowly. */
void motor_timer_handler()
{
  if( quad.quad_mode == QUAD_AUTOLAND_MODE )
  {
    #ifdef USE_SONAR
      // ...
    #else
      if( throttle > MOTOR_MIN )
      {
        throttle -= 5;
        
        /* Reset timer */
        motor_timer.setTimerVal( MOTOR_TIMER_PERIOD );
      }
    #endif
  }
  else if( quad.quad_mode == QUAD_AUTO_TAKEOFF_MODE )
  {
    
  }
}
