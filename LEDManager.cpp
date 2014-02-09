#include "LEDManager.h"
#include "Timer.h"

LEDManager::LEDManager()
{
  /* Allocate memory for PATTERN structure */
  thisPattern = (LED_PATTERN *) malloc(sizeof( LED_PATTERN ));
  
  /* Initialize*/
  thisPattern->pattern     = 0;
  thisPattern->length      = 0;
  thisPattern->property    = NO_PATTERN;
  thisPattern->repeatCount = 0;
  
  /* Initialize class variables */
  stateCount = 0;
  repeatCount = 0;
  led_pin = BLUE_LED_PIN; /* By default if not pin is given */
}

LEDManager::LEDManager(LED_PATTERN pattern, uint8 led_gpio)
{
  thisPattern = &pattern;
  
  stateCount  = 0;
  repeatCount = 0;
  led_pin = led_gpio;
}

LEDManager::~LEDManager()
{
  if(thisPattern)
  {
    free(thisPattern->pattern); /* Free LED_STATE array */
    free(thisPattern);          /* Free pattern */
  }
}

/* State machine functions */
void LEDManager::nextLEDStateInPattern()
{
  stateCount = ++stateCount % (thisPattern->length);
}

uint8 LEDManager::hasNextStateInPattern()
{
  return (stateCount < (thisPattern->length - 1));
}

LED_STATE LEDManager::getCurrentLEDState(void)
{
  return thisPattern->pattern[stateCount];
}

/*void LEDManager::setCurrentLEDStateArray(LEDState *stateArray, uint8 length, uint8 property, uint8 repeats);
{
  if(thisPattern)
  {
    
  }
}*/

LED_PATTERN *LEDManager::getCurrentPattern(void)
{
  return thisPattern;
}

void LEDManager::setCurrentPattern(LED_PATTERN *thisPatt, uint8 led_gpio)
{ 
  /* Point to the new memory location */
  if(thisPatt)
  {
    thisPattern = thisPatt;
    stateCount = 0;
    repeatCount = 0;
    led_pin = led_gpio;
    
    led_timer.setTimerVal( LED_TIMER_PERIOD ); /* Will begin pattern execution in the next cycle ( 1ms ) */
  }
}

void LEDManager::setCurrentPattern(LED_PATTERN *thisPatt)
{ 
  setCurrentPattern(thisPatt, led_pin);
}

void LEDManager::resetPattern()
{
  if(thisPattern)
  {
    stateCount = 0;
    repeatCount = 0;
	
    /* Reset timer */
    led_timer.setTimerVal( LED_TIMER_PERIOD );
  }
}

void LEDManager::setRepeat(uint8 count)
{
   if(thisPattern)
   {
     thisPattern->property = REPEAT_PATTERN;
     thisPattern->repeatCount = count;
   }
 }
 
int8 LEDManager::getCurrentLEDStateValue(void)
{
  if(thisPattern)
    return (thisPattern->pattern[stateCount].value);
	
  return -1;
}

int16 LEDManager::getCurrentLEDStateTime(void)
{
  if(thisPattern)
    return thisPattern->pattern[stateCount].duration;
  	
  return -1;
}

uint8 LEDManager::getLEDpin(void)
{
  return led_pin;
}

void LEDManager::setLEDpin(uint8 pin)
{
  led_pin = pin;
}
    
uint8 LEDManager::patternRepeats(void)
{
  if(thisPattern)
  {
    if(thisPattern->property == INFINITE_PATTERN)
      return true;
    else if(thisPattern->property == REPEAT_PATTERN && repeatCount < thisPattern->repeatCount)
      return true;
  }
  
  return false;
}

/* ------------------------------- */
/*           State Arrays          */
/* ------------------------------- */
const LED_STATE simple_on_off_states[] = 
{
  { LED_ON, 500 },
  { LED_OFF, 500 }
};

const LED_STATE SOS_states[] = 
{
  /* Fast blink */
  { LED_ON,  100 },
  { LED_OFF, 100 },
  { LED_ON,  100 },
  { LED_OFF, 100 },
  { LED_ON,  100 },
  { LED_OFF, 100 },
  /* Slow blink */
  { LED_ON,  500 },
  { LED_OFF, 500 },
  { LED_ON,  500 },
  { LED_OFF, 500 },
  { LED_ON,  500 },
  { LED_OFF, 500 },
  /* Fast blink */
  { LED_ON,  100 },
  { LED_OFF, 100 },
  { LED_ON,  100 },
  { LED_OFF, 100 },
  { LED_ON,  100 },
  { LED_OFF, 100 },
  /* Pause */
  { LED_OFF, 1000 }
};

const LED_STATE _1_2_3_states[] =
{
  /* Blink once */
  { LED_ON,  100 },
  { LED_OFF, 500 },
  /* Blink twice */
  { LED_ON,  100 },
  { LED_OFF, 100 },
  { LED_ON,  100 },
  { LED_OFF, 500 },
  /* Blink 3 times */
  { LED_ON,  100 },
  { LED_OFF, 100 },
  { LED_ON,  100 },
  { LED_OFF, 100 },
  { LED_ON,  100 },
  { LED_OFF, 1000 },
};

const LED_STATE low_batt_states[] = 
{
  { LED_ON, 100 },
  { LED_OFF, 3000 }
};

/* ------------------------------- */
/*           Patterns              */
/* ------------------------------- */
LED_PATTERN simple_on_off_pattern =
{
  (LED_STATE *) simple_on_off_states,
  2,
  INFINITE_PATTERN,
  0
};

LED_PATTERN low_batt_pattern = 
{
  (LED_STATE *) low_batt_states,
  2,
  INFINITE_PATTERN,
  0
};

LED_PATTERN SOS_pattern = 
{
  (LED_STATE *) &SOS_states,
  19,
  INFINITE_PATTERN,
  0
};

LED_PATTERN _1_2_3_pattern = 
{
  (LED_STATE *) &_1_2_3_states,
  12,
  INFINITE_PATTERN,
  0
};
