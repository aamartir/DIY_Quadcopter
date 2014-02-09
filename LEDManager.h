#ifndef _LED_MANAGER_H
#define _LED_MANAGER_H

#include "Quad.h"

/* State definitions */
#define LED_OFF          0
#define LED_ON           1

/* Pattern properties (MASK) */
#define NO_PATTERN       0
#define INFINITE_PATTERN 1
#define REPEAT_PATTERN   2
#define ONCE_PATTERN     4

typedef struct _led_state
{
  uint16 value:1; /* ON/OFF */
  uint16 duration:15; /* In milliseconds */
} LED_STATE;

typedef struct _led_pattern
{
  LED_STATE *pattern;
  uint8 length;   /* How many LED_STATE members in pattern array */
  uint8 property; /* INFINITE/REPEAT/ONCE */
  uint8 repeatCount;
} LED_PATTERN;

/* Patterns */
extern LED_PATTERN simple_on_off_pattern;
extern LED_PATTERN SOS_pattern;
extern LED_PATTERN _1_2_3_pattern;
extern LED_PATTERN low_batt_pattern;

class LEDManager
{
  private:
    LED_PATTERN *thisPattern;
    uint8 stateCount;
    uint8 repeatCount;
    uint8 led_pin;	
  public:
    LEDManager();
    LEDManager(LED_PATTERN pattern, uint8 led_pin);
    ~LEDManager();
	
    LED_STATE getCurrentLEDState(void);
    //void setCurrentLEDStateArray(LEDState *stateArray, uint8 length, uint8 property, uint8 repeats);
    LED_PATTERN *getCurrentPattern(void);
    void setCurrentPattern(LED_PATTERN *pattern, uint8 led_pin);
    void setCurrentPattern(LED_PATTERN *pattern);

    int8 getCurrentLEDStateValue(void);
    int16 getCurrentLEDStateTime(void);
    
    uint8 getLEDpin(void);
    void setLEDpin(uint8 pin);
    
    uint8 patternRepeats(void);
    void setRepeat(uint8 count);
    void resetPattern(void);
    uint8 hasNextStateInPattern(void);
    void nextLEDStateInPattern(void);
	
	//void addLEDStateToPattern(LED_STATE newState);
};

extern LEDManager led_manager;

#endif _LED_MANAGER_H
