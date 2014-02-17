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
  uint16_t value:1; /* ON/OFF */
  uint16_t duration:15; /* In milliseconds */
} LED_STATE;

typedef struct _led_pattern
{
  LED_STATE *pattern;
  uint8_t length;   /* How many LED_STATE members in pattern array */
  uint8_t property; /* INFINITE/REPEAT/ONCE */
  uint8_t repeatCount;
} LED_PATTERN;

/* Patterns */
extern LED_PATTERN simple_on_off_pattern;
extern LED_PATTERN SOS_pattern;
extern LED_PATTERN _1_2_3_pattern;
extern LED_PATTERN low_batt_pattern;
extern LED_PATTERN motors_disarmed_pattern;

class LEDManager
{
  private:
    LED_PATTERN *thisPattern;
    uint8_t stateCount;
    uint8_t repeatCount;
    uint8_t led_pin;	
  public:
    LEDManager();
    LEDManager(LED_PATTERN pattern, uint8_t led_pin);
    ~LEDManager();
	
    LED_STATE getCurrentLEDState(void);
    //void setCurrentLEDStateArray(LEDState *stateArray, uint8_t length, uint8_t property, uint8_t repeats);
    LED_PATTERN *getCurrentPattern(void);
    void setCurrentPattern(LED_PATTERN *pattern, uint8_t led_pin);
    void setCurrentPattern(LED_PATTERN *pattern);

    int8_t getCurrentLEDStateValue(void);
    int16_t getCurrentLEDStateTime(void);
    
    uint8_t getLEDpin(void);
    void setLEDpin(uint8_t pin);
    
    uint8_t patternRepeats(void);
    void setRepeat(uint8_t count);
    void resetPattern(void);
    uint8_t hasNextStateInPattern(void);
    void nextLEDStateInPattern(void);
	
	//void addLEDStateToPattern(LED_STATE newState);
};

extern LEDManager led_manager;

#endif _LED_MANAGER_H
