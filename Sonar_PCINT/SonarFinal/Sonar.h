#ifndef _SONAR_H
#define _SONAR_H

#include "AB_Filter.h"

#define SONAR_PIN         4
#define SONAR_PIN_MASK    (1 << SONAR_PIN)

/* Sonar states */
#define SONAR_INACTIVE    0
#define SONAR_ACTIVE      1
#define SONAR_PULSE_START 2
#define SONAR_DATA_AVAIL  3

/* Sonar distance type */
#define DISTANCE_CM       0
#define DISTANCE_IN       1
#define DISTANCE_MTS      2

//#define LOW_ALTITUDE_MODE

#ifdef PARALLAX_SONAR
  #define MIN_SONAR_DISTANCE 0
  #define MAX_SONAR_DISTANCE 300 /* cm */
  #define SONAR_K           (1.0f/58.0f)
#else
  #define MIN_SONAR_DISTANCE 14
  
  #ifdef LOW_ALTITUDE_MODE
    #define MAX_SONAR_DISTANCE 200 /* cm */
  #else
    #define MAX_SONAR_DISTANCE 645
  #endif
  
  /* Units of measurement */
  #define US_PER_INCH        147.0f
  #define CM_PER_US          (2.54f/US_PER_INCH)     
  #define IN_PER_US          (1.0f/US_PER_INCH)
  #define MTS_PER_US         (CM_PER_US/100.0f)
#endif

//#define SMOOTH_SONAR_DATA 1
#ifdef SMOOTH_SONAR_DATA
  #define SMOOTH_FACTOR 0.90
#endif

#ifndef SMOOTH_SONAR_DATA
 #define MOVING_AVG_FILTER 1
 #ifdef MOVING_AVG_FILTER
   #define MOVING_AVG_SAMPLES 8
 #endif
#endif


/* Class definition */
class Sonar
{
  private:
    volatile char sonar_state;
    volatile unsigned int start_time;
    volatile unsigned int echo_time;
    
    float altitude;
    float last_altitude;
    float rate_of_change;  /* Derivative of altitude with respect to time */
    AB_Filter ab_filter;   /* Alfa-Beta filter to smooth out the rate_of_change of the altitude */
    
  public:
    Sonar();
    void sonarInit(void);
    
    void setStartTime(unsigned int val);
    void setEchoTime(unsigned int val);
    
    unsigned int getEchoTime(void);
    float getSonarAltitude( float dt, float coef );
    float getSonarAltitude_filtered( void );
    float getSonarROF( void ); /* getSonarAltitude upates rate of change function */
    void setSonarState(char newState);
    char getSonarState(void);
    char isSonarDataAvailable(void);
    
    char isActive(void);
};

extern Sonar sonar;

#endif
