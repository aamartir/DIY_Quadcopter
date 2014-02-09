#ifndef _SONAR_H
#define _SONAR_H

#include "Quad.h"
#include "AB_Filter.h"

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
  #define MAX_SONAR_DISTANCE 118.0f /* inches */
  
  #define US_PER_INCH_RES    58.0f
  
  #define IN_UNIT            (1.0f/US_PER_INCH_RES)
  #define CM_UNIT            (2.54f/US_PER_INCH_RES)  
  #define MTS_UNIT           (CM_UNIT / 100.0f)  
#else
  #define MIN_SONAR_DISTANCE 5.6f /* Minimum distance the sensor is able to detect (In inches) */
  
  #ifdef LOW_ALTITUDE_MODE
    #define MAX_SONAR_DISTANCE 80.0f   /* In inches */
  #else
    #define MAX_SONAR_DISTANCE 253.0f  /* In inches */
  #endif
  
  #define US_PER_INCH_RES    147.0f
  #define IN_UNIT            (1.0f/US_PER_INCH_RES)
  #define CM_UNIT            (2.54f/US_PER_INCH_RES)  
  #define MTS_UNIT           (CM_UNIT / 100.0f)  
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
    volatile uint8 sonar_state;
    volatile uint16 start_time;
    volatile uint16 echo_time;
    
    float altitude;
    float last_altitude;
    float rate_of_change; /* Derivative of altitude with respect to time */
    
    AB_Filter ab_filter;   /* Alfa-Beta filter to smooth out the rate_of_change of the altitude */
    
  public:
    Sonar();
    void sonarInit(void);
    
    #ifdef PARALLAX_SONAR
      void sonarPulse(void);
    #endif
    
    void setStartTime(uint16 val);
    void setEchoTime(uint16 val);
    
    uint16 getEchoTime(void);
    float getSonarAltitude( float coef, float dt );
    void setSonarState(uint8 newState);
    uint8 getSonarState(void);
    uint8 isSonarDataAvailable(void);
    
    uint8 isActive(void);
};

extern Sonar sonar;

#endif
