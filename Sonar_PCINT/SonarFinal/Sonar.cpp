#include <Arduino.h>
#include <avr/interrupt.h>
#include "Sonar.h"
#include "AB_Filter.h"

Sonar::Sonar()
{
  /* Initialize alfa-beta filter */
  ab_filter.setFilterCoefficients( 0.40f, 0.001f );
}

void Sonar::sonarInit()
{
  /* Setup PCINT interrupts */
  PCMSK2 = _BV(PCINT20);
  PCICR  = _BV(PCIE2); 

  sonar_state = SONAR_ACTIVE;
  altitude = 0;
  last_altitude = 0;
}

float Sonar::getSonarAltitude( float dt, float coef )
{
  if( isSonarDataAvailable() )
  {
    /* Keep track of the last altitude */
    last_altitude = ab_filter.smatrix[0]; //altitude;
    altitude = ((float)echo_time)*coef; 
    
    /* filter atitude */
    ab_filter.filter( altitude, dt );
    
    //if(altitude > MAX_SONAR_DISTANCE)
    //  altitude = MAX_SONAR_DISTANCE;
    
    /* Filter the discrete rate of change 
     * in order to produce a smooth derivative */ 
    rate_of_change =  (ab_filter.smatrix[0] - last_altitude)/dt;
    
    /* Reset state so that new measurements can be taken by the interrupt routine */
    sonar_state = SONAR_ACTIVE;
  }
  
  return ab_filter.smatrix[0];
}

float Sonar::getSonarAltitude_filtered()
{
  ab_filter.smatrix[0];
}

float Sonar::getSonarROF()
{
  return rate_of_change;
}

void Sonar::setSonarState( char newState )
{
  sonar_state = newState;
}
char Sonar::getSonarState(void)
{
  return sonar_state;
}

char Sonar::isSonarDataAvailable(void)
{
  return (sonar_state == SONAR_DATA_AVAIL);
}

char Sonar::isActive(void)
{
  return (sonar_state == SONAR_ACTIVE);
}

void Sonar::setStartTime(unsigned int val)
{
  start_time = val;
}

void Sonar::setEchoTime(unsigned int val)
{
  echo_time = val - start_time;
}
    
unsigned int Sonar::getEchoTime(void)
{
  return echo_time;
}

ISR(PCINT2_vect)
{
  /* Only extract new pulses if no data is present in buffer already */
  if( !sonar.isSonarDataAvailable() )
  {
    if( (PIND & SONAR_PIN_MASK) > 0 )
    {
      sonar.setStartTime(micros());
      sonar.setSonarState(SONAR_PULSE_START); 
    }
    else if( sonar.getSonarState() == SONAR_PULSE_START ) /* Look for SONAR_PULSE_START */
    {
      sonar.setEchoTime(micros()); /* micros() - start_time */
      sonar.setSonarState(SONAR_DATA_AVAIL); /* Measurement found */
    }
  }
}
