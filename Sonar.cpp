#ifdef USE_SONAR

#include <Arduino.h>
#include <avr/interrupt.h>
#include "Sonar.h"
#include "AB_Filter.h"

Sonar::Sonar()
{
  
}

void Sonar::sonarInit()
{
  DDRB &= ~(1 << SONAR_PIN);
  
  /* Setup PCINT interrupts */
  PCMSK0 = (1 << PCINT4); /* Enable PB4 (digital pin 12)*/
  PCICR  = (1 << PCIE0); /* Enable External Interrupts on pins PCINT0...7 */
  
  #ifdef PARALLAX_SONAR
    sonar_state = SONAR_INACTIVE;
  #else 
    sonar_state = SONAR_ACTIVE;
  #endif 
  
  altitude = 0;
  last_altitude = 0;
  
  /* Initialize alfa-beta-gamma filter */
  //ab_filter.setFilterCoefficients( 0.35f, 0.001f );
}

#ifdef PARALLAX_SONAR
  void Sonar::sonarPulse()
  {
    /* Disable interrupts */
    pinMode(SONAR_PIN + 8, OUTPUT);
    PORTB &= ~(1 << SONAR_PIN); /* Drive pin LOW */
    delayMicroseconds(2);
    PORTB |= (1 << SONAR_PIN);  /* Drive pin HIGH */
    delayMicroseconds(5);
    PORTB &= ~(1 << SONAR_PIN); /* Drive pin LOW */ 
    pinMode(SONAR_PIN + 8, INPUT);
      
    sonar_state = SONAR_ACTIVE;
  }
#endif

float Sonar::getSonarAltitude( float coef, float dt )
{
  float newVal = ((float)echo_time)*coef; /* Convert pulse width distance measurement */
  
  /* Reject high-frequency artifacts */
  if( isSonarDataAvailable() )
  {
    /* Keep track of the last altitude */
    last_altitude = altitude;
    
    #ifdef SMOOTH_SONAR_DATA
      altitude = SMOOTH_FACTOR*altitude + (1-SMOOTH_FACTOR)*(newVal);
    #else
      altitude = newVal;
    #endif
    
    /*
    if(altitude > MAX_SONAR_DISTANCE)
      altitude = MAX_SONAR_DISTANCE;
    */
    
    rate_of_change = (altitude - last_altitude)/dt;
  }

  /* Reset state so that new measurements can be taken by the interrupt routine */
  #ifdef PARALLAX_SONAR
    sonar_state = SONAR_INACTIVE;
  #else 
    sonar_state = SONAR_ACTIVE;
  #endif 
  
  return altitude;
}

void Sonar::setSonarState( uint8 newState )
{
  sonar_state = newState;
}
uint8 Sonar::getSonarState(void)
{
  return sonar_state;
}

uint8 Sonar::isSonarDataAvailable(void)
{
  return (sonar_state == SONAR_DATA_AVAIL);
}

uint8 Sonar::isActive(void)
{
  return (sonar_state == SONAR_ACTIVE);
}

void Sonar::setStartTime(uint16 val)
{
  start_time = val;
}

void Sonar::setEchoTime(uint16 val)
{
  echo_time = val - start_time;
}
    
uint16 Sonar::getEchoTime(void)
{
  return echo_time;
}

ISR(PCINT0_vect)
{
  if( (PINB & SONAR_PIN_MASK) > 0 )
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

#endif
