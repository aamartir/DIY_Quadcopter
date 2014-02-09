#include <Arduino.h>
#include <stdio.h>
#include <avr/io.h>
#include "Radio_PCINT.h"
#include "Quad.h"
#include "ESC.h"
#include "CONTROL_PARAMS.h"
#include "Timer.h"

void radioFrameInterrupt(uint8 port);

#ifdef DEBUG_PPM_SIGNAL
  volatile uint16 debug_buf[BUFFER_SIZE][2];
  volatile uint16 debug_cnt;
  volatile uint8 debug_data_rdy;
#endif

/* Constructor */
Radio_PCINT::Radio_PCINT()
{
  /* Initialize radio_timer */
  radio_timer.setTimerVal( RADIO_TIMER_PERIOD );
}

void Radio_PCINT::init()
{
  /* Set pin as input */
  DDRD &= ~(1 << RADIO_INT_PIN); //  DDRD = DDRD & (1111 1011);
  
  /* Configure Pin Change Interrupt */
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18 /*RADIO_INT_PIN*/ ); 
  
  /* Initialize ready buffer */
  for(int i = 0; i < RADIO_MAX_CHANNELS; i++)
    data_rdy_buffer[i] = 0;
  
  state             = RADIO_IDLE;
  data_ready        = 0;
  retry_attempts    = RADIO_SYNCH_ATTEMPTS;
}

uint8 Radio_PCINT::getState()
{
  return state;
}

void Radio_PCINT::setState( uint8 newState )
{
  state = newState;
}

uint8 Radio_PCINT::radioIsSynched()
{
  return (state & RADIO_SYNCHED);
}

/* radio.radio_data[]         contains raw data from Radio. It is NOT filtered nor manipulated in any way. 
 * radio.radio_rdy_buffer[]   contains radio data for the user as it becomes available. It CAN be filtered or manipulated to improve noise. */
void Radio_PCINT::readReceiverData()
{
  /* Only extract new data if 'ready' flag is set */
  if(data_ready)
  {
    /* Clear flag as soon as possible so buffer are not updated by the ISR */
    data_ready = 0; 
    
    /* Signal filtering */
    for( int i = 0; i < RADIO_MAX_CHANNELS; i++ )
    {
      #ifdef RADIO_SMOOTH_DATA
        data_rdy_buffer[i] = RADIO_SMOOTH_FACTOR * data_rdy_buffer[i] + (1-RADIO_SMOOTH_FACTOR) * radio_data[i];
      #else
        data_rdy_buffer[i] = radio_data[i];
      #endif
    }
    
    /* Map buffer values to actual numbers we will use in the control algorithm */
    target[ROLL]  = map(data_rdy_buffer[RADIO_ROLL_CH],     ROLL_MIN,  ROLL_MAX,  -45, 	       45);     // ROLL
    target[PITCH] = map(data_rdy_buffer[RADIO_PITCH_CH],    PITCH_MIN, PITCH_MAX, -45,         45);     // PITCH
    throttle      = map(data_rdy_buffer[RADIO_THROTTLE_CH], THTLE_MIN, THTLE_MAX,  MOTOR_MIN,  MOTOR_MAX); // THROTTLE
    
    /* Yaw is a bit different. It will be mapped to a floating point value for more precision */
    headingIncr   = mapToDouble((double) data_rdy_buffer[RADIO_YAW_CH],    
                                (double) YAW_MIN,   
                                (double) YAW_MAX,    
                                -10.0f,         
                                 10.0f);     
 
    //throttle = constrain(throttle, MOTOR_MIN, MOTOR_MAX); 
    if(throttle > (MOTOR_MAX << 1) || (throttle < MOTOR_MIN))
      throttle = MOTOR_MIN;  
    
    /* Manage dead-zones (thesholds) */
    if(abs(target[ROLL]) <= 5.0)
       target[ROLL] = 0;
    if(abs(target[PITCH]) <= 5.0)
       target[PITCH] = 0;
    if(abs(headingIncr) >= 0.5)
    {
      targetHeading -= headingIncr;
      
      /* ROLL-OVER */
      //if(targetHeading < -180)
      //  targetHeading = 180;
      //else if(targetHeading > 180)
      //  targetHeading = -180;
    }
  }
}

volatile uint32 time_last;
volatile uint32 time_new;
volatile uint16 diff;
volatile uint8 frameStarted;
volatile uint8 pin_state;
volatile uint8 last_pin_state = 0;

/* Before you get depressed if the code/thing does NOT work, 
 * Make sure that RADIO_MAX_CHANNELS has the correct value! */
ISR( PCINT2_vect )
{
  /* Enable interrupts for pwm control (May not need this, but leave just in case) */
  sei();

  time_new = micros();
  diff = time_new - time_last;
  
  /* Hight or Low */
  pin_state = (PIND >> RADIO_INT_PIN) & 0x1;
  
  if( !radio.data_ready ) /* Only fill buffer if data NOT used yet */
  {
    if( pin_state == 0 && last_pin_state ) /* If signal is LOW (Beginning of new frame) */
    {
      if( radio.chCounter == RADIO_LAST_CHAN_INDEX && 
          radio.state & RADIO_SYNCH_END_FRAME )
      {
        radio.radio_data[radio.chCounter] = time_new - radio.radio_data[radio.chCounter]; /* This is the duration of the last pulse */
        
        /* Last valid pulse? */
        if(radio.radio_data[radio.chCounter] < MIN_PW_US || radio.radio_data[radio.chCounter] > MAX_PW_US)
          SET_BIT( radio.state, RADIO_SYNCH_ERROR );
        else
        {
          radio.data_ready = 1; /* Data buffer is ready */
          CLEAR_BIT( radio.state, RADIO_SYNCH_END_FRAME ); 
          
          /* Reset radio timer */
          if( radio.state & RADIO_SIGNAL_LOST )
          {
            gpio_clear( RED_LED_PIN );
            CLEAR_BIT( radio.state, RADIO_SIGNAL_LOST ); 
          }
          
          /* Reset radio timer */
          radio_timer.setTimerVal( RADIO_TIMER_PERIOD );
        }  
      }
      
      /* Reset state machine */
      SET_BIT( radio.state, RADIO_SYNCH_START_FRAME );
      CLEAR_BIT( radio.state, RADIO_SYNCH_ERROR );
      CLEAR_BIT( radio.state, RADIO_SYNCH_END_FRAME );
      
      frameStarted = 0;
      radio.chCounter = 0; /* Reset counter */
    }
    else if(radio.state & RADIO_SYNCH_START_FRAME && 
           (radio.state & RADIO_SYNCH_ERROR) == 0 &&
            diff >= MIN_PW_US)
    {
      if(frameStarted)
      {
        radio.radio_data[radio.chCounter] = time_new - radio.radio_data[radio.chCounter];
        
        /* Check for PW Error!!! */
        if(radio.radio_data[radio.chCounter] > MAX_PW_US)
        {
          SET_BIT( radio.state, RADIO_SYNCH_ERROR );
        }
        else
        {  
          radio.radio_data[++radio.chCounter] = time_new;
          
          if(radio.chCounter == RADIO_LAST_CHAN_INDEX)
          {
            SET_BIT(radio.state, RADIO_SYNCH_END_FRAME);
            CLEAR_BIT(radio.state, RADIO_SYNCH_START_FRAME);
          }
        }
      }
      else
      {
        radio.radio_data[radio.chCounter] = time_new;
        frameStarted = 1;
      }
    }
  }
  
  last_pin_state = pin_state;
  time_last = time_new;
}

void Radio_PCINT::printRadio( void )
{
  uint8 ch;
  
  for( ch = 0; ch < RADIO_MAX_CHANNELS; ch++ )
  {
    Serial.print( data_rdy_buffer[ch] );
    Serial.print( "\t" );
  }
  
  Serial.println();
}
