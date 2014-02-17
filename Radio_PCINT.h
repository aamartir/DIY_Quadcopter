#ifndef _RADIO_H_
#define _RADIO_H_

#include "Quad.h"

/* Useful definitions */

#define RADIO_MAX_CHANNELS    4
#define RADIO_LAST_CHAN_INDEX (RADIO_MAX_CHANNELS - 1)
#define MAX_PW_US 	      3000  /* Empirical value */
#define MIN_PW_US 	      500   /* Empirical value */
#define FRAME_SYNCH_PW_MIN    4000  /* Empirical value */
#define FRAME_SYNCH_PW_MAX    20000 /* Empirical value */

#define RADIO_PITCH_CH        0
#define RADIO_ROLL_CH         1
#define RADIO_YAW_CH          3 
#define RADIO_THROTTLE_CH     2

#define RADIO_SMOOTH_DATA    1
#define RADIO_SMOOTH_FACTOR  0.50

//#define DEBUG_PPM_SIGNAL
#ifdef DEBUG_PPM_SIGNAL
  #define BUFFER_SIZE 100
#endif

/* State masks (Should be multiples of 2) */
#define RADIO_IDLE              0x00   /* Radio has not detected any data */
#define RADIO_SYNCH_ERROR       0x01   /* There was an error within a frame */
#define RADIO_SYNCH_START_FRAME 0x02   /* Radio has detected the synch frame, and will now expects channel pulses */
#define RADIO_SYNCH_END_FRAME   0x04   /* Radio has received the pulses for all channels, and it now expects another synch frame */
#define RADIO_SYNCH_RETRY       0x08   /* Radio failed to detect a full proper frame, and it is re-attempting frame parsing */
#define RADIO_SYNCHED           0x10   /* Radio has successfully detected full frame */
#define RADIO_SIGNAL_LOST       0x20   /* Radio has lost signal (Out of range, garbage, or radio is OFF) */

#define RADIO_SYNCH_ATTEMPTS    5      /* Number of maximum frame parse attempts */

/* These values have been 
 * carefully measured from
 * the receiver. A different
 * receiver may yield very
 * different values */
#define PITCH_MIN	  1052
#define PITCH_MAX	  1872
#define ROLL_MIN	  1052
#define ROLL_MAX	  1884
#define YAW_MIN		  1056
#define YAW_MAX		  1892
#define THTLE_MIN	  1060
#define THTLE_MAX	  1892
#define THTLE_MIDDLE      (THTLE_MIN + THTLE_MAX)/2.0f

class Radio_PCINT
{
public:
  volatile uint8_t chCounter;
  volatile uint16_t data_rdy_buffer[RADIO_MAX_CHANNELS];
  volatile uint16_t radio_data[RADIO_MAX_CHANNELS]; 
  
  volatile uint16_t state;
  volatile uint8_t retry_attempts;
  volatile uint8_t data_ready;

  Radio_PCINT();
  void init();
  void readReceiverData( void );
  void setState( uint8_t newState );
  uint8_t getState( void );
  uint8_t radioIsSynched( void );
  
  void printRadio( void );
};

extern Radio_PCINT radio;

//#ifdef DEBUG_PPM_SIGNAL
//  extern volatile uint8_t debug_data_rdy;
//  extern volatile uint16_t debug_buf[BUFFER_SIZE][2];
//#endif

#endif

