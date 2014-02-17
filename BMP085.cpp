#include "BMP085.h"
#include "I2C.h"

BMP085::BMP085() {
}

boolean BMP085::init( uint8_t mode ) 
{
  if ( mode > ULTRAHIGHRES ) 
    mode = ULTRAHIGHRES;
    
  oversampling = mode;

  if( I2C::read8( DEV_ADDR, 0xD0 ) != 0x55 ) 
    return false;

  /* read calibration data */
  ac1 = I2C::read16( DEV_ADDR, CAL_AC1 );
  ac2 = I2C::read16( DEV_ADDR, CAL_AC2 );
  ac3 = I2C::read16( DEV_ADDR, CAL_AC3 );
  ac4 = I2C::read16( DEV_ADDR, CAL_AC4 );
  ac5 = I2C::read16( DEV_ADDR, CAL_AC5 );
  ac6 = I2C::read16( DEV_ADDR, CAL_AC6 );

  b1 = I2C::read16( DEV_ADDR, CAL_B1 );
  b2 = I2C::read16( DEV_ADDR, CAL_B2 );

  mb = I2C::read16( DEV_ADDR, CAL_MB );
  mc = I2C::read16( DEV_ADDR, CAL_MC );
  md = I2C::read16( DEV_ADDR, CAL_MD );
  
  /* Configure (EOC_PIN = PD2) as input */  
  DDRD &= ~(1 << EOC_PIN);
  
  /* Setup PCINT interrupts */
  PCMSK2 |= (1 << PCINT18); /* Enable PD2 (digital pin 2)*/
  PCICR  |= (1 << PCIE2);   /* Enable External Interrupts on pins PCINT0...7 */
}

void BMP085::requestRawTemperature(void) 
{
  curr_cmd.val = GET_RAW_TEMPERATURE_CMD;
  curr_cmd.state = FETCHING;
  
  I2C::write8( DEV_ADDR, CONTROL_REG, READTEMP_CMD );
}

void BMP085::requestRawPressure(void) 
{
  curr_cmd.val = GET_RAW_PRESSURE_CMD;
  curr_cmd.state = FETCHING;

  I2C::write8( DEV_ADDR, CONTROL_REG, READPRESSURE_CMD + (oversampling << 6) );
}

/* Read the contents of temperature register and store in public variable */
void BMP085::rawTemperatureReadSrv( void )
{
  UT = I2C::read16( DEV_ADDR, TEMPERATURE_REG );
}

/* Read the contents of pressure register and store in public variable */
void BMP085::rawPressureReadSrv( void )
{
  UP = I2C::read16( DEV_ADDR, PRESSURE_REG );
  UP <<= 8; 
  UP |= I2C::read8( DEV_ADDR, PRESSURE_REG + 2 ); 
  UP >>= (8 - oversampling);
}

#ifdef IGNORE
int32_t BMP085::calibratePressureAtSeaLevel( void )
{
  #if BMP085_DEBUG == 1
    Serial.print("Calibrating... ");
  #endif
  
  uint8_t i;
  int32_t p = 0;
  for(i = 0; i < 100; i++) /* Average 100 readings */
    p += readPressure();
    
  p /= 100;
  
  #if BMP085_DEBUG == 1
    Serial.println("OK");
  #endif

  return p;
}
#endif

void BMP085::calculateRealTempAndPressure( void )
{
  int32_t B3, B5, B6, X1, X2, X3, p, t;
  uint32_t B4, B7;
  
  /* State machine */
  curr_cmd.val = GET_REAL_TEMP_AND_PRESSURE_CMD;
  curr_cmd.state = FETCHING;
  
  // do temperature calculations
  X1 = ((UT - (int32_t)ac6) * (int32_t)ac5) >> 15;
  X2 = ((int32_t)mc << 11)/(X1 + md);
  B5 = X1 + X2;
  //temperature = (B5 + 8) >> 4; /* Don't need to know this for now */
  
  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6) >> 12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = (((((int32_t)ac1 << 2) + X3) << oversampling) + 2) >> 2; 
  
  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );
  
  if (B7 < 0x80000000) 
    p = (B7 << 1) / B4; //(B7 * 2) / B4;
  else 
    p = (B7 << 1) / B3; //(B7 * 2) / B3;
    
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  
  p = p + ((X1 + X2 + (int32_t)3791) >> 4);
  
  if( pressure == 0 )
    pressureAtSeaLevel = p;
  
  pressure = p;
  
  /* Update state machine */
  curr_cmd.state = READY;
}

/* Cannot execute within interrupt routine. This fuction has to be
 * called normally within program loop */
void BMP085::update_state_machine( void )
{
  //Serial.print( curr_cmd.val, HEX );
  //Serial.print( " " );
  //Serial.println( curr_cmd.state, HEX );
  
  /* If value is ready to be read, then read it, and advance to next state */
  if( curr_cmd.val != NO_CMD && curr_cmd.state == READY )
  {
    switch( curr_cmd.val )
    {
      case GET_RAW_TEMPERATURE_CMD:
        /* We can now read the temperature register */
        rawTemperatureReadSrv();
        break;
      case GET_RAW_PRESSURE_CMD:
        /* We can now read the pressure register */
        rawPressureReadSrv();
        break;
    }
    
    /* Current cmd cycle exectuted */
    curr_cmd.state = DONE;
  }
  
  if( curr_cmd.val == NO_CMD || curr_cmd.state == DONE ) 
  {
    if( curr_cmd.val == NO_CMD )
    {
      /* First, we request raw temperature */
      requestRawTemperature();
    }
    else if( curr_cmd.val == GET_RAW_TEMPERATURE_CMD )
    {
      requestRawPressure();
    }
    else if( curr_cmd.val == GET_RAW_PRESSURE_CMD )
    {
      /* Here we do the altitude calculations. At this point
       * it is assumed that the public members UT and UP have
       * been populated. 
       */
      calculateRealTempAndPressure();
    }
    else if( curr_cmd.val == GET_REAL_TEMP_AND_PRESSURE_CMD )
    {
      updateAltitude( pressureAtSeaLevel );
    }
  }
}

void BMP085::updateAltitude( float pressureAtSeaLevel )
{
  /* Calculate altitude */
  #ifdef SMOOTH_ALTITUDE_DATA
    altitude = SMOOTH_FACTOR * altitude + (1 - SMOOTH_FACTOR) * (44330 * (1.0 - pow( pressure / pressureAtSeaLevel, 0.1903 )));
  #else
    altitude = 44330 * (1.0 - pow( pressure / pressureAtSeaLevel, 0.1903 ));
  #endif
  
  /* Altitude ready for user to read */  
  new_altitude_val_ready = 1;
  
  /* Update state machine to allow new altitude calculations */
  curr_cmd.val = NO_CMD;
  curr_cmd.state = NO_OP;
}

float BMP085::getAltitude( void )
{
  new_altitude_val_ready = 0;
  return altitude;
}

ISR(PCINT2_vect)
{
  /* End of Conversion (1). Still running (0) */
  if( (PIND & EOC_PIN_MASK) )
  {
    /* Requested value is ready */
    bmp.curr_cmd.state = READY;
  }
}


