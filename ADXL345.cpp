/**************************************************************************
 *                                                                         *
 * ADXL345 Driver for Arduino                                              *
 *                                                                         *
 ***************************************************************************
 *                                                                         * 
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/
#include <Arduino.h>
#include "ADXL345.h"

ADXL345::ADXL345() 
{
  status = OK;
  error_code = NO_ERROR;
  
  /* These are empirical values that map accel raw values to +/-1g on all axis */
  gains[0] = 0.00376390;
  gains[1] = 0.00373134;
  gains[2] = 0.00396825;
}

void ADXL345::powerOn() 
{
  /* Turning on the ADXL345 */
  I2C::write8(DEVICE_ID, POWER_CTL_REG, 0);      
  I2C::write8(DEVICE_ID, POWER_CTL_REG, 16);
  I2C::write8(DEVICE_ID, POWER_CTL_REG, 8); 
  delay(50);
  
  set_bw(BW_400); /* Set bandwidth */
  delay(10);
  
  /* Set axis offsets (Empirical values) */
  setAxisOffset( -4, 0, 5 );
  
  /* Configure axis signs */
  sign[X_axis] =  1; 
  sign[Y_axis] = -1;
  sign[Z_axis] =  1;
  
  initialized = 1;
}

void ADXL345::readAccel() 
{
  I2C::readBytes( DEVICE_ID, DATAX0_REG, 6, _buff ); //read the acceleration data from the ADXL345

  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  
  /* Raw data */
  raw[0] = (((int)_buff[1]) << 8) | _buff[0];   /* x */
  raw[1] = (((int)_buff[3]) << 8) | _buff[2];   /* y */
  raw[2] = (((int)_buff[5]) << 8) | _buff[4];   /* z */
  
  /* Scaled data (acceleration). Data will also be filtered using coefficient SMOOTH_FACTOR */
  #ifdef SMOOTH_ACCEL_DATA
    acceleration[0] = (SMOOTH_FACTOR*acceleration[0]) + (1-SMOOTH_FACTOR)*(raw[0] * gains[0]);
    acceleration[1] = (SMOOTH_FACTOR*acceleration[1]) + (1-SMOOTH_FACTOR)*(raw[1] * gains[1]);
    acceleration[2] = (SMOOTH_FACTOR*acceleration[2]) + (1-SMOOTH_FACTOR)*(raw[2] * gains[2]);
  #else
    acceleration[0] = raw[0] * gains[0];
    acceleration[1] = raw[1] * gains[1];
    acceleration[2] = raw[2] * gains[2];
  #endif
  
  /* Angles based on acceleration (in radians) */
  getAngleRad(acceleration, angle);
}

// Gets the range setting and return it into rangeSetting
// it can be 2, 4, 8 or 16
void ADXL345::getRangeSetting(uint8_t * rangeSetting) 
{
  uint8_t _b;
  _b = I2C::read8( DEVICE_ID, DATA_FORMAT_REG );
  *rangeSetting = _b & B00000011;
}

// Sets the range setting, possible values are: 2, 4, 8, 16
void ADXL345::setRangeSetting(int val) 
{
  uint8_t _s;
  uint8_t _b;

  switch (val) {
  case 2:  
    _s = B00000000; 
    break;
  case 4:  
    _s = B00000001; 
    break;
  case 8:  
    _s = B00000010; 
    break;
  case 16: 
    _s = B00000011; 
    break;
  default: 
    _s = B00000000;
  }
  
  _b = I2C::read8( DEVICE_ID, DATA_FORMAT_REG );
  _s |= (_b & B11101100);
  I2C::write8( DEVICE_ID, DATA_FORMAT_REG, _s );
}

// Gets the state of the FULL_RES bit
bool ADXL345::getFullResBit() 
{
  return I2C::readBit( DEVICE_ID, DATA_FORMAT_REG, 3 );
}

// Sets the FULL_RES bit
// if set to 1, the device is in full resolution mode, where the output resolution increases with the
//   g range set by the range bits to maintain a 4mg/LSB scal factor
// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
//   and scale factor
void ADXL345::setFullResBit(bool fullResBit) 
{
  I2C::writeBit( DEVICE_ID, DATA_FORMAT_REG, 3, fullResBit );
}

// set/get the gain for each axis in Gs / count
void ADXL345::setAxisGains(double *_gains)
{
  int i;
  for(i = 0; i < 3; i++){
    gains[i] = _gains[i];
  }
}

void ADXL345::getAxisGains(double *_gains)
{
  int i;
  for(i = 0; i < 3; i++){
    _gains[i] = gains[i];
  }
}
  

// Sets the OFSX, OFSY and OFSZ bytes
// OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
// a scale factor of 15,6mg/LSB
// OFSX, OFSY and OFSZ should be comprised between 
void ADXL345::setAxisOffset(int x, int y, int z) 
{
  I2C::write8(DEVICE_ID, OFSX_REG, (byte) x);  
  I2C::write8(DEVICE_ID, OFSY_REG, (byte) y);  
  I2C::write8(DEVICE_ID, OFSZ_REG, (byte) z);  
}

// Gets the OFSX, OFSY and OFSZ bytes
void ADXL345::getAxisOffset(int* x, int* y, int*z) 
{
  uint8_t _b;
  _b = I2C::read8( DEVICE_ID, OFSX_REG );  
  *x = int (_b);
  
  _b = I2C::read8(DEVICE_ID, OFSY_REG );  
  *y = int (_b);
  
  _b = I2C::read8(DEVICE_ID, OFSZ_REG );  
  *z = int (_b);
}

double ADXL345::getRate()
{
  uint8_t _b;
  _b = I2C::read8( DEVICE_ID, BW_RATE_REG );
  _b &= B00001111;
  return (pow(2,((int) _b)-6)) * 6.25;
}

void ADXL345::setRate(double rate)
{
  uint8_t _b,_s;
  
  int v = (int) (rate / 6.25);
  int r = 0;
  
  while (v >>= 1)
  {
    r++;
  }
  if (r <= 9) 
  { 
    _b = I2C::read8( DEVICE_ID, BW_RATE_REG );
    _s = (uint8_t) (r + 6) | (_b & B11110000);
    I2C::write8(DEVICE_ID, BW_RATE_REG, _s);
  }
}

void ADXL345::set_bw(uint8_t bw_code)
{
  if((bw_code < BW_3) || (bw_code > BW_1600))
  {
    status = false;
    error_code = BAD_ARG;
  }
  else
  {
    I2C::write8( DEVICE_ID, BW_RATE_REG, bw_code );
  }
}

uint8_t ADXL345::get_bw_code()
{
  return I2C::read8( DEVICE_ID, BW_RATE_REG );
}

void ADXL345::getAngleRad(double *accel_data, double *angles)
{
   angles[X_axis] = atan2(accel_data[X_axis] * sign[X_axis], sqrt(accel_data[Y_axis] * accel_data[Y_axis] + accel_data[Z_axis] * accel_data[Z_axis]));
   angles[Y_axis] = atan2(accel_data[Y_axis] * sign[Y_axis], sqrt(accel_data[X_axis] * accel_data[X_axis] + accel_data[Z_axis] * accel_data[Z_axis]));
   //angles[Z_axis] = 0.9*angles[Z_axis] + 0.1*atan2(accel_data[Z_axis] * sign[Z_axis], sqrt(accel_data[X_axis] * accel_data[X_axis] + accel_data[Y_axis] * accel_data[Y_axis]));
}

void ADXL345::getAngleDegrees(double *accel_data, double *angles)
{
  getAngleRad(accel_data, angles);
  angles[X_axis] = angles[X_axis] * 180.0f/PI;
  angles[Y_axis] = angles[Y_axis] * 180.0f/PI;
  //angles[Z_axis] = angles[Z_axis] * _180_OVER_PI;
}

int ADXL345::Xaxis()
{
  return X_axis;
}

int ADXL345::Yaxis()
{
  return Y_axis;
}

int ADXL345::Zaxis()
{
  return Z_axis;
}
