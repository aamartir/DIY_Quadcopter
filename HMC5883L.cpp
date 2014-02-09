/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Magnetometer Arduino Library.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for HMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf

*/

#include <Arduino.h>
#include "HMC5883L.h"
#include "I2C.h"

#define NEW_CODE 1

HMC5883L::HMC5883L()
{
  /* Initialize gains */
  setGain(1);
}

void HMC5883L::init()
{
  delay(100);
  
  setNumberOfSamplesAveraged(8);
  delay(50);
  
  setScale(SCALE_1_3);
  delay(50);
  
  setDataOutputRate(RATE_75_HZ);
  delay(50);
  
  setMeasurementMode(CONTINUOUS);
  delay(6);
  
  /* Calibrate scale and offsets per axis */
  setMagCal(X_axis, 0, 1);
  setMagCal(Y_axis, 0, 1);
  setMagCal(Z_axis, 0, 1);
}

void HMC5883L::readRawAxis(int *buf)
{
  uint8 tmp[6];
  i2c.mem_read(DEVICE_ID, DATA_OUT_REG, 6, tmp);
  
  buf[0] = (((int) tmp[0]) << 8) | tmp[1]; /* x axis */
  buf[1] = (((int) tmp[2]) << 8) | tmp[3]; /* y axis */
  buf[2] = (((int) tmp[4]) << 8) | tmp[5]; /* z axis */
}

void HMC5883L::readScaledAxis()
{
  int raw[3];
  readRawAxis(raw);
  
  /* Scale and calibrate */
  #ifdef SMOOTH_DATA_COMPASS
    scaled[X_axis] = SMOOTH_FACTOR*scaled[X_axis] + (1-SMOOTH_FACTOR)*(raw[X_axis] - offset[X_axis])*magScale[X_axis];
    scaled[Y_axis] = SMOOTH_FACTOR*scaled[Y_axis] + (1-SMOOTH_FACTOR)*(raw[Y_axis] - offset[Y_axis])*magScale[Y_axis];
    scaled[Z_axis] = SMOOTH_FACTOR*scaled[Z_axis] + (1-SMOOTH_FACTOR)*(raw[Z_axis] - offset[Z_axis])*magScale[Z_axis];
  #else
    scaled[X_axis] = (raw[X_axis] - offset[X_axis]) * magScale[X_axis];
    scaled[Y_axis] = (raw[Y_axis] - offset[Y_axis]) * magScale[Y_axis];
    scaled[Z_axis] = (raw[Z_axis] - offset[Z_axis]) * magScale[Z_axis];
  #endif
}

void HMC5883L::getHeading_tiltCompensate(double rollAngleDeg /* x axis */, double pitchAngleDeg /*y axis */)
{
  readScaledAxis(); /* Populate in array and store in scaled[...] */

  /* Swap accelerometer axis */
  float roll = TO_RADIANS(rollAngleDeg);
  float pitch = TO_RADIANS(pitchAngleDeg);
  float mag;
  float Hx;
  float Hy;
  
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);
  
  /* Algorithm */
  #ifdef NEW_CODE
    tiltCompensated[X_axis] = ((float) scaled[X_axis]) * cosPitch + \
                              ((float) scaled[Y_axis]) * sinPitch * sinRoll + \
                              ((float) scaled[Z_axis]) * cosRoll * sinPitch;
                              
    tiltCompensated[Y_axis] = ((float) scaled[Y_axis]) * cosRoll - \
                              ((float) scaled[Z_axis]) * sinRoll;
                              
    
    /* Vector magnitude */                           
    mag = sqrt(tiltCompensated[X_axis] * tiltCompensated[X_axis] + tiltCompensated[Y_axis] * tiltCompensated[Y_axis]);

    /* Normalized components */
    Hx = tiltCompensated[X_axis] / mag;
    Hy = -tiltCompensated[Y_axis] / mag;
    
    heading = atan2(Hy, Hy);
        
  #else 
    tiltCompensated[X_axis] = scaled[X_axis]*cosRoll + /*1.5*/scaled[Z_axis]*sinRoll;
    tiltCompensated[Z_axis] = -scaled[X_axis]*sinRoll + scaled[Z_axis]*cosRoll;
    tiltCompensated[Y_axis] = scaled[Y_axis]*cosPitch + /*1.5*/tiltCompensated[Z_axis]*sinPitch;
    tiltCompensated[Z_axis] = -scaled[Y_axis]*sinPitch + tiltCompensated[Z_axis]*cosPitch;

    heading = atan2(tiltCompensated[Y_axis], tiltCompensated[X_axis]);
    if(heading < -PI)
      heading += _2_PI;
    else if(heading > PI)
      heading -= _2_PI;
    
    heading = TO_DEGREES(heading);
  #endif
}

void HMC5883L::setMagCal(byte axis, float minVal, float maxVal)
{
  magMin[axis] = minVal;
  magMax[axis] = maxVal;
  
  // Assume max/min is scaled to +1 and -1
  // y2 = 1, x2 = max; y1 = -1, x1 = min
  // m = (y2 - y1) / (x2 - x1)
  // m = 2 / (max - min)
  magScale[axis] = 2.0f / (magMax[axis] - magMin[axis]);
  // b = y1 - mx1; b = -1 - (m * min)
  offset[axis] = -(magScale[axis] * magMin[axis]) - 1;
}

bool HMC5883L::isDataReady() 
{
  uint8 val;
  i2c.mem_read(DEVICE_ID, STATUS_REG, 1, &val);
  return (val & RDY_BIT);
}

int HMC5883L::setScale(double gauss)
{
  uint8 regValue = 0x00;
  if(gauss <= 0.89)
  {
    regValue = 0x00;
    setGain(0.73);
  }
  else if(gauss <= 1.4)
  {
    regValue = 0x01;
    setGain(0.92);
  }
  else if(gauss <= 2.0)
  {
    regValue = 0x02;
    setGain(1.22);
  }
  else if(gauss <= 2.6)
  {
    regValue = 0x03;
    setGain(1.52);
  }
  else if(gauss <= 4.1)
  {
    regValue = 0x04;
    setGain(2.27);
  }
  else if(gauss <= 4.8)
  {
    regValue = 0x05;
    setGain(2.56);
  }
  else if(gauss <= 5.7)
  {
    regValue = 0x06;
    setGain(3.03);
  }
  else if(gauss <= 8.2)
  {
    regValue = 0x07;
    setGain(4.35);
  }
  else
    return -1;
	
  // Setting is in the top 3 bits of the register.
  regValue = (regValue << 5);
  i2c.mem_write(DEVICE_ID, CRB_REG, regValue);
}

void HMC5883L::setGain(double val)
{
  setGains(val, val, val);
}

void HMC5883L::setGains(double gain_x, double gain_y, double gain_z)
{
  gains[X_axis] = gain_x;
  gains[Y_axis] = gain_y;
  gains[Z_axis] = gain_z;
}

int HMC5883L::setMeasurementMode(uint8 mode)
{
  i2c.mem_write(DEVICE_ID, MODE_REG, mode);
}

void HMC5883L::setDataOutputRate(uint8 rate)
{
  uint8 _s;
  i2c.mem_read(DEVICE_ID, CRA_REG, 1, &_s);
  _s &= ~(0x1C); /* Reset bits Dout_2 to Dout_0*/
  
  _s |= (rate << 2);
  i2c.mem_write(DEVICE_ID, CRA_REG, _s);
}

/* Possible values: 
 * 0x00 = 1 sample
 * 0x01 = 2 samples
 * 0x10 = 4 samples
 * 0x11 = 8 samples
*/
void HMC5883L::setNumberOfSamplesAveraged(uint8 samples)
{
  uint8 _conf;
  uint8 _s;
  i2c.mem_read(DEVICE_ID, CRA_REG, 1, &_s);
  
  /* Reset bits MA1 to MA0 */
  _s &= ~(0x60);
  
  switch(samples)
  {
    case 1:
      _conf = 0;
      break;
    
    case 2:
      _conf = 0x01;
      break;
    
    case 4:
      _conf = 0x10;
      break;
    
    case 8:
      _conf = 0x11;
      break;  
    
    /* If command not recognized, set the default to 1 sample */
    default:
      _conf = 0;
  }
  
  _s |= (_conf << 5);
  i2c.mem_write(DEVICE_ID, CRA_REG, _s);
}

double HMC5883L::getHeadingDegrees(double *data)
{
  return (getHeadingRad(data) * 180/PI);
}

double HMC5883L::getHeadingRad(double *data) /* data is scaled and with proper offset */
{
  double heading;
  
  //heading = wrap(atan2(-data[Y_axis], data[X_axis]) + DECLINATION*(PI/180));
  heading = atan2(-data[Y_axis], data[X_axis]) + DECLINATION;
  /*if(heading < 0)
    heading += _2_PI;
  if(heading > _2_PI)
    heading -= _2_PI;*/
    
  return heading;
}

int HMC5883L::Xaxis()
{
  return X_axis;
}

int HMC5883L::Yaxis()
{
  return Y_axis;
}

int HMC5883L::Zaxis()
{
  return Z_axis;
}

