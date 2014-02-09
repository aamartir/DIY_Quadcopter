/****************************************************************************
* ITG3200.h - ITG-3200/I2C library v0.5 for Arduino                         *
* Copyright 2010-2011 Filipe Vieira & various contributors                  *
* http://code.google.com/p/itg-3200driver                                   *
* This file is part of ITG-3200 Arduino library.                            *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/
/****************************************************************************
* Tested on Arduino Mega with ITG-3200 Breakout                             *
* SCL     -> pin 21     (no pull up resistors)                              *
* SDA     -> pin 20     (no pull up resistors)                              *
* CLK & GND -> pin GND                                                      *
* INT       -> not connected  (but can be used)                             *
* VIO & VDD -> pin 3.3V                                                     *
*****************************************************************************/
#include <Arduino.h>
#include "ITG3200.h"
#include "I2C.h"

ITG3200::ITG3200() {
  setOffsets(0,0,0);
  setScaleFactor(1.0, 1.0, 1.0, false);  // true to change readGyro output to radians
  //Wire.begin();       //Normally this code is called from setup() at user code
                        //but some people reported that joining I2C bus earlier
                        //apparently solved problems with master/slave conditions.
                        //Uncomment if needed.
}

void ITG3200::init(unsigned int  address) {
  // Uncomment or change your default ITG3200 initialization
  
  // fast sample rate - divisor = 0 filter = 0 clocksrc = 0, 1, 2, or 3  (raw values)
  init(address, NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, true, true);
  
  // slow sample rate - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 0, 1, 2, or 3  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, INTERNALOSC, true, true);
  
  // fast sample rate 32Khz external clock - divisor = 0  filter = 0  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_EXTERNAL32, true, true);
  
  // slow sample rate 32Khz external clock - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 4  (raw values)
  //init(NOSRDIVIDER, RANGE2000, BW010_SR1, PLL_EXTERNAL32, true, true);
  
  sign[X_axis] = -1;
  sign[Y_axis] = -1;
  sign[Z_axis] = 1;
  
  initialized = 1;
}

void ITG3200::init(unsigned int address, uint8 _SRateDiv, uint8 _Range, uint8 _filterBW, uint8 _ClockSrc, bool _ITGReady, bool _INTRawDataReady) {
  _dev_address = address;
  setSampleRateDiv(_SRateDiv);
  delay(10);
  setFSRange(_Range);
  delay(10);
  setFilterBW(_filterBW);
  delay(10);
  setClockSource(_ClockSrc);
  delay(10);
  setITGReady(_ITGReady);
  delay(10);
  setRawDataReady(_INTRawDataReady);  
  delay(GYROSTART_UP_DELAY);  // startup 
}

uint8 ITG3200::getDevAddr() {
  /*i2c.mem_read(_dev_address, WHO_AM_I, 1, &_buff[0]); 
  return _buff[0];  */
  return _dev_address;
}

void ITG3200::setDevAddr(unsigned int  _addr) {
  i2c.mem_write(_dev_address, WHO_AM_I, _addr); 
  _dev_address = _addr;
}

uint8 ITG3200::getSampleRateDiv() {
  i2c.mem_read(_dev_address, SMPLRT_DIV, 1, &_buff[0]);
  return _buff[0];
}

void ITG3200::setSampleRateDiv(uint8 _SampleRate) {
  i2c.mem_write(_dev_address, SMPLRT_DIV, _SampleRate);
}

uint8 ITG3200::getFSRange() {
  i2c.mem_read(_dev_address, DLPF_FS, 1, &_buff[0]);
  return ((_buff[0] & DLPFFS_FS_SEL) >> 3);
}

void ITG3200::setFSRange(uint8 _Range) {
  i2c.mem_read(_dev_address, DLPF_FS, 1, &_buff[0]);   
  i2c.mem_write(_dev_address, DLPF_FS, ((_buff[0] & ~DLPFFS_FS_SEL) | (_Range << 3)) ); 
}

uint8 ITG3200::getFilterBW() {  
  //readmem(DLPF_FS, 1, &_buff[0]);
  i2c.mem_read(_dev_address, DLPF_FS, 1, &_buff[0]);
  return (_buff[0] & DLPFFS_DLPF_CFG); 
}

void ITG3200::setFilterBW(uint8 _BW) {   
  //readmem(DLPF_FS, 1, &_buff[0]);
  //writemem(DLPF_FS, ((_buff[0] & ~DLPFFS_DLPF_CFG) | _BW)); 
  i2c.mem_read(_dev_address, DLPF_FS, 1, &_buff[0]);
  i2c.mem_write(_dev_address, DLPF_FS, ((_buff[0] & ~DLPFFS_DLPF_CFG) | _BW));
}

bool ITG3200::isINTActiveOnLow() {  
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_ACTL) >> 7);
}

void ITG3200::setINTLogiclvl(bool _State) {
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  i2c.mem_write(_dev_address, INT_CFG, ((_buff[0] & ~INTCFG_ACTL) | (_State << 7))); 
}

bool ITG3200::isINTOpenDrain() {  
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_OPEN) >> 6);
}

void ITG3200::setINTDriveType(bool _State) {
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  i2c.mem_write(_dev_address, INT_CFG, ((_buff[0] & ~INTCFG_OPEN) | _State << 6)); 
}

bool ITG3200::isLatchUntilCleared() {    
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_LATCH_INT_EN) >> 5);
}

void ITG3200::setLatchMode(bool _State) {
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  i2c.mem_write(_dev_address, INT_CFG, ((_buff[0] & ~INTCFG_LATCH_INT_EN) | _State << 5)); 
}

bool ITG3200::isAnyRegClrMode() {    
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}

void ITG3200::setLatchClearMode(bool _State) {
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  i2c.mem_write(_dev_address, INT_CFG, ((_buff[0] & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4)); 
}

bool ITG3200::isITGReadyOn() {   
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  return ((_buff[0] & INTCFG_ITG_RDY_EN) >> 2);
}

void ITG3200::setITGReady(bool _State) {
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  i2c.mem_write(_dev_address, INT_CFG, ((_buff[0] & ~INTCFG_ITG_RDY_EN) | _State << 2)); 
}

bool ITG3200::isRawDataReadyOn() {
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  return (_buff[0] & INTCFG_RAW_RDY_EN);
}

void ITG3200::setRawDataReady(bool _State) {
  i2c.mem_read(_dev_address, INT_CFG, 1, &_buff[0]);
  i2c.mem_write(_dev_address, INT_CFG, ((_buff[0] & ~INTCFG_RAW_RDY_EN) | _State)); 
}

bool ITG3200::isITGReady() {
  i2c.mem_read(_dev_address, INT_STATUS, 1, &_buff[0]);
  return ((_buff[0] & INTSTATUS_ITG_RDY) >> 2);
}

bool ITG3200::isRawDataReady() {
  i2c.mem_read(_dev_address, INT_STATUS, 1, &_buff[0]);
  return (_buff[0] & INTSTATUS_RAW_DATA_RDY);
}

void ITG3200::readTemp(double *_Temp) {
  i2c.mem_read(_dev_address, TEMP_OUT,2,_buff);
  *_Temp = 35 + ((_buff[0] << 8 | _buff[1]) + 13200) / 280.0;    // F=C*9/5+32
}

void ITG3200::readGyro()
{
  //readmem(GYRO_XOUT, 6, _buff);
  i2c.mem_read(_dev_address, GYRO_XOUT, 6, _buff);
  
  /* Populate Raw values */
  raw[0] = sign[0]*(_buff[0] << 8 | _buff[1]); /* local X axis. It corresponds to accelerometer's Y axis. Have to remember to invert it later */
  raw[1] = sign[1]*(_buff[2] << 8 | _buff[3]); /* local Y axis. accel's X axis */
  raw[2] = sign[2]*(_buff[4] << 8 | _buff[5]);
  
  /* Populate Calibrated Values */
  #ifdef SMOOTH_GYRO_DATA
    rate[X_axis] = SMOOTH_FACTOR*(rate[X_axis]) + (1-SMOOTH_FACTOR)*(raw[X_axis] - offsets[X_axis]) / scalefactor[X_axis]; 
    rate[Y_axis] = SMOOTH_FACTOR*(rate[Y_axis]) + (1-SMOOTH_FACTOR)*(raw[Y_axis] - offsets[Y_axis]) / scalefactor[Y_axis]; 
    rate[Z_axis] = SMOOTH_FACTOR*(rate[Z_axis]) + (1-SMOOTH_FACTOR)*(raw[Z_axis] - offsets[Z_axis]) / scalefactor[Z_axis]; 
  #else
    rate[X_axis] = (raw[X_axis] - offsets[X_axis]) / scalefactor[X_axis];
    rate[Y_axis] = (raw[Y_axis] - offsets[Y_axis]) / scalefactor[Y_axis];
    rate[Z_axis] = (raw[Z_axis] - offsets[Z_axis]) / scalefactor[Z_axis];
  #endif
}

void ITG3200::readGyroRaw(int *gX, int *gY, int *gZ)
{
  i2c.mem_read(_dev_address, GYRO_XOUT, 6, _buff);
  *gX = sign[0]*(_buff[0] << 8 | _buff[1]);
  *gY = sign[1]*(_buff[2] << 8 | _buff[3]); 
  *gZ = sign[2]*(_buff[4] << 8 | _buff[5]);
}

void ITG3200::readGyroRaw( int *_GyroXYZ){
  readGyroRaw(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200::setScaleFactor(double _Xcoeff, double _Ycoeff, double _Zcoeff, bool _Radians) { 
  scalefactor[0] = 14.375 * _Xcoeff;   
  scalefactor[1] = 14.375 * _Ycoeff;
  scalefactor[2] = 14.375 * _Zcoeff;    
    
  if (_Radians){
    scalefactor[0] /= 0.0174532925;//0.0174532925 = PI/180
    scalefactor[1] /= 0.0174532925;
    scalefactor[2] /= 0.0174532925;
  }
}

void ITG3200::setOffsets(double _Xoffset, double _Yoffset, double _Zoffset) /* Global axis */
{
  if(_Xoffset != NULL)
    offsets[0] = _Xoffset;
  if(_Yoffset != NULL)
    offsets[1] = _Yoffset;
  if(_Zoffset != NULL)
    offsets[2] = _Zoffset;
}

void ITG3200::zeroCalibrate(unsigned int totSamples, unsigned int sampleDelayMS) {
  double tmpOffsets[] = {0,0,0};
  int xyz[3];

  for (int i = 0; i < totSamples; i++)
  {
    readGyroRaw(xyz);
    delay(sampleDelayMS);
    tmpOffsets[0] += xyz[0];
    tmpOffsets[1] += xyz[1];
    tmpOffsets[2] += xyz[2];
  }
  
  setOffsets(-tmpOffsets[0]/totSamples + 0.5, -tmpOffsets[1]/totSamples + 0.5, -tmpOffsets[2]/totSamples + 0.5);
  
//  delay(1000);
}

void ITG3200::readGyroRawCal(int *_GyroX, int *_GyroY, int *_GyroZ) { 
  readGyroRaw(_GyroX, _GyroY, _GyroZ); 
  *_GyroX += offsets[0]; 
  *_GyroY += offsets[1]; 
  *_GyroZ += offsets[2]; 
} 

void ITG3200::readGyroRawCal(int *_GyroXYZ) 
{ 
  readGyroRawCal(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2); 
} 

void ITG3200::readGyro(double *_GyroX, double *_GyroY, double *_GyroZ)
{
  int x, y, z; 
  readGyroRawCal(&x, &y, &z); // x,y,z will contain calibrated integer values from the sensor 
  #ifdef SMOOTH_GYRO_DATA
    *_GyroX = SMOOTH_FACTOR*(*_GyroX) + (1-SMOOTH_FACTOR)*(x / scalefactor[0]); 
    *_GyroY = SMOOTH_FACTOR*(*_GyroY) + (1-SMOOTH_FACTOR)*(y / scalefactor[1]); 
    *_GyroZ = SMOOTH_FACTOR*(*_GyroZ) + (1-SMOOTH_FACTOR)*(z / scalefactor[2]);   
  #else
    *_GyroX = x / scalefactor[0]; 
    *_GyroY = y / scalefactor[1]; 
    *_GyroZ = z / scalefactor[2];
  #endif
} 

void ITG3200::readGyro(double *_GyroXYZ)
{
  readGyro(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}

void ITG3200::reset() {     
  i2c.mem_write(_dev_address, PWR_MGM, PWRMGM_HRESET); 
  delay(GYROSTART_UP_DELAY); //gyro startup 
}

bool ITG3200::isLowPower() {   
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_SLEEP) >> 6;
}
  
void ITG3200::setPowerMode(bool _State) {
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  i2c.mem_write(_dev_address, PWR_MGM, ((_buff[0] & ~PWRMGM_SLEEP) | _State << 6));  
}

bool ITG3200::isXgyroStandby() {
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_XG) >> 5;
}

bool ITG3200::isYgyroStandby() {
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_YG) >> 4;
}

bool ITG3200::isZgyroStandby() {
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_STBY_ZG) >> 3;
}

void ITG3200::setXgyroStandby(bool _Status) {
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  i2c.mem_write(_dev_address, PWR_MGM, ((_buff[0] & PWRMGM_STBY_XG) | _Status << 5));
}

void ITG3200::setYgyroStandby(bool _Status) {
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  i2c.mem_write(_dev_address, PWR_MGM, ((_buff[0] & PWRMGM_STBY_YG) | _Status << 4));
}

void ITG3200::setZgyroStandby(bool _Status) {
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  i2c.mem_write(_dev_address, PWR_MGM, ((_buff[0] & PWRMGM_STBY_ZG) | _Status << 3));
}

uint8 ITG3200::getClockSource() {  
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  return (_buff[0] & PWRMGM_CLK_SEL);
}

void ITG3200::setClockSource(uint8 _CLKsource) {   
  i2c.mem_read(_dev_address, PWR_MGM, 1, &_buff[0]);
  i2c.mem_write(_dev_address, PWR_MGM, ((_buff[0] & ~PWRMGM_CLK_SEL) | _CLKsource)); 
}

int ITG3200::Xaxis()
{
  return X_axis;
}

int ITG3200::Yaxis()
{
  return Y_axis;
}

int ITG3200::Zaxis()
{
  return Z_axis;
}
