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
#ifndef ADXL345_h
#define ADXL345_h

#include "I2C.h"
//#include "Quad.h"

#define DEVICE_ID 0x53    // ADXL345 device address

/* ------- Register names ------- */
#define DEVID              0x00
#define RESERVED1          0x01
#define THRESH_TAP_RED     0x1d
#define OFSX_REG           0x1e
#define OFSY_REG           0x1f
#define OFSZ_REG           0x20
#define DUR_REG            0x21
#define LATENT_REG         0x22
#define WINDOW_REG         0x23
#define THRESH_ACT_REG     0x24
#define THRESH_INACT_REG   0x25
#define TIME_INACT_REG     0x26
#define ACT_INACT_CTL_REG  0x27
#define THRESH_FF_REG      0x28
#define TIME_FF_REG        0x29
#define TAP_AXES_REG       0x2a
#define ACT_TAP_STATUS_REG 0x2b
#define BW_RATE_REG        0x2c
#define POWER_CTL_REG      0x2d
#define INT_ENABLE_REG     0x2e
#define INT_MAP_REG        0x2f
#define INT_SOURCE_REG     0x30
#define DATA_FORMAT_REG    0x31
#define DATAX0_REG         0x32
#define DATAX1_REG         0x33
#define DATAY0_REG         0x34
#define DATAY1_REG         0x35
#define DATAZ0_REG         0x36
#define DATAZ1_REG         0x37
#define FIFO_CTL_REG       0x38
#define FIFO_STATUS_REG    0x39

/* Bandwidth configuration */
#define BW_1600            0x0F // 1111
#define BW_800             0x0E // 1110
#define BW_400             0x0D // 1101  
#define BW_200             0x0C // 1100
#define BW_100             0x0B // 1011  
#define BW_50              0x0A // 1010 
#define BW_25              0x09 // 1001 
#define BW_12              0x08 // 1000 
#define BW_6               0x07 // 0111
#define BW_3               0x06 // 0110

/* 
 Interrupt PINs
 INT1: 0
 INT2: 1
 */
#define INT1_PIN 0x00
#define INT2_PIN 0x01

/* 
 Interrupt bit position
 */
#define INT_DATA_READY_BIT 0x07
#define INT_SINGLE_TAP_BIT 0x06
#define INT_DOUBLE_TAP_BIT 0x05
#define INT_ACTIVITY_BIT   0x04
#define INT_INACTIVITY_BIT 0x03
#define INT_FREE_FALL_BIT  0x02
#define INT_WATERMARK_BIT  0x01
#define INT_OVERRUNY_BIT   0x00

#define OK    1 // no error
#define ERROR 0 // indicates error is predent

#define NO_ERROR   0 // initial state
#define READ_ERROR 1 // problem reading accel
#define BAD_ARG    2 // bad method argument

#define SMOOTH_ACCEL_DATA

#ifdef SMOOTH_ACCEL_DATA
#define SMOOTH_FACTOR 0.80 
#endif

/* Axis */
#define X_axis        0
#define Y_axis        1
#define Z_axis        2

class ADXL345
{
public:
  bool initialized;
  bool status;           // set when error occurs 
                         // see error code for details
  uint8 error_code;       // Initial state
  double gains[3];       // counts to Gs
  int sign[3];
  
  int raw[3];
  double acceleration[3]; /* Compensated */
  double angle[3];
  
  ADXL345();
  void powerOn();
  void readAccel(void);
  void getAngleRad(double *accel_data, double *angles);
  void getAngleDegrees(double *accel_data, double *angles);
  
  void setAxisGains(double *_gains);
  void getAxisGains(double *_gains);
  void setAxisOffset(int x, int y, int z);
  void getAxisOffset(int* x, int* y, int*z);

  double getRate();
  void setRate(double rate);
  void set_bw(uint8 bw_code);
  uint8 get_bw_code();  

  void getRangeSetting(uint8* rangeSetting);
  void setRangeSetting(int val);
  
  bool getFullResBit();
  void setFullResBit(bool fullResBit);
  void printAllRegister();

  int Xaxis();
  int Yaxis();
  int Zaxis();
  
private:
  void setRegisterBit(uint8 regAdress, int bitPos, bool state);
  bool getRegisterBit(uint8 regAdress, int bitPos);  
  uint8 _buff[6];    // 6 bytes buffer for saving data read from the device
};

void print_byte(uint8 val);

#endif
