/*
HMC5883L.h - Header file for the HMC5883L Triple Axis Magnetometer Arduino Library.
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

#ifndef HMC5883L_h
#define HMC5883L_h

#include "Quad.h"

/* Registers */
#define DEVICE_ID     0x1E
#define CRA_REG       0x00 /* Configuration Register A */
#define CRB_REG       0x01 /* Configuration Register B */
#define MODE_REG      0x02
#define DATA_OUT_REG  0x03
#define STATUS_REG    0x09

/* Measurement type */
#define CONTINUOUS    0x00
#define SINGLE_SHOT   0x01
#define IDLE          0x03

/* STATUS_REG bits */
#define RDY_BIT 0x01

/* Data Output Rates */
#define RATE_0_75_HZ  0
#define RATE_1_5_HZ   0x01
#define RATE_3_HZ     0x02
#define RATE_7_5_HZ   0x03
#define RATE_15_HZ    0x04
#define RATE_30_HZ    0x05
#define RATE_75_HZ    0x06

/* SCALE VALUES (Gauss) */
#define SCALE_0_88    0x00   /* 0.88 */
#define SCALE_1_3     0x01   /* 1.3 */
#define SCALE_1_9     0x02   /* 1.9 */
#define SCALE_2_5     0x03   /* 2.5 */
#define SCALE_4_0     0x04   /* 4.0 */
#define SCALE_4_7     0x05   /* 4.7 */
#define SCALE_5_6     0x06   /* 5.6 */
#define SCALE_8_1     0x07   /* 8.1 */

#define DECLINATION         -0.1053
#define SMOOTH_DATA_COMPASS  1
#define SMOOTH_FACTOR        0.90

/* Calibration parameters */
#define xoffset             -41.4
#define yoffset              246.1

/* Axis */
#define Y_axis               0
#define X_axis               1
#define Z_axis               2

class HMC5883L
{
  private:
    /* Gain per axis */
    float gains[3];   /* x, y, z */
    int raw[3];        /* x, y, z */
    
    float offset[3];
    float magScale[3];
    float magMax[3];
    float magMin[3];
    
  public:
    HMC5883L();  /* Constructor */
    void readRawAxis(int *buf);
    void readScaledAxis();
    void getHeading_tiltCompensate(double angleX, double angleY);
    void setGains(double x, double y, double z);
    void setGain(double val);
    void setMagCal(byte axis, float minVal, float maxVal);
    double getHeadingRad(double *data);
    double getHeadingDegrees(double *data);
    
    void init(void);
    void setDataOutputRate(uint8_t rate); 
    void setNumberOfSamplesAveraged(uint8_t samples);
    void readRegister(uint8_t addr, uint8_t *reg);
    int setMeasurementMode(uint8_t mode);
    int setScale(double gauss);
    bool isDataReady(void);
    
    int Xaxis();
    int Yaxis();
    int Zaxis();
    
    double heading;
    double scaled[3];  /* x, y, z */
    double tiltCompensated[3];
};

#endif
