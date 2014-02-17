#ifndef _BMP085_H
#define _BMP085_H

#include "Arduino.h"
#include "Wire.h"

//#define BMP085_DEBUG      1

/* Device Address */
#define DEV_ADDR          0x77

/* Configuration Parameters */
#define ULTRALOWPOWER     0
#define STANDARD          1
#define HIGHRES           2
#define ULTRAHIGHRES      3

/* Calibration Registers (16 bits each, MSB followed by LSB) */
#define CAL_AC1           0xAA  // MSB
#define CAL_AC2           0xAC  // LSB
#define CAL_AC3           0xAE  // and so on
#define CAL_AC4           0xB0  
#define CAL_AC5           0xB2  
#define CAL_AC6           0xB4  
#define CAL_B1            0xB6  
#define CAL_B2            0xB8  
#define CAL_MB            0xBA  
#define CAL_MC            0xBC  
#define CAL_MD            0xBE  

/* Registers */
#define CONTROL_REG       0xF4 
#define TEMPERATURE_REG   0xF6 /* Temperature data is stored in this register */
#define PRESSURE_REG      0xF6 /* Pressure data is stored in this register */ 

/* Commands */
#define READTEMP_CMD      0x2E
#define READPRESSURE_CMD  0x34

#define SMOOTH_ALTITUDE_DATA
#ifdef SMOOTH_ALTITUDE_DATA
  #define SMOOTH_FACTOR 0.80 
#endif

/* End of Conversion Interrupt pin */
#define EOC_PIN           2                 /* PD2 */
#define EOC_PIN_MASK      (1 << EOC_PIN)

/* State machine */

typedef struct _bmp085_state
{
  uint8_t val;              /* Read temp, pressure, etc */
  volatile uint8_t state;   /* Status of command: In progress, done, error, etc */
} BMP085_STATUS;

enum _states
{
  NO_OP,
  FETCHING,     /* Waiting for bmp085 to sample requested value */
  READY,        /* bmp085 has sampled requested value, and the appropriate register is ready to be read */
  DONE,
  MAX_STATES
};

enum _cmds
{
  NO_CMD,
  GET_RAW_TEMPERATURE_CMD,
  GET_RAW_PRESSURE_CMD,
  GET_REAL_TEMP_AND_PRESSURE_CMD,
  GET_ALTITUDE_CMD,
  MAX_CMDS
};


class BMP085 
{
  public:
    BMP085_STATUS curr_cmd;
    bool new_altitude_val_ready;
    
    BMP085();
    boolean init( uint8_t mode = ULTRAHIGHRES );  // by default go highres
    void update_state_machine( void );
    float getAltitude( void );
    //int32_t calibratePressureAtSeaLevel( void );
  
 private:
    uint8_t oversampling;
    uint16_t UT; /* Raw temperature */
    int32_t UP;  /* Raw pressure */
    int32_t pressure;
    int32_t pressureAtSeaLevel;
    int32_t temperature;
    float altitude;
    
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    
    void requestRawTemperature(void);
    void requestRawPressure(void);
    void rawTemperatureReadSrv( void );
    void rawPressureReadSrv( void );
    void calculateRealTempAndPressure( void );
    void updateAltitude( float sealevelPressure = 101325 ); // std atmosphere
};

extern BMP085 bmp;

#endif 
