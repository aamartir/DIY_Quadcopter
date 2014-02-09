#ifndef _QUAD_H_
#define _QUAD_H_

#include "arduino.h"
#include "pins_arduino.h"

#define uint8 unsigned char
#define int8 char
#define uint16 unsigned short
#define int16 short
#define uint32 unsigned long
#define int32 long

#define _180_OVER_PI (180/PI)
#define _PI_OVER_180 (PI/180)
#define _2_PI (2*PI)

#define SET_BIT(_byte, _bit) (_byte |= _bit)
#define CLEAR_BIT(_byte, _bit) (_byte &= ~_bit)
#define GET_BIT(_byte, _bit) (_byte & _bit)

#define TO_DEGREES(val) (val*_180_OVER_PI)
#define TO_RADIANS(val) (val*_PI_OVER_180)

#define BAUD           115200
#define LEDPIN         13
#define ON             1
#define OFF            0

/* ===================== MOTORS ========================== */

/* Motor pins (PORTB, 8, 9, 10, 11) */
#define MOTOR1         0 //8 /* Opposite to MOTOR3 */
#define MOTOR2         1 //9 /* Opposite to MOTOR4 */
#define MOTOR3         2 //10
#define MOTOR4         3 //11

#define MOTOR1_MASK    (1 << MOTOR1)
#define MOTOR2_MASK    (1 << MOTOR2)
#define MOTOR3_MASK    (1 << MOTOR3)
#define MOTOR4_MASK    (1 << MOTOR4)

/* ======================================================= */

/* ===================== RADIO =========================== */

#define RADIO_INT_PIN  2 /* Digital pin PD2 */
#define RADIO_PIN_MASK (1 << RADIO_INT_PIN)

/* ======================================================= */

/* Sonar */
#define SONAR_PIN      4 /* Digital pin PB4 (Note: port B, not port D!) */
#define SONAR_PIN_MASK (1 << SONAR_PIN)

/* LED pins */
#define BLUE_LED_PIN   5 /* Digital pin PD5 */
#define RED_LED_PIN    6 /* Digital pin PD6 */

/* Basic axis definitions
 *  {RH, RV, LV, LH}  Joystick -> RH stands for Right Horizontal, LV stands for Left Vertical, and so on.
 *   RH : Roll
 *   RV : Pitch
 *   LV : Throttle
 *   LH : Unused (Maybe camera pan)
 */
#define ROLL           0
#define PITCH          1
#define YAW            2
#define ALTITUDE       3

/* Smoothing filter parameters */
#define GYRO_SMOOTH_FACTOR   0.8
#define ACC_SMOOTH_FACTOR    0.8
#define CALIBRATION_LOOP     100

//extern volatile unsigned int radio_data[RADIO_MAX_CHANNELS]; 


/**************************************************************/
/******************* Loop timing parameters *******************/
/**************************************************************/
#define RECEIVER_LOOPTIME_MS   50
#define CONTROL_LOOPTIME_MS     0
#define SENSOR_LOOPTIME_MS      0
#define RECEIVER_LOOPTIME_US   (RECEIVER_LOOPTIME_MS*1000) //20 Hz
#define CONTROL_LOOPTIME_US    (CONTROL_LOOPTIME_MS*1000)
#define SENSOR_LOOPTIME_US     (SENSOR_LOOPTIME_MS*1000)
#define SONAR_LOOPTIME_MS      50 /* milliseconds */

typedef struct _motor_node
{
  volatile uint16 pwm;
  volatile uint8 port;
  uint8 done;
} MOTOR_NODE;

/* Keep track of global time */
extern double dt_ms; 
extern double dt_sec;
extern unsigned long last_us;

/* Times in milliseconds */
extern unsigned long previousTime;
extern unsigned long currentTime;
extern unsigned long receiverTime;
extern unsigned long sensorTime;
extern unsigned long controlTime; // offset control loop from analog input loop by 1ms
extern unsigned int sonarTime_ms;
extern unsigned int sonar_last_ms;

/* Autoland feature */
extern volatile uint8 autoLand;

extern void RadioFrameInterrupt();
extern void setupTimerInterrupt();

// Function declaration
extern double mapToDouble(double x, double in_min, double in_max, double out_min, double out_max);
extern void gpio_set(uint8 pin);
extern void gpio_clear(uint8 pin);
extern void gpio_value(uint8 val, uint8 pin);
extern void quickSort(MOTOR_NODE * motors, int array_size);
extern void q_sort(MOTOR_NODE * motors, int left, int right);

//void readSensors(); // defined in Sensors.pde
//void RadioFrameInterrupt(void); // defined in Receiver.pde
//void readReceiverData(void); // defined in Receiver.pde
//int findStatisticalMode(int *data, int arraySize); // defined in Sensors.pde

#endif
