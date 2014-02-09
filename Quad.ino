#include <Arduino.h>
#include <stdio.h>
#include <avr/io.h>
#include "Quad.h" // Most declaractions
#include "Timer.h"
#include "LEDManager.h"
#include "Wire.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "Kalman.h"
#include "CONTROL_PARAMS.h" // PID 
#include "ESC.h" // Motor Interrupts
#include "Radio_PCINT.h"
#include "Sonar.h"

//#define ISR_DEBUG              1
//#define USE_SONAR              1
//#define SONAR_DEBUG            1
//#define ALTITUDE_CONTROL_TEST  1
//#define ESC_CALIBRATION        1
//#define MOTOR_VIBRATION_TEST   1
//#define RADIO_TEST             1

#ifdef MOTOR_VIBRATION_TEST
uint8 motor_counter;
uint16 motors_last_value[MOTORS];
#endif  

void printAccelData();
void printSensorData();

/* Sensor data */
double accel_data[3];
double gyro_data[3];
double compass_data[3];
double kalman_data[3];

/* Objects */
QUAD_STATE quad;
ADXL345 Accel;// = ADXL345();
ITG3200 Gyro;// = ITG3200();
HMC5883L Compass;// = HMC5883L();
Kalman kalman;// = Kalman();
Radio_PCINT radio;

#ifdef USE_SONAR
  Sonar sonar;
#endif

uint16 TIM16_ReadTCNT1( void );
void TIM16_WriteTCNT1( uint16 val );
void OCR1A_Write( uint16 val );
void configurePINS( void );
void timer2_setup( void );

/* LEDManager object */
LEDManager led_manager;

/* Timers */
Timer led_timer(&led_timer_handler);
Timer radio_timer(&radio_timer_handler);
Timer motor_timer(&motor_timer_handler);

unsigned long previousTime = 0;
unsigned long currentTime  = 0;
unsigned long receiverTime = 5;
unsigned long sensorTime   = 0;
unsigned long controlTime  = 1; // offset control loop from analog input loop by 1ms
unsigned int sonarTime_ms  = 0;
unsigned int sonar_last_ms = 0;

unsigned long last_us;
double dt_ms; 
double dt_sec;
uint8 now_ms;

/* Angle control */
float PID_out[4];     // {roll, pitch, yaw, altitude}
float error[4];  // {Error_roll, Error_pitch, Error_yaw}
float last_error[4];
float error_int[4];  // {Roll_Integration, Pitch_Int, Yaw_Int}
int target[2];        // {roll, pitch}

/* Heading control */
double targetHeading;
double headingIncr;

/* Throttle and altitude (Sonar) */
float PID_out_alt; 

#ifdef USE_SONAR
  float sonar_altitude;
  float sonar_alt_last;
  float target_altitude;
  float error_alt;      /* Error */
  float error_int_alt;  /* Error Integration */
  float error_last_alt; /* Error rate of change */
  float error_alt_der;
  float sonar_alt_der;
  float sonar_error;
#endif

float tmp;

uint16 throttle;

/////////////////////////////////////////////
MOTOR_NODE motors[MOTORS];
MOTOR_NODE middle_buffer[MOTORS];
volatile MOTOR_NODE ready_buffer[MOTORS];

volatile uint8 buffer_rdy;
volatile uint8 mCounter;

volatile uint8 cnt1;
volatile uint8 cnt2;
/////////////////////////////////////////////

void setup()
{   
  Serial.begin(BAUD); 
  Wire.begin();
  configurePINS();
  
  /* Initialize Quad state */
  quad.quad_mode = QUAD_NORMAL_MODE;
  quad.motors_armed = MOTORS_DISARMED;
  
  //Serial.println("Initializing... ");
  for(int i = 0; i < 10; i++)
  {
    gpio_clear(BLUE_LED_PIN);
    delay(100);
    gpio_set(BLUE_LED_PIN);
    delay(100);
  }

#if (!defined(ESC_CALIBRATION) && !defined(RADIO_TEST) && !defined(ISR_DEBUG) && !defined(SONAR_DEBUG))
  // Sensors initialize
  Serial.print("Powering Accelerometer...");
  Accel.powerOn();
  delay(10);
  Serial.println("OK");

  Serial.print("Powering Gyro... ");
  Gyro.init(ITG3200_ADDR_AD0_LOW);
  delay(5);
  Gyro.zeroCalibrate(500, 10); 
  delay(10);
  Serial.println("OK");

  Serial.print("Powering Compass... ");
  Compass.init();
  delay(10);
  Serial.println("OK");

  Serial.print("Initializing Kalman filters... "); 
  kalman.initialize();
  Serial.println("OK"); 
#endif

  // Initialize all motors to minimum throttle
  //motorPWM[0] = MOTOR_MIN;
  //motorPWM[1] = MOTOR_MIN;
  //motorPWM[2] = MOTOR_MIN;
  //motorPWM[3] = MOTOR_MIN;

  ////////////////////
  memset((MOTOR_NODE *) &motors, 0, sizeof(MOTOR_NODE) * MOTORS);
  motors[0].port = MOTOR1_MASK;
  motors[1].port = MOTOR2_MASK;
  motors[2].port = MOTOR3_MASK;
  motors[3].port = MOTOR4_MASK;
  ////////////////////

#ifdef MOTOR_VIBRATION_TEST
  motor_counter = 2 
  motors_last_value[1] = MOTOR_MIN;
  motors_last_value[2] = MOTOR_MIN;
  motors_last_value[3] = MOTOR_MIN;
#endif

  // target angles init
  target[0] = 0;
  target[1] = 0;
  targetHeading = 0; /* Yaw */
  
  throttle = MOTOR_MIN;

  error_int[0] = 0;
  error_int[1] = 0;
  error_int[2] = 0;

  // Configure TIMER1 to start generating proper PWM signals
  cli();
  setupTimerInterrupt();

  // Configure TIMER2 for the LED sequence
  timer2_setup();
  
#if ( defined(USE_SONAR) || #defined(SONAR_DEBUG) )
  Serial.print("Initializing Sonar... ");
  sonar.sonarInit();
  
  /* May not need the following variables, 
   * since they are all contained in sonar class */
  sonar_altitude = 0;
  error_alt      = 0;  /* Error */
  error_int_alt  = 0;  /* Error Integration */
  error_last_alt = 0;  /* Last error */
  error_alt_der  = 0;  /* Derivative of alt error */
  
  Serial.println("OK");
#endif 

  // Radio decoder works by PCINT
  radio.init();
  sei();
  
  //Signal that init is DONE 
  gpio_clear(BLUE_LED_PIN);

  // Setup LED Timer 
  led_manager.setCurrentPattern( &motors_disarmed_pattern, BLUE_LED_PIN );
 
  // last time (for timing control) 
  last_us = micros();
}


void configurePINS()
{   
  // RC PPM signal pin
  pinMode(RADIO_INT_PIN, INPUT); 

  // MOTORS (ESC) POR
  pinMode(MOTOR1 + 8, OUTPUT); // motorPWM[0] 
  pinMode(MOTOR2 + 8, OUTPUT); // motorPWM[1]
  pinMode(MOTOR3 + 8, OUTPUT); // motorPWM[2] 
  pinMode(MOTOR4 + 8, OUTPUT); // motorPWM[3] 

  // Sonar 
#ifdef USE_SONAR
  //pinMode(SONAR_PIN + 8, INPUT); /* Pin changes direction (INPUT/OUTPUT) */
#endif

  // Status LEDs
  pinMode(RED_LED_PIN, OUTPUT); 
  pinMode(BLUE_LED_PIN, OUTPUT); 
}

void loop()
{
  // Get new time differential
  dt_ms = (micros() - last_us)/1000.0; /* milli-seconds */
  dt_sec = dt_ms/1000.0;
  last_us = micros();

  /* Read radio commands */
  if( radio.data_ready )
  {
    radio.readReceiverData();
    
    /* ARM motors if following joystick positions are satisfied:
     * LEFT JOYSTICK: DOWN and to the LEFT completely
     * RIGHT JOYSTICK: UP and to the RIGHT completely
     */
    if( quad.motors_armed == MOTORS_DISARMED )
    {
      if( radio.data_rdy_buffer[RADIO_ROLL_CH]     >= (ROLL_MAX - 100) &&
          radio.data_rdy_buffer[RADIO_PITCH_CH]    >= (PITCH_MAX - 100) &&
          radio.data_rdy_buffer[RADIO_THROTTLE_CH] <= (THTLE_MIN + 100) && 
          radio.data_rdy_buffer[RADIO_YAW_CH]      <= (YAW_MIN + 100) )
      {
        quad.motors_armed = MOTORS_ARMED;
        
        /* Change LED pattern */
        led_manager.setCurrentPattern( &simple_on_off_pattern, BLUE_LED_PIN );
      }
    }
    else
    {
      /* If autoland was enabled due to radio signal lost, 
         once signal is available again, disable autoland */
      if( quad.quad_mode == QUAD_AUTOLAND_MODE )
      {
        quad.quad_mode = QUAD_NORMAL_MODE;
        
        /* Change LED pattern */
        led_manager.setCurrentPattern( &simple_on_off_pattern, BLUE_LED_PIN );
      }
    }
  }
  else if( radio.state & RADIO_SIGNAL_LOST )
  {
    /* Set motor timer to start decreasing throttle */
    if( quad.quad_mode != QUAD_AUTOLAND_MODE )
    {
      quad.quad_mode = QUAD_AUTOLAND_MODE;
      motor_timer.setTimerVal( MOTOR_TIMER_PERIOD );
      
      /* Change LED pattern */
      led_manager.setCurrentPattern( &_1_2_3_pattern, BLUE_LED_PIN );
    }
    /* Once throttle is 0, Disengage motors until radio signal is stable again */
    else if( throttle <= (MOTOR_MIN + 100) )
    {
      quad.motors_armed = MOTORS_DISARMED;
    }
  }
  

  /* Read sensors */

  /* Read accelerometer */
  if( Accel.initialized )
    Accel.readAccel(); /* Produces angles in radians (stored in Accel.angle[...] )*/

  /* Read gyro */
  if( Gyro.initialized )
  {
    if(Gyro.isRawDataReady())
      Gyro.readGyro(gyro_data);
  }
  
  /* Calculate Kalman angles for ROLL and PITCH */
  if( Accel.initialized && Gyro.initialized )
  {
    kalman.calculate(kalman.Xaxis(), TO_DEGREES(Accel.angle[Accel.Xaxis()]), gyro_data[Gyro.Xaxis()], dt_sec);
    kalman.calculate(kalman.Yaxis(), TO_DEGREES(Accel.angle[Accel.Yaxis()]), gyro_data[Gyro.Yaxis()], dt_sec);
  }
  
  // Use compass only if device is not so tilted. Otherwise, loss of sensitivity causes wrong heading results.
  //if(abs(kalman.getAngle(kalman.Xaxis())) > 30 || abs(kalman.getAngle(kalman.Yaxis())) > 30)
  //{
#ifdef USE_SONAR
  if(abs(gyro_data[Gyro.Zaxis()]) > 2.5 && (throttle >= QUAD_HOVER || sonar_altitude >= MIN_SONAR_DISTANCE /* inches*/ )) /* Eliminate some gyro noise */
#else
  if(abs(gyro_data[Gyro.Zaxis()]) > 1.5 && (throttle >= QUAD_HOVER)) /* Eliminate some gyro noise */
#endif
  {
    Compass.heading += gyro_data[Gyro.Zaxis()]*dt_sec; /* Gyro-only orientation */
  }

  //    if(Compass.heading < -PI)
  //      Compass.heading += _2_PI;
  //    else if(Compass.heading > PI)
  //      Compass.heading -= _2_PI;
  //}
  //if(Compass.isDataReady()) /* Otherwise just use compass */
  {
    //Compass.getHeading_tiltCompensate(kalman.getAngle(kalman.Xaxis()),   /* Kalman X axis */
    //                                  kalman.getAngle(kalman.Yaxis()));  /* Kalman Y axis */

    /* Calculate Kalman for YAW */
    //kalman.calculate(kalman.Zaxis(), Compass.heading, gyro_data[Gyro.Zaxis()], dt_sec);
    //Compass.readScaledAxis();
  }

  //if(abs(target[YAW]) > 0 || throttle < QUAD_MIN_HOVER)
  //     gyro.setHeading(0);
  //else if(yawBiasFound && throttle >= QUAD_MIN_HOVER) // Only calculate yaw when throttle is high enough          
  //     gyro.calculateHeading();

  //    sensorTime = currentTime;
  //}

  /* Sonar looptime 1/15ms = 66 Hz */
#ifdef USE_SONAR
  //sonarTime_ms = millis() - sonar_last_ms;
  //if(sonar.getSonarState() == SONAR_INACTIVE || (sonarTime_ms >= SONAR_LOOPTIME_MS && sonar.getSonarState() != SONAR_DATA_AVAIL))
  //{
  /* Only use sensor if Quad is steady */
  //  if(abs(kalman.getAngle(kalman.Xaxis())) < 20 && abs(kalman.getAngle(kalman.Yaxis())) < 20)
  //  {
  //    sonar.sonarPulse();
  //    sonar_last_ms = millis();
  //  }
  //}
  
  if( sonar.isSonarDataAvailable() )
  {
    //sonar_alt_last = sonar_altitude; /* Keep track of the last value */
    //tmp = sonar.getSonarAltitude_cm( dt_sec );
    //sonar_error = tmp - sonar_altitude;

    // Average between the last and current derivative 
    //sonar_alt_der = 0.5*(sonar_alt_der + (tmp - sonar_alt_last)); 
    
    /* Map the joystick value to a target altitude */
    target_altitude = mapToDouble((double) throttle,   
                                  (double) QUAD_HOVER,   
                                  (double) MOTOR_MAX,   
                                  MIN_SONAR_DISTANCE,   
                                  MAX_SONAR_DISTANCE);

    /* Constrain target altitude */
    target_altitude = constrain( target_altitude, MIN_SONAR_DISTANCE, MAX_SONAR_DISTANCE );  

    //error_last_alt = error[ALTITUDE];  /* Save previous error */
    //error[ALTITUDE] = target_altitude - sonar_altitude; /* New error */
    //error_alt_der = 0.9*error_alt_der - 0.1*(error[ALTITUDE] - error_last_alt)/dt_sec; /* Derivative of error */


   // if(error_int_alt > -200 && error_int_alt < 500)
   //   error_int_alt += error[ALTITUDE]*dt_sec; /* Integrate error */

  }
  else
  {
    // Estimates in-between
    //sonar_altitude += 0.03*(sonar_alt_der + sonar_error); // 0.035
  }

#endif

  //--------------------------------------------------------------
  //--------------------------------------------------------------

  // PID Control algorithm
  // Run PID and motor update at 500 Hz
  //if(micros() >= controlTime + CONTROL_LOOPTIME_US)
  //{
  // Error (P term) -> Using Kalman
  error[ROLL]  = kalman.getAngle(kalman.Xaxis()) + target[ROLL];   // roll error (inverted)
  error[PITCH] = kalman.getAngle(kalman.Yaxis()) + target[PITCH];  // pitch error (inverted)
  error[YAW]   = Compass.heading - targetHeading;  // yaw error

  // Error integration (I term)
//#ifdef IGNORE
  /* Start integrating the errors if the 
   * throttle is above the dead-zone. */
  if( throttle > QUAD_HOVER )
  {
     error_int[ROLL]  += error[ROLL] * dt_sec;
     error_int[PITCH] += error[PITCH] * dt_sec;
     
     /* Constrain integration error */
     error_int[ROLL]  = (float) constrain( error_int[ROLL], -2000.0, 2000.0 );
     error_int[PITCH] = (float) constrain( error_int[PITCH], -2000.0, 2000.0 );
  }
  else
  {
     error_int[ROLL]  = 0;
     error_int[PITCH] = 0;
  }
//#endif

  //if(abs(gyro.getHeading) < 1)    
  //    error_int[YAW] += error[YAW]*G_dt;

  // Error rate (D term): unbiased gyro
  PID_out[ROLL]  = PID_gains_roll[PID_P_TERM]  * error[ROLL]  + PID_gains_roll[PID_I_TERM]  * error_int[ROLL]      + PID_gains_roll[PID_D_TERM]  * gyro_data[Gyro.Xaxis()];    // x axis (roll angle)
  PID_out[PITCH] = PID_gains_pitch[PID_P_TERM] * error[PITCH] + PID_gains_pitch[PID_I_TERM] * error_int[PITCH]     + PID_gains_pitch[PID_D_TERM] * gyro_data[Gyro.Yaxis()];  // y axis (pitch angle)
  PID_out[YAW]   = PID_gains_yaw[PID_P_TERM]   * error[YAW] /*  + PID_gains_yaw[PID_I_TERM] * error_int[YAW]  */   + PID_gains_yaw[PID_D_TERM]   * gyro_data[Gyro.Zaxis()]; // PD Controller: yaw

  /* Sonar PID */
#ifdef USE_SONAR
  PID_out[ALTITUDE] = PID_gains_alt[PID_P_TERM] * error[ALTITUDE] + 
                      PID_gains_alt[PID_I_TERM] * error_int_alt + 
                      PID_gains_alt[PID_D_TERM] * error_alt_der;

  /* Constrain altitude control */
  PID_out[ALTITUDE] = constrain( PID_out[ALTITUDE], -250, 1000 );
#endif 
   
  if((throttle <= MOTOR_MIN + MOTORS_DEAD_ZONE) || throttle > (MOTOR_MAX << 1))
  {
    /* Clear completely, may save some power. This creates close 0 volts since duty % = 0, thus saving battery life (I think...) */
    motors[0].pwm = MOTOR_MIN;
    motors[1].pwm = MOTOR_MIN;
    motors[2].pwm = MOTOR_MIN;
    motors[3].pwm = MOTOR_MIN;

    error_int[ROLL]  = 0;
    error_int[PITCH] = 0;
    error_int[YAW]   = 0;

  #ifdef USE_SONAR
    /* Sonar clear */
    error_int_alt = 0;
    target_altitude = 0;
    PID_out[ALTITUDE] = 0;
  #endif
  
    /* If the copter is on the ground (motor_PWM = MIN), then force the heading to 0 */
    Compass.heading = 0;
    targetHeading = 0;
  }
  else if( quad.motors_armed == MOTORS_ARMED &&
           throttle <= MOTOR_MAX ) /* Here's the Motor control algorithm */
  { 
  #ifdef MOTOR_VIBRATION_TEST
    motors[motor_counter].pwm = throttle;
    motors_last_value[motor_counter] = throttle;
  #elif ESC_CALIBRATION
    motors[0].pwm = throttle;
    motors[1].pwm = throttle;
    motors[2].pwm = throttle;
    motors[3].pwm = throttle;
  #else
    // roll (x_axis)
    motors[0].pwm = throttle + PID_out[ROLL] - PID_out[PITCH] - PID_out[YAW]; 
    motors[1].pwm = throttle + PID_out[ROLL] + PID_out[PITCH] + PID_out[YAW]; 

    // pitch (y_axis)
    motors[2].pwm = throttle - PID_out[ROLL] + PID_out[PITCH] - PID_out[YAW]; 
    motors[3].pwm = throttle - PID_out[ROLL] - PID_out[PITCH] + PID_out[YAW]; 
  #endif
  
    /* Constrain motor pwm values */
    motors[0].pwm = constrain(motors[0].pwm, MOTOR_MIN, MOTOR_MAX);
    motors[1].pwm = constrain(motors[1].pwm, MOTOR_MIN, MOTOR_MAX);  
    motors[2].pwm = constrain(motors[2].pwm, MOTOR_MIN, MOTOR_MAX); 
    motors[3].pwm = constrain(motors[3].pwm, MOTOR_MIN, MOTOR_MAX);
  }

  buffer_rdy = 0;
  middle_buffer[0] = motors[0];
  middle_buffer[1] = motors[1];
  middle_buffer[2] = motors[2]; 
  middle_buffer[3] = motors[3]; 
  buffer_rdy = 1;

  //controlTime = currentTime;
  //}

  //radio.printRadio();
  
  //printAccelData();
  //printSensorData();
  //delay(5);
}

void printAccelData()
{
   Serial.print(Accel.acceleration[0]);
   Serial.print("\t");
   Serial.print(Accel.acceleration[1]);
   Serial.print("\t");
   Serial.println(Accel.acceleration[2]);
}

void printSensorData()
{
  #ifdef IGNORE
  Serial.print(kalman.getAngle(kalman.Zaxis()));
  Serial.print(" ");
  Serial.println(Compass.heading);
  #endif
  
  //#ifdef IGNORE
  
  //Serial.print(" ");
  //Serial.println(kalman.getAngle(kalman.Yaxis()));
  //#endif
  
  Serial.print(Accel.angle[Accel.Xaxis()]);
  Serial.print(" ");
  Serial.print(gyro_data[Gyro.Xaxis()]);
  Serial.print(" ");
  Serial.println(kalman.getAngle(kalman.Xaxis()));
  
   //Serial.print("\t");
   //Serial.print(motorPWM[motor_counter]);
   //Serial.print("\t");
   //Serial.print(motor_counter);
   //Serial.print("\t");
   //Serial.print(kalmanXYZ.getAngle(kalmanXYZ.Zaxis())); /* Heading */
  //Serial.print("\t");
  /*Serial.print(Compass.heading); 
   Serial.print("\t");
   Serial.print(kalman.getAngle(kalman.Zaxis())); 
   Serial.print("\t");
   Serial.print(headingIncr);
   Serial.print("\t");
   Serial.print(targetHeading); 
   Serial.print("\t");
   Serial.print(PID_out[YAW]);
   Serial.println("\t");*/
   
   #ifdef IGNORE
   Serial.print(degrees(Accel.angle[Accel.Xaxis()]));
   Serial.print("\t");
   Serial.print(degrees(Accel.angle[Accel.Yaxis()]));
   Serial.print("\t");
   Serial.println(degrees(Accel.acceleration[Accel.Zaxis()]));
   #endif
   
   #ifdef IGNORE
   Serial.print(gyro_data[Gyro.Xaxis()]);
   Serial.print("\t");
   Serial.print(gyro_data[Gyro.Yaxis()]);
   Serial.print("\t");
   Serial.println(gyro_data[Gyro.Zaxis()]);
   #endif
   
   //Serial.print("\t");
  /*Serial.print(Compass.heading); 
   Serial.print("\t");
   Serial.print(throttle); 
   Serial.println();*/

#ifdef USE_SONAR
  Serial.print(sonar_altitude);
  Serial.print("\t");
  Serial.print(error_alt_der);
  //Serial.print("\t");
  //Serial.print(PID_out[ALTITUDE]);
#endif
}

ISR(TIMER1_CAPT_vect)
{
  if(buffer_rdy)
  {
    ready_buffer[0].pwm  = middle_buffer[0].pwm;
    ready_buffer[0].port = middle_buffer[0].port;
    
    ready_buffer[1].pwm  = middle_buffer[1].pwm;
    ready_buffer[1].port = middle_buffer[1].port;
    
    ready_buffer[2].pwm  = middle_buffer[2].pwm;
    ready_buffer[2].port = middle_buffer[2].port;
    
    ready_buffer[3].pwm  = middle_buffer[3].pwm;
    ready_buffer[3].port = middle_buffer[3].port;
    
    // Clear flag
    buffer_rdy = 0;
  }
  
  OCR1A = ready_buffer[cnt1].pwm;
  OCR1B = ready_buffer[cnt2].pwm;
  
  PORTB |= (ready_buffer[cnt1].port |
            ready_buffer[cnt2].port); 
}

ISR(TIMER1_COMPA_vect)
{
  PORTB &= ~(ready_buffer[cnt1].port);
  
  if(cnt1 == 0)
    cnt1 = 1;
  else
    cnt1 = 0;
}

ISR(TIMER1_COMPB_vect)
{
  PORTB &= ~(ready_buffer[cnt2].port);
  
  if(cnt2 == 2)
    cnt2 = 3;
  else
    cnt2 = 2;
}

void setupTimerInterrupt()
{
  // Pre-scaler Clock/8. Fast-PWM 16-bits. TOP = ICR1
  TCCR1A = 0;   
  TCCR1B = 0;
  TIMSK1 = 0;
  
  TCCR1B |= _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  // Output compare flag set on Compare match.
  TIMSK1 |= _BV(ICIE1) | _BV(OCIE1A) | _BV(OCIE1B);// | _BV(TOIE1);// | 

  /* Reset TIMER1 COUNTER register */
  TCNT1 = 0;

  /* Timer 1 TOP (ICR1 register) 
   * 0xfa0 hex = 4000 decimal = gives 2*2ms = 4ms periods
   * 0x1f40 = 8 ms periods
   * 0x2710 = 10 ms periods (tested)
   */
  ICR1 = 0x7fff; /* Highest byte */ 
}

void timer2_setup()
{
  /* Setup: Disable timer 2 while we set it up */
  TCCR2B = 0;

  /* Setup registers 
   * TCCR2A Timer/Counter Control Register A 
   * COM2A1(0) COM2A0(0): Normal port operation, OC2A disconnected
   * COM2B1(0) COM2B0(0): Normal port operation, OC2B disconnected
   * WGM1(0) WGM0(0): Normal mode, TOP(0xFF)
   */
  TCCR2A = 0; 

  /* TCCR2B Timer/Counter Control Register B 
   * WGM2(0): Normal mode, TOP(0xFF)
   * CS22(1) CS21(0) CS20(0): Prescaler = 64
   * Period = (256)(N)/freq = 256(64)/16Mhz = 1.024 ms
   *
   *   CS22      CS21      CS20       Description
   *   ------------------------------------------
   *   0         0         0          No clk source
   *   0         0         1          Clk/1 (No Pre-scaling)
   *   0         1         0          Clk/8
   *   0         1         1          Clk/32
   *   1         0         0          Clk/64
   *   1         0         1          Clk/128
   *   1         1         0          Clk/256
   *   1         1         1          Clk/1024 
   */
  TCCR2B |= (1 << CS22);

  /* Clear TCNT2 */
  TCNT2 = 0;

  /* Activate Overflow Interrupt Enable 
   * TIMSK2 - Timer/Counter2 Interrupt Mask Register
   * TOIE2(1)
   */
  TIMSK2 |= (1 << TOIE2);
}

/* Timer 2 Overflow interrupt routine */
ISR( TIMER2_OVF_vect )  /* Executes every millisecond. Routine has to execute fairly quickly, and when I say fairly I mean REALLY quickly. */
{  
  /* For all timers */
  uint8 t;

  for(t = 0; t < MAX_TIMERS; t++)
  {
    if(Timers[t]->val == 0)
      continue;

    /*
    if((Timers[t]->val) > 0 && Timers[t]->fptr)
     {
     if(--Timers[t]->val == 0) // Execute timer routine 
     Timers[t]->fptr();
     }
     */

    if(--Timers[t]->val == 0 && Timers[t]->fptr) // Execute timer routine 
      Timers[t]->fptr();
  }
}

volatile uint8 sonar_pin_last;
volatile uint8 barometer_pin_last;

ISR( PCINT0_vect ) /* PCINT0_vect (handles portB PCINT) */
{
  #ifdef USE_BAROMETER
  if( ((PINB >> BAROMETER_PIN) & 0x01) != barometer_pin_last )
  {
    // ...
    
    barometer_pin_last = (PINB >> BAROMETER_PIN) & 0x01;
  }
  #endif
  
  #ifdef USE_SONAR
  if( ((PINB >> SONAR_PIN) & 0x01) != sonar_pin_last )
  {
    if( (PINB & SONAR_PIN_MASK) > 0 )
    {
      sonar.setStartTime(micros());
      sonar.setSonarState(SONAR_PULSE_START); 
    }
    else if( sonar.getSonarState() == SONAR_PULSE_START ) /* Look for SONAR_PULSE_START */
    {
      sonar.setEchoTime(micros()); /* micros() - start_time */
      sonar.setSonarState(SONAR_DATA_AVAIL); /* Measurement found */
    }
    
    /* Save state of last pin */
    sonar_pin_last = PINB >> SONAR_PIN) & 0x01;
  }
  #endif
}
  
uint16 TIM16_ReadTCNT1( void )
{
  uint8 sreg;
  uint16 i;
  
  /* Save global interrupt flag */
  sreg = SREG;
  /* Disable interrupts */
  cli();  
  /* Read TCNT1 into i */
  i = TCNT1;
  /* Restore global interrupt flag */
  SREG = sreg;
  /* Return TCNT1 value */
  return i;
}

void TIM16_WriteTCNT1( uint16 val )
{
  uint8 sreg;

  /* Save global interrupt flag */
  sreg = SREG;
  /* Disable interrupts */
  cli();  
  /* Read TCNT1 into i */
  TCNT1 = val;
  /* Restore global interrupt flag */
  SREG = sreg;
}

void OCR1A_Write( uint16 val )
{
  uint8 sreg;

  /* Save global interrupt flag */
  sreg = SREG;
  /* Disable interrupts */
  cli();  
  /* Read TCNT1 into i */
  OCR1A = val;
  /* Restore global interrupt flag */
  SREG = sreg;
}

double mapToDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return ((x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min);
}

void gpio_set(uint8 pin)
{
  if(pin >= 8)
    PORTB |= (1 << (pin - 8));
  else
    PORTD |= (1 << pin);
}

void gpio_clear(uint8 pin)
{
  if(pin >= 8)
    PORTB &= ~(1 << (pin - 8));
  else
    PORTD &= ~(1 << pin);
}

void gpio_value(uint8 val, uint8 pin)
{
  if(val)
    gpio_set(pin);
  else 
    gpio_clear(pin);
}


