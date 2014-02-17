#include "Quad.h"

// PID control algorithm
#define PID_P_TERM 0
#define PID_I_TERM 1
#define PID_D_TERM 2
 
const float PID_gains_roll[3]     = {35.0, 20.0, 25.0}; // {50.0, 30.0, 30.0};
const float PID_gains_pitch[3]    = {35.0, 20.0, 25.0};  //
const float PID_gains_yaw[3]      = {50.0, 0, 40.0}; // {60.0, 0, 40.0};

//#ifdef USE_SONAR
  const float PID_gains_alt[3]    = {0.5, 0.2, -0.2}; /* Sonar */
//#endif

extern float PID_out[4];     // {roll, pitch, yaw, altitude}
extern float error[4];       // {Error_roll, Error_pitch, Error_yaw, Error_alt}
extern float last_error[4];
extern float error_int[4];   // {Roll_Integration, Pitch_Int, Yaw_Int}

/* Sonar */
#ifdef USE_SONAR
  extern float error_alt;      /* Error */
  extern float error_int_alt;  /* Error Integration */
  extern float error_last_alt; /* Error rate of change */
  
  /* Altiltude (Sonar sensor) */
  extern float target_altitude;  /* Sonar */
#endif

// Target angle vector
extern int target[2];        // {roll, pitch}
extern double targetHeading;
extern double headingIncr;
extern unsigned short counter;
