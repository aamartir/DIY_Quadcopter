#include "ABG_Filter.h"

/* Constructor */
ABG_Filter::ABG_Filter()
{
  alfa = 0;
  beta = 0;
  gamma = 0;
}

void ABG_Filter::setFilterCoefficients( float new_a, float new_b, float new_g )
{
  alfa = new_a;
  beta = new_b;
  gamma = new_g;
}

float * ABG_Filter::getFilterCoefficients()
{
  float * coef = new float[3];
  
  coef[0] = alfa;
  coef[1] = beta;
  coef[2] = gamma;
  
  return coef;
}

void ABG_Filter::filter( float xm, float dt )
{
  float rk;
  
  /* Model equation */
  smatrix[0] += (smatrix[1] * dt) + (smatrix[2]*(dt*dt)); 			  /* Position */
  smatrix[1] += (smatrix[2] * dt);						  			  /* Velocity */
  smatrix[2] = smatrix[2];											  /* Acceleration (gamma) is projected */
  
  /* Error between measurement and model */
  rk = (xm - smatrix[0]);
  
  /* Update model with alfa, beta, gamma parameters 
   * in the direction of the gradient. */
  smatrix[0] += ( alfa*rk );
  smatrix[1] += ( beta*rk )/dt;
  smatrix[2] += (gamma * 2)/(dt*dt) * rk;
}
