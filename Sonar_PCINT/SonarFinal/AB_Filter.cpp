#include "AB_Filter.h"

/* Constructor */
AB_Filter::AB_Filter()
{
  alfa = 0;
  beta = 0;
}

void AB_Filter::setFilterCoefficients( float new_a, float new_b )
{
  alfa = new_a;
  beta = new_b;
}

float * AB_Filter::getFilterCoefficients()
{
  float * coef = new float[2];
  
  coef[0] = alfa;
  coef[1] = beta;
  
  return coef;
}

void AB_Filter::filter( float xm, float dt )
{
  float rk;
  
  /* Model equation */
  smatrix[0] += (smatrix[1] * dt);		 	/* Position */
  smatrix[1] = smatrix[1];		                /* Velocity */
  
  /* Error between measurement and model */
  rk = (xm - smatrix[0]);
  
  /* Update model with alfa, beta parameters 
   * in the direction of the gradient. */
  smatrix[0] += ( alfa*rk );
  smatrix[1] += ( beta*rk )/dt;
}
