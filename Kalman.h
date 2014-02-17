#define X_axis 0
#define Y_axis 1
#define Z_axis 2

class Kalman 
{
private:
    double x_angle[3]; /* X, Y, Z */
    double x_bias[3];
    
    double P_00[3];
    double P_01[3];
    double P_10[3];
    double P_11[3];
    	
    double Q_angle, Q_gyro;
    double R_angle;
    
    double y, S;
    double K_0, K_1;

public:
  Kalman() 
  {
      for( int axis = 0; axis <= 2; axis++ )
      {
          x_angle[axis] = 0;
          x_bias[axis] = 0;
          
          P_00[axis] = 0;
          P_01[axis] = 0;
          P_10[axis] = 0;
          P_11[axis] = 0;
      }
  }
  
  void initialize()
  {
     /*
      * Bigger Q_angle: trust accelerometer more
      * Bigger Q_gyro: trust Gyro more
      */
      
      Q_angle = 0.0001; //0.001 ----- 0.0001
      Q_gyro = 0.0003; //0.003 ---- 0.0003
      R_angle = 20; //1.719; // 0.3 = 30
  }
  
  double getGyroBias(byte axis) 
  {
      return x_bias[axis];
  }
  
  double getAngle(byte axis)
  {
      return x_angle[axis];
  }
  
  void calculate(byte axis, double newAngle, double newRate, double G_dt) 
  {
      /* Linear Stochastic Model equation */
      x_angle[axis] += G_dt * (newRate - x_bias[axis]);
      
      /* Calculate Covariance Matrix */
      P_00[axis]    += G_dt * (P_11[axis]*G_dt - P_10[axis] - P_01[axis] + Q_angle);
      P_01[axis]    -= G_dt * P_11[axis];
      P_10[axis]    -= G_dt * P_11[axis];
      P_11[axis]    += Q_gyro * G_dt;
      
      /* Measurement */
      y = newAngle - x_angle[axis];
      S = P_00[axis] + R_angle;
      
      /*  Calculate Kalman gains */
      K_0 = P_00[axis] / S; 
      K_1 = P_10[axis] / S;
      
      /* Correction */
      x_angle[axis] +=  K_0 * y;
      x_bias[axis]  +=  K_1 * y;
      
      P_00[axis] -= K_0 * P_00[axis];
      P_01[axis] -= K_0 * P_01[axis];
      P_10[axis] -= K_1 * P_00[axis];
      P_11[axis] -= K_1 * P_01[axis];
      
      //return x_angle[axis];
  }
  
  int Xaxis()
  {
    return X_axis;
  }
  
  int Yaxis()
  {
    return Y_axis;
  }
  
  int Zaxis()
  {
    return Z_axis;
  }
};

