class CompFilter 
{
private:
  float previousAngle;
  float newAngle;
  float newRate;
  float filterTerm0;
  float filterTerm1;
  float filterTerm2;
  float bw; //bw = 1.1 seems to work

public:
  CompFilter() 
  {
    filterTerm0 = 0;
    filterTerm1 = 0;
  }
  
  void initialize(float acc_angle, float gyro_rate, float BW) 
  {
    previousAngle = acc_angle;
    filterTerm2 = gyro_rate;
    bw = BW; // Filter bandwidth
  }
  
  float calculate(float newAngle, float newRate) 
  {
    // Written by RoyLB at http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286    
    filterTerm0 = (newAngle - previousAngle) * bw *  bw;
    filterTerm2 += filterTerm0 * G_dt;
    filterTerm1 = filterTerm2 + (newAngle - previousAngle) * 2 *  bw + newRate;
    previousAngle = (filterTerm1 * G_dt) + previousAngle;
    
    return previousAngle; // This is actually the current angle, but is stored for the next iteration
  }
};
