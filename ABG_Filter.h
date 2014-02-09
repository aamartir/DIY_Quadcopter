#ifndef ABG_FILTER_H
#define ABG_FILTER_H

class ABG_Filter
{
  private:
    float alfa;
    float beta;
    float gamma;

  public:
	    
    /* Variables */
		
    /* State matrix 
     * smatrix[0] = position
     * smatrix[1] = velocity
     * smatrix[2] = acceleration
     */
    float smatrix[3]; 
		
    /* Functions */
    ABG_Filter(); /* Constructor */
    void setFilterCoefficients( float new_a, float new_b, float new_g );
    float * getFilterCoefficients( void );
    void filter( float new_val, float dt ); 		/* Filter */
};

#endif

