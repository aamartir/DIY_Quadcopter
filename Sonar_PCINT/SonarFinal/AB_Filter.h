#ifndef AB_FILTER_H
#define AB_FILTER_H

class AB_Filter
{
  private:
    float alfa;
    float beta;

  public:
	    
    /* Variables */
		
    /* State matrix 
     * smatrix[0] = position
     * smatrix[1] = velocity
     */
    float smatrix[2]; 
		
    /* Functions */
    AB_Filter(); /* Constructor */
    void setFilterCoefficients( float new_a, float new_b );
    float * getFilterCoefficients( void );
    void filter( float new_pos, float dt ); 		/* Filter */
};

#endif

