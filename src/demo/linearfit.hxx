#ifndef _AURA_LINEAR_FIT_FILTER_HXX
#define _AURA_LINEAR_FIT_FILTER_HXX

// a class to implement a simple linear fit using low pass filtered
// internal terms so it should track changes over time and slowly
// forget history

class LinearFitFilter {
    
private:
    
    double _time_factor;
    int n;
    
    double sum_x;
    double sum_y;
    double sum_x2;
    double sum_y2;
    double sum_xy;

    // the current fit estimate
    double a0;
    double a1;

public:
    
    LinearFitFilter( double time_factor );
    ~LinearFitFilter();
    void init( double value );
    void update( double x, double y, double dt );
    void reset();
    inline double get_value( double x ) {
	return a1*x + a0;
    }
    inline double get_a0() { return a0; }
    inline double get_a1() { return a1; }
};


#endif // _AURA_LINEAR_FIT_FILTER_HXX
