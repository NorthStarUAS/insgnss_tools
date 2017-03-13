#include <stdio.h>

#include "linearfit.hxx"

LinearFitFilter::LinearFitFilter( double time_factor ):
    n(0),
    sum_x(0.0),
    sum_y(0.0),
    sum_x2(0.0),
    sum_y2(0.0),
    sum_xy(0.0),
    a0(0.0),
    a1(0.0)
{
    _time_factor = time_factor;
}

LinearFitFilter::~LinearFitFilter() {};

void LinearFitFilter::update( double x, double y, double dt ) {
    int max_n = _time_factor / dt;
    n++;
    double wf = 1.0;
    if ( n > max_n ) {
	n = max_n;
	wf = (n-1.0)/n;
    }
    
    sum_x = sum_x * wf + x;
    sum_y = sum_y * wf + y;
    sum_x2 = sum_x2 * wf + x*x;
    sum_y2 = sum_y2 * wf + y*y;
    sum_xy = sum_xy * wf + x*y;

    if ( n > 1 ) {
	a1 = (n*sum_xy - sum_x*sum_y) / (n*sum_x2 - sum_x*sum_x);
	a0 = (sum_y - a1*sum_x) / n;
    } else {
	a1 = 0.0;
	a0 = y;
    }
    //printf("fit: %.4f %.4f\n", a1, a0);
}

void LinearFitFilter::reset() {
    n = 0;
    sum_x = 0.0;
    sum_y = 0.0;
    sum_x2 = 0.0;
    sum_y2 = 0.0;
    sum_xy = 0.0;
    a0 = 0.0;
    a1 = 0.0;
}
