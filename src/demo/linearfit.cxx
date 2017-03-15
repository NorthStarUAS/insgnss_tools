#include <math.h>
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
    a1(1.0)
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

    // printf("%d %.0f %.0f %.0f %.0f %.0f\n",
    //        n, sum_x, sum_y, sum_x2, sum_y2, sum_xy);
    
    double denom = n*sum_x2 - sum_x*sum_x;
    if ( n > 1 and fabs(denom) > 0.00001 ) {
	a1 = (n*sum_xy - sum_x*sum_y) / denom;
	a0 = (sum_y - a1*sum_x) / n;
    } else {
	a1 = 0.0;
	a0 = y;
    }

    // the following demonstrates we can multiply all the internal
    // terms by a common factor and produce the exact same answer as
    // above.  This shows we can "scale" the whole problem by (n-1)/n
    // every iteration to constrain size of the internal terms.  This
    // also has the effect of tracking changes in the relationship
    // over time (if we are modeling a physical relationshpi and that
    // relationship has variation over time.)
    // double factor = 0.75;
    // double new_n = n * factor;
    // double new_sum_x = sum_x * factor;
    // double new_sum_y = sum_y * factor;
    // double new_sum_x2 = sum_x2 * factor;
    // double new_sum_y2 = sum_y2 * factor;
    // double new_sum_xy = sum_xy * factor;
    // double new_denom = new_n*new_sum_x2 - new_sum_x*new_sum_x;
    // double new_a0, new_a1;
    // if ( n > 1 and fabs(new_denom) > 0.00001 ) {
    //     new_a1 = (new_n*new_sum_xy - new_sum_x*new_sum_y) / new_denom;
    //     new_a0 = (new_sum_y - new_a1*new_sum_x) / new_n;
    // } else {
    //     new_a1 = 0.0;
    //     new_a0 = y;
    // }
    // printf("fit: %.4f %.4f\n", a1, a0);
    // printf("new: %.4f %.4f\n", new_a1, new_a0);
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
