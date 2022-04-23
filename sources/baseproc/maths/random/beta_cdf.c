//==============================================================================
//
//    OPENROX   : File beta_cdf.c
//
//    Summary   : Implementation of beta_cdf module 
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <math.h>                    

#include "beta_cdf.h" 

#define THRESHOLD 1.0e-8
#define EPSILON 1.0e-30
#define MAX_ITERS 200

Rox_Double rox_beta_cdf ( Rox_Double a, Rox_Double b, Rox_Double x ) 
{
    if (x < 0.0 || x > 1.0) 
    { goto function_terminate; }

    /// The continued fraction converges nicely for x < (a+1)/(a+b+2)
    if (x > (a+1.0)/(a+b+2.0)) 
    {
        return ( 1.0 - rox_beta_cdf ( b, a, 1.0-x ) ); // Use the fact that beta is symmetrical.
    }

    // Find the first part before the continued fraction
    const Rox_Double lbeta_ab = lgamma(a)+lgamma(b)-lgamma(a+b);
    const Rox_Double front = exp(log(x)*a+log(1.0-x)*b-lbeta_ab) / a;

    // Use Lentz's algorithm to evaluate the continued fraction
    Rox_Double f = 1.0, c = 1.0, d = 0.0;

    for (Rox_Sint i = 0; i <= MAX_ITERS; ++i) 
    {
        Rox_Sint m = i/2;

        Rox_Double numerator;
        if (i == 0) 
        {
            numerator = 1.0; // First numerator is 1.0
        } 
        else if (i % 2 == 0) 
        {
            numerator = (m*(b-m)*x)/((a+2.0*m-1.0)*(a+2.0*m)); // Even term
        } 
        else 
        {
            numerator = -((a+m)*(a+b+m)*x)/((a+2.0*m)*(a+2.0*m+1)); // Odd term
        }

        // Do an iteration of Lentz's algorithm
        d = 1.0 + numerator * d;
        if (fabs(d) < EPSILON) d = EPSILON;
        d = 1.0 / d;

        c = 1.0 + numerator / c;
        if (fabs(c) < EPSILON) c = EPSILON;

        const Rox_Double cd = c*d;
        f *= cd;

        // Check for stop
        if (fabs(1.0-cd) < THRESHOLD) {
            return front * (f-1.0);
        }
    }

function_terminate:

    return DBL_MAX; // Needed more loops, did not converge
}
