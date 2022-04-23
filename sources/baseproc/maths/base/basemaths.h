//==============================================================================
//
//    OPENROX   : File basemaths.h
//
//    Contents  : API of basemaths module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BASEMATHS__
#define __OPENROX_BASEMATHS__

#include <baseproc/maths/maths_macros.h>
#include <system/errors/errors.h>
                                                                                                                                          
//! \defgroup Maths Maths                                                                                                                                                    
//! \brief Maths structures and methods.                                                                                                                                     

//! \ingroup Maths
//! \addtogroup Basemaths
//! @{

ROX_API Rox_ErrorCode rox_angle_isinrange ( 
   Rox_Sint * inrange, 
   const Rox_Double angle_model, 
   const Rox_Double angle_measure, 
   const Rox_Double angle_range
);

//! Compute the POPCNT value for a 32 bit value
//! \param [in] v the value to analyze
//! \return the number of set bits.
Rox_Uint _rox_popcount_slow(Rox_Uint v);

//! Factorial function
Rox_Uint rox_factorial(Rox_Uint x);

// Approximate atan2 used in quad detection : quad_gradientclusterer_funcs.c
static inline Rox_Float quad_arctan2(Rox_Float y, Rox_Float x)
{
   static const Rox_Float coeff_1 = (Rox_Float) (ROX_PI/4);
   static const Rox_Float coeff_2 = (Rox_Float) (3*ROX_PI/4);

   Rox_Float abs_y = (Rox_Float) (fabs(y)+1e-10);
   Rox_Float angle;

   if (x >= 0)
   {
      Rox_Float r = (x - abs_y) / (x + abs_y);
      angle = coeff_1 - coeff_1 * r;
   }
   else
   {
      Rox_Float r = (x + abs_y) / (abs_y - x);
      angle = coeff_2 - coeff_1 * r;
   }

   if (y < 0) return -angle;
   else return angle;
}

// Fast atan2 used in : ehid_matcher.c
static inline Rox_Float fast_atan2f2(Rox_Float y, Rox_Float x)
{
   Rox_Float angle, r ;
   Rox_Float const c3 = 0.1821f;
   Rox_Float const c1 = 0.9675f;
   Rox_Float abs_y = fabsf(y) + 1.19209290E-07F;

   if (x >= 0)
   {
      r = (x - abs_y) / (x + abs_y) ;
      angle = (Rox_Float) (ROX_PI / 4) ;
   }
   else
   {
      r = (x + abs_y) / (abs_y - x) ;
      angle = (Rox_Float) (3 * ROX_PI / 4) ;
   }
   angle += (c3*r*r - c1) * r ;
   return (y < 0) ? - angle : angle ;
}

static inline Rox_Double fast_atan2( Rox_Double y, Rox_Double x )
{
   //return atan2( y,x );
   Rox_Double angle, r ;
   Rox_Double const c3 = 0.1821;
   Rox_Double const c1 = 0.9675;
   Rox_Double abs_y = fabs( y ) + 1.19209290E-07;

   if ( x >= 0 )
   {
      r = ( x - abs_y ) / ( x + abs_y ) ;
      angle = (Rox_Double)( ROX_PI / 4 ) ;
   }
   else
   {
      r = ( x + abs_y ) / ( abs_y - x ) ;
      angle = (Rox_Double)( 3 * ROX_PI / 4 ) ;
   }
   angle += ( c3 * r * r - c1 ) * r ;
   return ( y < 0 ) ? - angle : angle ;
}

static ROX_INLINE Rox_Float fast_expfneg( const Rox_Float val )
{
   return expf( val );

   //static const float f1 = 1.0f/720.0f;
   //static const float f2 = 1.0f/120.0f;
   //static const float f3 = 1.0f/240.0f;
   //static const float f4 = 1.0f/6.0f;

   //float x = val;
   //float x2 = x * x;
   //float x3 = x2 * x;
   //float x4 = x3 * x;
   //float x5 = x4 * x;
   //float x6 = x5 * x;

   //return f1*x6+f2*x5+f3*x4+f4*x3+x2*0.5f+x+1.0f;
}

#endif
