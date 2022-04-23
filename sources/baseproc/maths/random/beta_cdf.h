//==============================================================================
//
//    OPENROX   : File beta_cdf.h
//
//    Contents  : API of beta_cdf module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BETA_CDF__
#define __OPENROX_BETA_CDF__

#include <baseproc/maths/maths_macros.h>
#include <system/errors/errors.h>

//! The Beta Cumulative Distribution Function
//! For more details on the algorithm see https://codeplea.com/incomplete-beta-function-c
//! \param  [in ]  a       The a parameter of the Beta distribution
//! \param  [in ]  b       The b parameter of the Beta distribution
//! \param  [in ]  x       The value x must lie on the interval [0,1]
ROX_API Rox_Double rox_beta_cdf ( Rox_Double a, Rox_Double b, Rox_Double x );

#endif // __OPENROX_BETA_CDF__
