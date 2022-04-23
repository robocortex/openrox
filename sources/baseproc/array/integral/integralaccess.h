//==============================================================================
//
//    OPENROX   : File integralaccess.h
//
//    Contents  : API of integralaccess module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INTEGRALACCESS__
#define __OPENROX_INTEGRALACCESS__

#include <system/memory/array2d.h>

//! \ingroup Image
//! \addtogroup integral
//! @{

//! Using an integral array, compute a value for a given region of interest.
//! This value is dependent with the integral meaning
//! \param  [out]  integral the integral image row pointers
//! \param  [in ]  x        the ROI left
//! \param  [in ]  y        the ROI top
//! \param  [in ]  w        the ROI width
//! \param  [in ]  h        the ROI right
//! \return The computed value
Rox_Slint rox_array2d_slint_integralaccess ( Rox_Slint ** integral, const int x, const int y, const int w, const int h);

//! @} 

#endif
