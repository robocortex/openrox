//==============================================================================
//
//    OPENROX   : File sl3from4points.h
//
//    Contents  : API of sl3from4points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SL3_FROM_4_POINTS__
#define __OPENROX_SL3_FROM_4_POINTS__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/maths/linalg/matsl3.h>

//! \ingroup MatSL3
//! \addtogroup sl3from4points
//! @{

//! Generate a homography given 4 points in 2 views
//! \param  [out]  homography     Computed homography (dest = destHsource * source)
//! \param  [in ]  source         The 4 points in the source view
//! \param  [in ]  dest           The 4 points in the destination view
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matsl3_from_4_points_double (
   Rox_MatSL3 homography, 
   Rox_Point2D_Double source, 
   Rox_Point2D_Double dest
);

//! Generate a homography given 4 points in 2 views
//! \param  [out]  homography     computed homography (destHsource)
//! \param  [in ]  source         the 4 points in the source view
//! \param  [in ]  dest           the 4 points in the destination view
//! \return An error code
ROX_API Rox_ErrorCode rox_matsl3_from_4_points_float (
   Rox_MatSL3 homography, 
   Rox_Point2D_Float source, 
   Rox_Point2D_Float dest
);

//! @}

#endif
