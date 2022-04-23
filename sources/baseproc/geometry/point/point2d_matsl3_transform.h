//==============================================================================
//
//    OPENROX   : File point2d_matsl3_transform.h
//
//    Contents  : API of point2D transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINT2D_MATSL3_TRANSFORM__
#define __OPENROX_POINT2D_MATSL3_TRANSFORM__

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point2d.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>

//! \addtogroup Point2D
//! @{

//! Given a point, compute the transformation of the points for a given homography
//! \param  [out]  output         Result pixels coordinates
//! \param  [in ]  input          A point list to transform
//! \param  [in ]  homography     Pixel to pixel homography matrix
//! \param  [in ]  count          The point list size
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_double_homography (
   Rox_Point2D_Double output, 
   Rox_Point2D_Double input, 
   Rox_MatSL3 homography, 
   Rox_Sint count
);

//! Given a point, compute the transformation of the points for a given homography
//! \param  [out]  output         Result pixels coordinates
//! \param  [in ]  input          A point list to transform
//! \param  [in ]  homography     Pixel to pixel homography matrix
//! \param  [in ]  count          The point list size
//! \return An error code
ROX_API Rox_ErrorCode rox_point2d_float_homography (
   Rox_Point2D_Float output, 
   Rox_Point2D_Float input, 
   Rox_MatSL3 homography, 
   Rox_Sint count
);

//! @}

#endif
