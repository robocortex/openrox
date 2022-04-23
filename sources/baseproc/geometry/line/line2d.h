//==============================================================================
//
//    OPENROX   : File line_2d.h
//
//    Contents  : API of line 2D module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE_2D__
#define __OPENROX_LINE_2D__

#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/geometry/point/point2d.h>

//! \addtogroup Line
//! @{

//! Pointer to the structure of a 2D line in normal form 
typedef struct Rox_Line2D_Normal_Struct * Rox_Line2D_Normal;

//! Pointer to the structure of a 2D line in homogeneous form 
typedef struct Rox_Line2D_Homogeneous_Struct * Rox_Line2D_Homogeneous;

//! Pointer to the structure of a 2D line in parametric form 
typedef struct Rox_Line2D_Parametric_Struct * Rox_Line2D_Parametric;

//! Compute the signed distance of a point a 2D line in polar cordinates
//! \param  [out]  signed_distance       The point on the segment line which is closest to the ray
//! \param  [in ]  line2d_normal         The 2D line in polar cordinates
//! \param  [in ]  point2d               The ray to intersect with the line
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_normal_signed_distance ( 
   Rox_Double * signed_distance, 
   const Rox_Line2D_Normal line2d_normal, 
   const Rox_Point2D_Double point2d
);

//! Compute the unsigned distance of a point a 2D line in polar cordinates
//! \param  [out]  unsigned_distance     The point on the segment line which is closest to the ray
//! \param  [in ]  line2d_normal         The 2D line in polar cordinates
//! \param  [in ]  point2d               The ray to intersect with the line
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_normal_unsigned_distance (
   Rox_Double * unsigned_distance, 
   const Rox_Line2D_Normal line2d_normal, 
   const Rox_Point2D_Double point2d
);

//! Tranform the line from meters to pixels coordinates
//! \param  [out]  line2d_pixels       The 2D line in pixels cordinates
//! \param  [in ]  line2d_meters       The 2D line in meters cordinates
//! \param  [in ]  pix_K_met           The meters to pixels calibration matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_line2d_transform_meters_to_pixels (   
   Rox_Line2D_Normal line2d_pixels,  
   Rox_Line2D_Normal line2d_meters,  
   const Rox_MatUT3 pix_K_met
);

//! @}

#endif
