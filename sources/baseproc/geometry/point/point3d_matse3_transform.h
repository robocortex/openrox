//==============================================================================
//
//    OPENROX   : File point3d_matse3_transform.h
//
//    Contents  : API of point3d matse3 transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINT3D_MATSE3_TRANSFORM__
#define __OPENROX_POINT3D_MATSE3_TRANSFORM__

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/maths/linalg/matse3.h>

//! \addtogroup Point3D
//! @{

//! Given a list of points, transform them as viewed for a given pose
//! \param  [out]  res            Result pixel coordinates
//! \param  [in ]  pose           Camera pose matrix
//! \param  [in ]  input          A points list to transform
//! \param  [in ]  count          The list size
//! \return An error code
ROX_API Rox_ErrorCode rox_point3d_double_transform (
   Rox_Point3D_Double   point3d_out, 
   Rox_MatSE3           pose, 
   Rox_Point3D_Double   point3d_inp, 
   Rox_Uint             count
);

//! Given a list of points, transform them as viewed for a given pose
//! \param  [out]  res            Result pixel coordinates
//! \param  [in ]  pose           Camera pose matrix
//! \param  [in ]  input          A points list to transform
//! \param  [in ]  count          The list size
//! \return An error code
ROX_API Rox_ErrorCode rox_point3d_float_transform ( 
   Rox_Point3D_Float    point3d_out, 
   Rox_MatSE3           pose, 
   Rox_Point3D_Float    point3d_inp, 
   Rox_Uint             count
);

//! @}

#endif
