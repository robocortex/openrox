//==============================================================================
//
//    OPENROX   : File dynvec_point3d_print.h
//
//    Contents  : API of dynvec point3d display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNVEC_POINT3D_PRINT__
#define __OPENROX_DYNVEC_POINT3D_PRINT__

#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_uint.h>
#include <baseproc/geometry/point/point3d_struct.h>

//! \addtogroup Point3D
//! @{

//! Display a 3D points dynamic vector of float on stdout
//! \param  [in]  points3D          The dynamic vector of 3D points to print
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_float_print ( const Rox_DynVec_Point3D_Float dynvec_points3d );

//! Display a 3D points dynamic vector of double on stdout
//! \param  [in]  points3D          The dynamic vector of 3D points to print
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_print ( const Rox_DynVec_Point3D_Double dynvec_points3d );

//! @} 

#endif
