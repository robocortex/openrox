//==============================================================================
//
//    OPENROX   : File objset_dynvec_point3d_print.h
//
//    Contents  : API of objset dynvec point3d display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_OBJSET_dynvec_point3d_print__
#define __OPENROX_OBJSET_dynvec_point3d_print__

#include <generated/objset_dynvec_point3d_float.h>
#include <generated/objset_dynvec_point3d_double.h>

//! \addtogroup Point3D
//! @{

//! Display a 3D points objset dynamic vector of float on stdout
//! \param  [in]  points3D          The dynamic vector of 3D points to print
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_dynvec_point3d_float_print ( const Rox_ObjSet_DynVec_Point3D_Float objset_dynvec_points3d );

//! Display a 3D points objset dynamic vector of double on stdout
//! \param  [in]  points3D          The dynamic vector of 3D points to print
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_dynvec_point3d_double_print ( const Rox_ObjSet_DynVec_Point3D_Double objset_dynvec_points3d );

//! @} 

#endif
