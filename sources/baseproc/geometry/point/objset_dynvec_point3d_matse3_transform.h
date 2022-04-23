//==============================================================================
//
//    OPENROX   : File objset_dynvec_point3d_matse3_transform.h
//
//    Contents  : API of dynvec point3d matse3 transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_OBJSET_DYNVEC_POINT3D_MATSE3_TRANSFORM__
#define __OPENROX_OBJSET_DYNVEC_POINT3D_MATSE3_TRANSFORM__

#include <generated/objset_dynvec_point3d_double.h>
#include <generated/objset_dynvec_point3d_float.h>

#include <generated/objset_matse3.h>

//! \addtogroup Point3D
//! @{

//! Given a list of points, transform them as viewed for a given pose
//! \param  [out]  dynvec_point3d_out     Result pixel coordinates
//! \param  [in ]  pose                   Camera pose matrix
//! \param  [in ]  dynvec_point3d_inp     A points list to transform
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_dynvec_point3d_double_transform ( 
   Rox_ObjSet_DynVec_Point3D_Double mo, 
   Rox_ObjSet_MatSE3                oTb, 
   Rox_ObjSet_DynVec_Point3D_Double mb
);

//! @}

#endif
