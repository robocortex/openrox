//==============================================================================
//
//    OPENROX   : File dynvec_point3d_save.h
//
//    Contents  : API of dynvec point3d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNAMIC_POINT3D_SAVE__
#define __OPENROX_DYNAMIC_POINT3D_SAVE__

#include <stdio.h>

#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_point3d_double.h>

//! \addtogroup Point2D
//! @{

//! Ascii serialization of a dynvec of point3d of double.
//! \param [in]     filename             
//! \param [in]      dynvec_point3d       The dynvec to save
//! \return An error code.

ROX_API Rox_ErrorCode rox_dynvec_point3d_double_save(const Rox_Char * filename, const Rox_DynVec_Point3D_Double dynvec_point3d);
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_save_stream(FILE * output, Rox_DynVec_Point3D_Double input);

ROX_API Rox_ErrorCode rox_dynvec_point3d_double_save_binary(const Rox_Char * filename, Rox_DynVec_Point3D_Double source);
ROX_API Rox_ErrorCode rox_dynvec_point3d_double_load_binary(Rox_DynVec_Point3D_Double source, Rox_Char *filename);

//! @} 

#endif
