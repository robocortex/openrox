//==============================================================================
//
//    OPENROX   : File dynvec_point2d_save.h
//
//    Contents  : API of dynvec point2d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNAMIC_POINT2D_SAVE__
#define __OPENROX_DYNAMIC_POINT2D_SAVE__

#include <stdio.h>

#include <generated/dynvec_point2d_float.h>
#include <generated/dynvec_point2d_double.h>

//! \addtogroup Point2D
//! @{

//! Ascii serialization of a dynvec of point2d of float.
//! The input is NOT portable at ALL. Meant to load buffers.
//! \param [out]     filename   
//! \param [in]      input      The dynvec to save
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_point2d_float_save(const Rox_Char *filename, Rox_DynVec_Point2D_Float input);
ROX_API Rox_ErrorCode rox_dynvec_point2d_float_save_stream(FILE * output, Rox_DynVec_Point2D_Float input);

//! Ascii serialization of a dynvec of point2d of double.
//! \param [in]     filename   
//! \param [in]      input      The dynvec to save
//! \return An error code.
ROX_API Rox_ErrorCode rox_dynvec_point2d_double_save(const Rox_Char * filename, const Rox_DynVec_Point2D_Double dynvec_point2d);
ROX_API Rox_ErrorCode rox_dynvec_point2d_double_save_stream(FILE * output, Rox_DynVec_Point2D_Double input);

//! @} 

#endif
