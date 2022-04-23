//==============================================================================
//
//    OPENROX   : File dynvec_print.h
//
//    Contents  : API of sdynvec display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_DYNAMIC_DISPLAY__
#define __OPENROX_DYNAMIC_DISPLAY__

#include <generated/dynvec_float.h>
#include <generated/dynvec_double.h>
#include <generated/dynvec_uint.h>

//! \addtogroup Point2D
//! @{

//! Display dynamic vector of float on stdout
//! \param [in] dynvec 		The dynvec to print
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_float_print(Rox_DynVec_Float dynvec);

//! Display dynamic vector of double on stdout
//! \param [in] dynvec_ 		The dynvec to print
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_double_print(Rox_DynVec_Double dynvec);

//! Display a  dynamic vector of unit on stdout
//! \param [in] dynvec      The dynvec to print
//! \return An error code
ROX_API Rox_ErrorCode rox_dynvec_uint_print(Rox_DynVec_Uint dynvec);

//! @} 

#endif
