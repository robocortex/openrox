//==============================================================================
//
//    OPENROX   : File gftt.h
//
//    Contents  : API of gftt module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_GFTT__
#define __OPENROX_GFTT__

#include <generated/array2d_float.h>
#include <generated/dynvec_segment_point.h>

//! \ingroup Detectors
//! \addtogroup GFTT
//! @{

//! Good Features To Track. Given a response image (computed using harris for example), extract corners
//! \param[out] corners the list of corners
//! \param[out] response response image
//! \param[in] radius the radius for non maximal suppression
//! \param[in] threshold the minimal value for corners
//! \return an error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_gftt_detector(Rox_DynVec_Segment_Point corners, Rox_Array2D_Float response, Rox_Uint radius, Rox_Float threshold);

//! @}

#endif
