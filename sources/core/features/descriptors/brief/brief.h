//==============================================================================
//
//    OPENROX   : File brief.h
//
//    Contents  : API of brief module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BRIEF__
#define __OPENROX_BRIEF__

#include <generated/dynvec_brief_point.h>
#include <generated/dynvec_segment_point.h>

#include <baseproc/image/image.h>

//! \ingroup Descriptors
//! \defgroup BRIEF BRIEF
//! \brief BRIEF keypoints descriptor.

//! \addtogroup BRIEF
//! @{

//! Compute the descriptor for a list of feature points
//! \param  [out]  obj            The list of features descriptions
//! \param  [in ]  source         The image associated with the points to extract description from
//! \param  [in ]  points         The list of features with 2D coordinates of the points set extracted with a FAST like method
//! \return An error code
ROX_API Rox_ErrorCode rox_brief_points_compute (
   Rox_DynVec_Brief_Point obj, 
   const Rox_Image source, 
   Rox_DynVec_Segment_Point points
);

//! @} 

#endif
