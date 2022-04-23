//==============================================================================
//
//    OPENROX   : File fastst_score.h
//
//    Contents  : API of fastst_score module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_FASTST_SCORE__
#define __OPENROX_FASTST_SCORE__

#include <generated/dynvec_segment_point.h>
#include <baseproc/image/image.h>


//! \addtogroup Fastst
//! @{


//! Give a score to a list of detected points using fast corner score
//! \param  []  points            The output/input list of scored points
//! \param  []  source            The input image to extract points from
//! \param  []  barrier           The intensity minimal difference for segment
//! \return an error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_fastst_detector_score(Rox_DynVec_Segment_Point points, Rox_Image source, Rox_Sint barrier);


//! Keep corners which are extremas locally
//! !!! WARNING !!!
//! Has some expectation about the ordering of input points.
//  This expectation is not checked and may cause a SEGFAULT if violated.
//! It seems to be that points shall be ordered by increasing i coordinate.
//!
//! \param  []  ptr               The output list of maxima points
//! \param  []  corners           The input list of points
//!
//! \return an error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_fastst_nonmax_suppression(Rox_DynVec_Segment_Point ptr, Rox_DynVec_Segment_Point corners);


//! Sort corners by decreasing score
//! \param  []  ptr               The input/output list of sorted points
//! \return an error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_fastst_detector_sort(Rox_DynVec_Segment_Point ptr);


//! @}


#endif
