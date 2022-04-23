//==============================================================================
//
//    OPENROX   : File orimoments.h
//
//    Contents  : API of orimoments module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ORIMOMENTS__
#define __OPENROX_ORIMOMENTS__

#include <generated/dynvec_segment_point.h>
#include <baseproc/image/image.h>

//! \addtogroup Detectors
//! @{

//! Compute principal orientation of patches using moments
//! \param  [out]	 points 	       A list of points (the orientation is stored inside)
//! \param  [in ]	 source 	       The image to extract patches' luminosity from
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_segment_points_compute_orientation_moments(Rox_DynVec_Segment_Point points, Rox_Image source);

//! Compute principal orientation of patches using moments (See orb paper)
//! \param  [out]  theta 	       Computed theta angle
//! \param  [in ]  source	       The image luminosity (must be square)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_compute_orientation_moments(Rox_Double * theta, Rox_Image source);

//! Compute principal orientation of patches using second order moments
//! \param  [out]  theta		    Computed theta angle
//! \param  [in ]	 source	       The image luminosity (must be square)
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_array2d_uchar_compute_orientation_secondmoments(Rox_Double * theta, Rox_Image source);

//! @} 

#endif
