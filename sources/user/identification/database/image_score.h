//==============================================================================
//
//    OPENROX   : File image_score.h
//
//    Contents  : API of image_score module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_SCORE__
#define __OPENROX_IMAGE_SCORE__

#include <baseproc/image/image.h>

//! \addtogroup Image
//!	@{

//! \brief Test the image quality using a fast method
//! \param [out] score The quality score
//! \param [in] image The image to test
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_image_get_quality_score(Rox_Double *score, Rox_Image image);

//! \brief Test the image quality using a slow method
//! \param [out] score The quality score
//! \param [in] image The image to test
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_image_get_quality_score_precise(Rox_Double *score, Rox_Image image);

//! @}

#endif // __OPENROX_IMAGE_SCORE__
