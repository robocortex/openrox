//==============================================================================
//
//    OPENROX   : File image_warp_matsl3.h
//
//    Contents  : API of image warping module with matrix in SL3
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMAGE_WARP_MATSL3__
#define __OPENROX_IMAGE_WARP_MATSL3__

#include <generated/array2d_uchar.h>
#include <generated/array2d_float.h>
#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>
#include <baseproc/maths/linalg/matsl3.h>

//! Set the image from an external pixel buffer
//! \param [in]   image_warped      The warped image
//! \param [in]   image_source      The source image
//! \param [in]   homography		   The homography matrix in SL3
//! \todo To be tested
//! \return An error code
ROX_API Rox_ErrorCode rox_image_warp_matsl3(Rox_Image image_warped, Rox_Image image_source, Rox_MatSL3 homography);

//! Set the image from an external pixel buffer
//! \param [in]   image_warped      The warped image
//! \param [in]   imask_warped      The imask of the warped image
//! \param [in]   image_source      The source image
//! \param [in]   homography        The homography matrix in SL3
//! \todo To be tested
//! \return An error code
ROX_API Rox_ErrorCode rox_image_imask_warp_matsl3(Rox_Image image_warped, Rox_Imask imask_warped, Rox_Image image_source, Rox_MatSL3 homography);

//! Set the image from an external pixel buffer
//! \param [in]   image_warped      The warped image float
//! \param [in]   image_source      The source image float
//! \param [in]   homography        The homography matrix in SL3
//! \todo To be tested
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_warp_matsl3(Rox_Array2D_Float image_warped, Rox_Array2D_Float image_source, Rox_MatSL3 homography);
//! Set the image from an external pixel buffer
//! \param [in]   image_warped      The warped image float
//! \param [in]   imask_warped      The imask of the warped image
//! \param [in]   image_source      The source image float
//! \param [in]   homography        The homography matrix in SL3
//! \todo To be tested
//! \return An error code
ROX_API Rox_ErrorCode rox_array2d_float_imask_warp_matsl3(Rox_Array2D_Float image_warped, Rox_Imask imask_warped, Rox_Array2D_Float image_source, Rox_MatSL3 homography);

#endif // __OPENROX_IMAGE_WARP_MATSL3__
