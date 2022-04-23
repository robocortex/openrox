//==============================================================================
//
//    OPENROX   : File optimalcalib.h
//
//    Contents  : API of optimalcalib module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_OPTIMALCALIB__
#define __OPENROX_OPTIMALCALIB__

#include <generated/array2d_double.h>

//! \ingroup Camera_Calibration
//! \addtogroup OptimalCalib
//! @{

//! Compute the camera calibration of the optimal undistorted image 
//! \param [out] newcalib         The estimated camera calibration matrix
//! \param [in ] calib            A classic 3*3 camera calibration matrix with focal, skew and central point.
//! \param [in ] distortion       A 5x1 array with 3 radial distortion parameters and 2 tangential distortion parameters [k1, k2, k3, k4, k5] of the Brown's model as in Bouguet's matlab calibration toolbox.
//! \param [in ] image_width      The image width in pixels
//! \param [in ] image_height     The image height in pixels
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_calibration_optimalview (
   Rox_Array2D_Double newcalib, Rox_Array2D_Double calib, Rox_Array2D_Double distortion, Rox_Uint image_width, Rox_Uint image_height);

//! @}

#endif // __OPENROX_OPTIMALCALIB__

