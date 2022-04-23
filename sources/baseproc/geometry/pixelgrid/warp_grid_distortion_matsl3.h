//==============================================================================
//
//    OPENROX   : File warp_grid_distortion_matsl3.h
//
//    Contents  : API of warp_grid_distortion_matsl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_WARP_GRID_DISTORTION_SL3__
#define __OPENROX_WARP_GRID_DISTORTION_SL3__

#include <generated/array2d_double.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

//! \ingroup Geometry
//! \addtogroup WarpGridDistortionSl3
//! @{

//! Compute a map of pixel coordinates after a parametric distortion plus a warp on sl3. Used to undistort and rectify image.
//! \param  [out]  grid           The result map (map(u,v) = u',v' -> distort(u,v))
//! \param  [in ]  H              A classic 3*3 homography
//! \param  [in ]  calib_in       A classic 3*3 camera calibration matrix with focal, skew and central point (For the undistorted frame)
//! \param  [in ]  calib_out      A classic 3*3 camera calibration matrix with focal, skew and central point (For the distorted frame)
//! \param  [in ]  distortion     A 5*1 array with 3 radial distortion parameters and 2 tangential distortion parameters [k1, k2, k3, k4, k5] as in bouguet's matlab calibration toolbox.
//! \return An error code
//! \see http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
//! \todo To be tested
ROX_API Rox_ErrorCode rox_warp_grid_distortion_sl3_float (
   Rox_MeshGrid2D_Float grid, 
   const Rox_Array2D_Double H, 
   const Rox_Array2D_Double calib_in, 
   const Rox_Array2D_Double calib_out, 
   const Rox_Array2D_Double distortion
);

//! @}

#endif
