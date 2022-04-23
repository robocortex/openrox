//==============================================================================
//
//    OPENROX   : File camera_struct.h
//
//    Contents  : API of camera module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CAMERA_STRUCT__
#define __OPENROX_CAMERA_STRUCT__

#include <generated/array2d_double.h>

#include <baseproc/image/image.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

//! \addtogroup Camera
//! @{

//! The Rox_Camera_Struct object
struct Rox_Camera_Struct
{
   //! The image
   Rox_Image  image;

   //! Camera intrinsic parameters
   Rox_MatUT3 calib_camera;

   //! Extrinsic parameters w.r.t. an arbitrary coordinate origin
   Rox_MatSE3 pose;

   //! The Undistort LUT
   Rox_MeshGrid2D_Float grid_undistort;
};

//! @}

#endif // __OPENROX_CAMERA_STRUCT__
