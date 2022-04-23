//==============================================================================
//
//    OPENROX   : File camera_calibration.h
//
//    Contents  : API of camera_calibration module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CAMERA_CALIBRATION__
#define __OPENROX_CAMERA_CALIBRATION__

#include <baseproc/maths/linalg/matut3.h>

#include <core/model/model_single_plane.h>
#include <core/calibration/mono/calibration_perspective.h>
#include <core/calibration/mono/calibration_perspective_struct.h>

#include <user/identification/texture/ident_texture_sl3.h>
#include <user/tracking/tracking.h>

//! \addtogroup Camera_Calibration
//! @}

//! \brief The Rox_Camera_Calibration_Struct object
struct Rox_Camera_Calibration_Struct
{
   //! The dense calibration object 
   Rox_Calibration_Mono_Perspective calib;

   //! The tracking object to refine the homographies 
   Rox_Tracking tracker;

   //! The identification object 
   Rox_Ident_Texture_SL3 identifier;

   //! The 3D-2D model calibration 
   Rox_MatUT3 template_calibration;

   //! The estimated intrinsic parameters 
   Rox_MatUT3 intrinsics;

   //! The tracking homography 
   Rox_MatSL3 G;

   //! The reference points 
   Rox_DynVec_Point2D_Double ref_pts;

   //! The image width 
   Rox_Sint width;

   //! The image height 
   Rox_Sint height;
};

//! Define the pointer of the rox_camera_calibration_Struct 
typedef struct Rox_Camera_Calibration_Struct * Rox_Camera_Calibration;

//! \brief Create a new calibration planar model of known size
//! \param  [out]  calibration    calibration object
//! \param  [in ]  model          Model of 2D target
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_new ( 
   Rox_Camera_Calibration * calibration, 
   const Rox_Model_Single_Plane model 
);

//! Delete model for camera calibration
//! \param  [out]  calibration calibration object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_del ( 
   Rox_Camera_Calibration * calibration
);

//! Add an image to the calibration set
//! \param  [out]  calibration    Calibration object
//! \param  [in ]  image          Calibration image with a visible image model
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_add_image ( 
   Rox_Camera_Calibration calibration, 
   const Rox_Image image
);

//! Add externally estimated homography to the calibration set
//! \param  [out]  calibration    Calibration object
//! \param  [in ]  homography     externally estimated homography
//! \param  [in ]  image_width    width of the camera image
//! \param  [in ]  image_height   height of the camera image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_calibration_add_homography ( 
   Rox_Camera_Calibration calibration, 
   const Rox_MatSL3 homography, 
   const Rox_Sint image_width, 
   const Rox_Sint image_height
);

//! Perform camera calibration of desired parameter
//! \param  [out]  calibration    calibration object
//! \param  [in ]  method         number of parameters to be calibrated
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_make ( 
   Rox_Camera_Calibration calibration, 
   const Rox_Sint method 
);

//! Get a copy of the estimated intrinsic parameters
//! \param  [out]  intrinsics     Copy of the estimated parameters
//! \param  [in ]  calibration    calibration object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_calibration_get_intrinsics ( 
   Rox_MatUT3 intrinsics, 
   const Rox_Camera_Calibration calibration
);

//! Get the mean reprojection error
//! \param  [out] mean              the mean reprojection error
//! \param  [in]  calibration       calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_camera_calibration_get_mean_error (
   Rox_Double * mean, 
   const Rox_Camera_Calibration calibration
);

//! @} 

#endif // __OPENROX_CAMERA_CALIBRATION__
