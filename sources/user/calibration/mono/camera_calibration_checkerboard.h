//==============================================================================
//
//    OPENROX   : File camera_calibration_checkerboard.h
//
//    Contents  : API of camera_calibration_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CAMERA_CALIBRATION_CHECKERBOARD__
#define __OPENROX_CAMERA_CALIBRATION_CHECKERBOARD__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/image.h>
#include <baseproc/geometry/point/point2d.h>

#include <core/calibration/mono/calibration_perspective.h>
#include <core/calibration/mono/calibration_perspective_struct.h>
#include <core/model/model_checkerboard.h>

#include <user/identification/checkerboard/ident_checkerboard.h>

//! \addtogroup Camera_Calibration
//! @}

//! \brief The Rox_Camera_Calibration_Checkerboard_Struct object
struct Rox_Camera_Calibration_Checkerboard_Struct
{
   //! The calibration object 
   Rox_Calibration_Mono_Perspective calib;
   
   //! The tracking homography 
   Rox_MatSL3 G;
   
   //! The instrinsics parameters 
   Rox_MatUT3 intrinsics;

   //! The point list of the grid 
   Rox_Point2D_Double_Struct *ref_pts2D;
   
   //! The detected points 
   Rox_Point2D_Double_Struct *detected_pts;
   
   //! The reference 3D points 
   Rox_Point3D_Double_Struct *ref_pts3D;

   //! The number of points to detect 
   Rox_Sint nbpts;

   //! The image resolution 
   Rox_Sint image_width;
   
   //! The image resolution 
   Rox_Sint image_height;

   //! The checkerboard detector
   Rox_Ident_CheckerBoard checkerdetector;
};

//! Define the pointer of the rox_camera_calibration_checkerboard_Struct 
typedef struct Rox_Camera_Calibration_Checkerboard_Struct* Rox_Camera_Calibration_Checkerboard;

//! Create a new checkerboard calibration object based on grid detection
//! \param  [out]  calibration    calibration object
//! \param  [in ]  model The checkerboard model
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_checkerboard_new (
   Rox_Camera_Calibration_Checkerboard * calibration, 
   const Rox_Model_CheckerBoard model
);

//! Delete a camera calibration
//! \param  [out]  calibration    calibration object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_checkerboard_del (
   Rox_Camera_Calibration_Checkerboard * calibration
);

//! Add an image to the calibration set
//! \param  [out]  calibration    Calibration object
//! \param  [in ]  image          Calibration image with a visible grid
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_checkerboard_add_image (
   Rox_Camera_Calibration_Checkerboard calibration, 
   const Rox_Image image
);

//! Add externally detected points to the calibration set
//! \param [in]  calibration      Calibration object
//! \param [in]  n_points         number of points externally detected
//! \param [in]  points           pointer to a buffer containing the externally detected points
//! \param [in]  image_width      width of the image
//! \param [in]  image_height     height of the image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_camera_calibration_checkerboard_add_points (
   Rox_Camera_Calibration_Checkerboard calibration, 
   const Rox_Sint n_points, 
   const Rox_Point2D_Double points, 
   const Rox_Sint image_width, 
   const Rox_Sint image_height 
);

//! Perform camera calibration of desired parameter
//! \param  [out]  calibration    Calibration object
//! \param  [in ]  method         Number of parameters to be calibrated
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_checkerboard_make(
   Rox_Camera_Calibration_Checkerboard calibration, 
   const Rox_Sint method
);

//! Get a copy of the estimated intrinsic parameters
//! \param  [out]  intrinsics     Copy of the estimated parameters
//! \param  [in ]  calibration    calibration object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_checkerboard_get_intrinsics(
   Rox_MatUT3 intrinsics, 
   const Rox_Camera_Calibration_Checkerboard calibration
);

//! Get a copy of the estimated pose parameters
//! \param  [out]  pose           Copy of the estimated pose
//! \param  [in ]  calibration    Calibration object
//! \param  [in ]  index          Camera id
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_camera_calibration_checkerboard_get_pose(
   Rox_MatSE3 pose, 
   const Rox_Camera_Calibration_Checkerboard calibration, 
   const Rox_Sint index
);

//! Get the mean reprojection error
//! \param  [out]  mean           the mean reprojection error
//! \param  [in ]  calibration    calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_camera_calibration_checkerboard_get_mean_error (
   Rox_Double * mean, 
   const Rox_Camera_Calibration_Checkerboard calibration
);

//! @}

#endif // __OPENROX_CAMERA_CALIBRATION_CHECKERBOARD__
