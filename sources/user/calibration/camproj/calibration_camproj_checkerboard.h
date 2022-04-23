//==============================================================================
//
//    OPENROX   : File calibration_camproj_checkerboard.h
//
//    Contents  : API of calibration camproj checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license 
//
//==============================================================================

#ifndef __OPENROX_CHECKERBOARD_CAMPROJ__
#define __OPENROX_CHECKERBOARD_CAMPROJ__

#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>

#include <core/model/model_checkerboard.h>
#include <core/model/model_projector_checkerboard.h>
#include <core/calibration/camproj/calibration_camproj.h>

#include <user/identification/checkerboard/ident_checkerboard.h>
#include <user/identification/checkerboard/ident_checkerboard_projector.h>

//! \ingroup CamProj
//! \defgroup Camera_Projector_Calibration_Checkerboard Camera Projector Calibration Checkerboard

//! \addtogroup Camera_Projector_Calibration_Checkerboard
//! @}

//! The Rox_Calibration_CamProj_CheckerBoard_Struct object
struct Rox_Calibration_CamProj_CheckerBoard_Struct
{
   //! The camproj calibration object 
   Rox_Calibration_CamProj_Perspective camproj_calib;

   //! The cam 3D-2D homography 
   Rox_MatSL3 Gc;

   //! The printed checkerboard detector 
   Rox_Ident_CheckerBoard print_detector;

   //! The projectoed checkerboard detector 
   Rox_Ident_CheckerBoard_Projector proj_detector;

   //! The 3D printed model 
   Rox_Point3D_Double_Struct *cam_refs3D;

   //! The Z-coordinate sliced printed model 
   Rox_Point2D_Double_Struct *cam_slicedRefs;

   //! The 2D projected model 
   Rox_Point2D_Double_Struct *proj_refs2D;

   //! The detected printed model 
   Rox_Point2D_Double_Struct * print_detected_corners;

   //! The detected projected model 
   Rox_Point2D_Double_Struct * proj_detected_corners;

   //! The printed model number of points 
   Rox_Sint print_npts;

   //! The projected model number of points 
   Rox_Sint proj_npts;

   //! The projector image width 
   Rox_Sint proj_width;

   //! The projector image height 
   Rox_Sint proj_height;

   //! The input images inverse sampling. 0 or 1 means no sampling. 
   Rox_Double inverse_sampling;
};

//! Define the pointer of the Rox_Calibration_CamProj_CheckerBoard_Struct 
typedef struct Rox_Calibration_CamProj_CheckerBoard_Struct * Rox_Calibration_CamProj_CheckerBoard;

//! Create a new calibration object
//! \param  []  obj is the newly created calibration object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_new (
   Rox_Calibration_CamProj_CheckerBoard * obj, 
   const Rox_Model_CheckerBoard print_model, 
   const Rox_Model_Projector_CheckerBoard proj_model
);


//!  Delete a calibration object
//! \param  []  obj is the created calibration object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_del (
   Rox_Calibration_CamProj_CheckerBoard * obj
);


//!  Add the camproj images to the measure set
//! \param  []  obj   is the calibration object
//! \param  []  print Image of the printed pattern
//! \param  []  proj  Image of the projected pattern
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_add_images (
   Rox_Calibration_CamProj_CheckerBoard obj, 
   const Rox_Image print, 
   const Rox_Image proj
);


//! Add externally detected points to the calibration set
//! If an inverse sampling has been set the coordinates and image resolution are corrected
//! ( i.e. multiplied by the inverse sampling, that is the sampling )
//! \param  []  obj         the calibration object
//! \param  []  print_obs2D coordinates of externally detected printed points, in pixels
//! \param  []  print_npts  number of such points
//! \param  []  proj_obs2D  coordinates of externally detected projected points, in pixels
//! \param  []  proj_npts   number of such points
//! \param  []  cam_width   width of image
//! \param  []  cam_height  height of image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_add_points (
   Rox_Calibration_CamProj_CheckerBoard obj, 
   const Rox_Point2D_Double print_obs2D, 
   const Rox_Sint print_nobs, 
   const Rox_Point2D_Double proj_obs2D, 
   const Rox_Sint proj_nobs, 
   const Rox_Sint cam_width, 
   const Rox_Sint cam_height
);


//! Perform the intrinsic parameter linear calibration for camera and projector and the intercamera pose calibration
//! \param  []  obj is the calibration object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_make(
   Rox_Calibration_CamProj_CheckerBoard obj);

//! Get a copy of the calibrated intrinsic parameters, before refinement
//! \param  []  obj the calibration object
//! \param  []  Kc  camera intrinsics
//! \param  []  Kp  projector intrinsics
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_get_raw_intrinsics(
   Rox_Calibration_CamProj_CheckerBoard obj, 
   const Rox_MatUT3 Kc, 
   const Rox_MatUT3 Kp
);

//! Get a copy of the refined calibrated intrinsic parameters
//! \param  []  obj the calibration object
//! \param  []  Kc  camera intrinsics
//! \param  []  Kp  projector intrinsics
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_get_fine_intrinsics(
   Rox_Calibration_CamProj_CheckerBoard obj, 
   const Rox_MatUT3 Kc, 
   const Rox_MatUT3 Kp 
);

//! Get a copy of the calibrated relative pose parameters between camera and projector, not refined
//! \param  []  obj   the calibration object
//! \param  []  pTc   camera pose
//! \param  []  index index of the relative pose
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_get_raw_pTc (
   Rox_Calibration_CamProj_CheckerBoard obj, 
   const Rox_MatSE3 pTc, 
   const Rox_Sint index
);

//! Get a copy of the refined calibrated relative pose parameters between camera and projector
//! \param  []  obj   the calibration object
//! \param  []  pTc   camera pose
//! \param  []  index index of the relative pose
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_get_fine_pTc (
   Rox_Calibration_CamProj_CheckerBoard obj, 
   const Rox_MatSE3 pTc
);

//! Compute mean reprojection error, once the calibration has been made, with unrefined parameters
//! (i.e. Kp and pTc )
//! \param  []  obj        The calibration object to use
//! \param  []  error_cam  The errors for printed points
//! \param  []  error_proj The errors for projected points
//! \param  []  index      The index of the camera-projector couple on which errors are to be computed
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_check_linear_results (
   Rox_Calibration_CamProj_CheckerBoard obj, 
   Rox_Double * error_cam, 
   Rox_Double * error_proj, 
   const Rox_Sint index 
);

//! Compute mean and median reprojection error, once the calibration has been made,
//! with refined parameters (i.e. Kp and pTc )
//! For one or all images. If only one image, the median return value should be ignored.
//! \param  []  obj                   The calibration object to use
//! \param  []  error_cam             The mean error   for the camera
//! \param  []  mean_error_proj_fwd   The mean error   for camera points transfered    to projector points
//! \param  []  median_error_proj_fwd The median error for camera points transfered    to projector points
//! \param  []  mean_error_proj_bwd   The mean error   for projector points transfered to camera points
//! \param  []  median_error_proj_bwd The median error for projector points transfered to camera points
//! \param  []  index                 if (!= -1): the index of the camera-projector couple on which errors are to be computed. Otherwise all pairs are treated.
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_check_fine_results(
   Rox_Calibration_CamProj_CheckerBoard obj,
   Rox_Double*                          error_cam,
   Rox_Double*                          mean_error_proj_fwd,
   Rox_Double*                          median_error_proj_fwd,
   Rox_Double*                          mean_error_proj_bwd,
   Rox_Double*                          median_error_proj_bwd,
   const Rox_Sint                             index );

//! Set camera intrinsics, therefore disabling camera calibration
//! \param  []  obj  The calibration object to use
//! \param  []  Kin  The input 3x3 intrinsics parameters matrix
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_set_camera_intrinsics (
   Rox_Calibration_CamProj_CheckerBoard obj, 
   const Rox_MatUT3 Kin
);

//! Set input images inverse sampling. Input inverse sampling is used to rescale
//! detected corners.
//! \param  []  obj              The calibration object to use
//! \param  []  inverse_sampling Value to set
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_camproj_checkerboard_set_images_inverse_sampling (
   Rox_Calibration_CamProj_CheckerBoard obj, 
   const Rox_Double inverse_sampling
);

//! @}

#endif // __OPENROX_CHECKERBOARD_CAMPROJ__
