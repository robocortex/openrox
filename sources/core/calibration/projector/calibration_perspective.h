//==============================================================================
//
//    OPENROX   : File calibration_perspective.h
//
//    Contents  : API of calibration_perspective module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CALIBRATION_PROJECTOR_PERSPECTIVE__
#define __OPENROX_CALIBRATION_PROJECTOR_PERSPECTIVE__

#include <generated/objset_array2d_double.h>
#include <baseproc/geometry/point/point2d.h>

//! \ingroup Projector
//! \defgroup Projector_Calibration Projector Calibration

//! \ingroup Projector_Calibration
//! \defgroup Calibration_Projector_Perspective Perspective Projector Calibration

//! \addtogroup Calibration_Projector_Perspective
//! @{

//! Define the pointer of the Rox_Calibration_Projector_Perspective_Struct  
typedef struct Rox_Calibration_Projector_Perspective_Struct * Rox_Calibration_Projector_Perspective;

//! Create a new projector calibration object
//! \param  []  obj               The newly created calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_new( Rox_Calibration_Projector_Perspective * obj );

//! Delete a projector calibration object
//! \param  []  obj               The created calibration object to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_del( Rox_Calibration_Projector_Perspective *obj );

//! Add the current homography and the current detected 3D points (with Z==0 and W sliced) to the calibration object
//! \param  []  obj               The calibration to use
//! \param  []  homography        The estimated homography
//! \param  []  obs2D             The list of sliced points with 3D information filled (3D in meters)
//! \param  []  count             The number of points in the list
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_add_measure( Rox_Calibration_Projector_Perspective obj, Rox_Array2D_Double homography, Rox_Point2D_Double  obs2D, Rox_Uint count );

//! Add the current 3D-2D homography (G) of the calibration model
//! \param  []  obj               The calibration to use
//! \param  []  homography        The estimated homography
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_add_homography(Rox_Calibration_Projector_Perspective obj, Rox_Array2D_Double homography);

//! Add the 3D (with Z==0 and W sliced) detected points of the current pose
//! \param  []  obj               The calibration to use
//! \param  []  obs2D             The list of sliced points with 3D information filled (2D in meters)
//! \param  []  count             The number of points in the list
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_add_current_points(Rox_Calibration_Projector_Perspective obj, Rox_Point2D_Double  obs2D, Rox_Uint count);

//! Set the 2D coordinates of the projected pattern
//! \param  []  obj   		       The calibration object to use
//! \param  []  refs2D   	       The list of the 2D pattern coordinates (in pixels)
//! \param  []  count		       The number of points in the list
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_set_model_points(Rox_Calibration_Projector_Perspective obj, Rox_Point2D_Double  refs2D, Rox_Uint count);

//! Compute a linear estimation of the intrinsic parameters using the homography set
//! \param  []  obj               The calibration object to use
//! \param  []  method            Define which intrinsic parameters to estimate (5: fu,fv,cu,cv,sk, 4: fu,fv,cu,cv, 3: f,cu,cv 2: fu,fv 1:f)
//! \return An error code
//! \todo Replace method by an enum
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_process_linear(Rox_Calibration_Projector_Perspective obj, Rox_Uint method);

//! Refine the linear estimation of the intrinsic parameters using a non linear approach (visual servoing to minimalize the projection error)
//! \param  []  obj               The calibration object to use
//! \param  []  method            Define which intrinsic parameters to estimate (5: fu,fv,cu,cv,sk, 4: fu,fv,cu,cv, 3: f,cu,cv 2: fu,fv 1:f)
//! \return An error code
//! \todo Replace method by an enum
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_process_nolinear(Rox_Calibration_Projector_Perspective obj, Rox_Uint method);

//! Refine the linear estimation of the intrinsic parameters using a non linear approach and normalized coordinates (visual servoing to minimalize the projection error)
//! \param  []  obj               The calibration object to use
//! \param  []  method            Define which intrinsic parameters to estimate (5: fu,fv,cu,cv,sk, 4: fu,fv,cu,cv, 3: f,cu,cv 2: fu,fv 1:f)
//! \return An error code
//! \todo Replace method by an enum
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_process_nolinear_normalize(Rox_Calibration_Projector_Perspective obj, Rox_Uint method);

//! Initialize all working buffers for the linear estimation
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_init_buffers( Rox_Calibration_Projector_Perspective obj );

//! Compute the linear estimation of the fu parameter using the homography set (the optical center is equal to the image center)
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_linear_fu( Rox_Calibration_Projector_Perspective obj );

//! Compute the linear estimation of the fu-fv parameters using the homography set (the optical center is equal to the image center and the skew is null)
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_linear_fu_fv( Rox_Calibration_Projector_Perspective obj );

//! Compute the linear estimation of the fu-fv-cu-cv parameters using the homography set (the skew is null)
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_linear_fu_fv_cu_cv( Rox_Calibration_Projector_Perspective obj );

//! Compute the linear estimation of all the intrinsic parameters using the homography set
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_linear_fu_fv_cu_cv_s( Rox_Calibration_Projector_Perspective obj );

//! Compute the linear estimation of f-cu-cv intrinsic parameters using the homography set (the skew is null)
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_linear_fu_cu_cv( Rox_Calibration_Projector_Perspective obj );

//! Compute the intrinsic parameters after the linear estimation: cholesky decomposition + un-normalization
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_update_linear_instrinsics( Rox_Calibration_Projector_Perspective obj );

//! Initialize the intrinsic parameters for the linear estimation
//! \param  []  obj               The calibration object to use
//! \param  []  K                 The initialization of the intrinsic parameters to copy
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_set_intrinsics(Rox_Calibration_Projector_Perspective obj, Rox_Array2D_Double K);

//! Get a copy of the estimated intrinsic parameters
//! \param  []  K                 The copy of the estimated intrinsic parameters
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_get_intrinsics(Rox_Array2D_Double K, Rox_Calibration_Projector_Perspective obj);

//! Compute the linear and no linear estimations of the intrinsic parameters
//! \param  []  obj               The calibration object to use
//! \param  []  method defines the intrinsic parameters to estimate
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_compute_parameters(Rox_Calibration_Projector_Perspective obj, Rox_Uint method);

//! For a given measure, computes the projection error using the estimated parameters and returns some useful statistics to determine the calibration quality
//! (i.e if the maximal error is less than one pixel, we consider that for the selected measure, the estimated parameters (pose + intrinsics) are correct)
//! \param  []  min               The minimal error in pixels between detected and reprojected models
//! \param  []  max               The maximal error in pixels between detected and reprojected models
//! \param  []  mean              The mean of the error in pixels between detected and reprojected models
//! \param  []  median            The median of the error in pixels between detected and reprojected models
//! \param  []  std               The standard deviation of the error in pixels between detected and reprojected models
//! \param  []  obj               The calibration object to use
//! \param  []  id                Defines the measure to use to make statistics of the measure set
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_get_statistics(Rox_Double *min, Rox_Double *max, Rox_Double *mean, Rox_Double *median, Rox_Double *std, Rox_Calibration_Projector_Perspective obj, Rox_Uint id);

//! Make a copy of the estimated intrinsic parameters and the pose set
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_save_data( Rox_Calibration_Projector_Perspective obj );

//! In case of a wrong calibration, restores the copied data to make a new calibration after the exclusion of the worst images (i.e if the reprojected model is too far of the detected model)
//! \param  []  obj               The calibration object to use
//! \return An error code
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_restore_data( Rox_Calibration_Projector_Perspective obj );

//! Check that the given homographies are correctly estimated by computing the reprojection error
//! \param  []  obj               The calibration object to use
//! \return An error code
//! \remarks The selection criteria is the maximal error in pixel (hard coded to 1)
ROX_API Rox_ErrorCode rox_calibration_projector_perspective_check_homographies( Rox_Calibration_Projector_Perspective obj );

//! @}

#endif // __OPENROX_CALIBRATION_PROJECTOR_PERSPECTIVE__
