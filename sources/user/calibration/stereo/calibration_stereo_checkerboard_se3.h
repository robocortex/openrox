//==============================================================================
//
//    OPENROX   : File calibration_stereo_checkerboard_se3.h
//
//    Contents  : API of calibration_stereo_checkerboard_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CHECKERBOARD_STEREO_SE3__
#define __OPENROX_CHECKERBOARD_STEREO_SE3__

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/image/image.h>

#include <core/calibration/mono/calibration_perspective.h>
#include <core/calibration/mono/calibration_perspective_struct.h>
#include <core/calibration/stereo/stereo_calibration_se3.h>
#include <core/model/model_checkerboard.h>

#include <user/identification/checkerboard/ident_checkerboard.h>

//! The Rox_Calibration_Stereo_CheckerBoard_SE3_Struct object
//! \brief the calibration of Kl, Kr is done first and then and lTr is computed
struct Rox_Calibration_Stereo_CheckerBoard_SE3_Struct
{
    //! The left perspective camera calibration
    Rox_Calibration_Mono_Perspective left;

    //! The right perspective camera calibration
    Rox_Calibration_Mono_Perspective right;

    //! The stereo camera calibration
    Rox_Calibration_Stereo_Perspective_SE3 stereo;

    //! The 2D - 3D transformation of the left image
    Rox_MatSL3 Gl;

    //! The 2D - 3D transformation of the right image
    Rox_MatSL3 Gr;

    //! The checkerboard detector
    Rox_Ident_CheckerBoard checkerdetector;

    //! The 3D model
    Rox_Point3D_Double_Struct *ref_pts_3D;
    
    //! The 2D model
    Rox_Point2D_Double_Struct *ref_pts_2D;

    //! The detected model in the left  image
    Rox_Point2D_Double_Struct *left_detected_corners;
    
    //! The detected model in the right image
    Rox_Point2D_Double_Struct *right_detected_corners;

    //! The model count
    Rox_Sint nbpts;
};

//! Define the pointer of the Rox_Calibration_Stereo_CheckerBoard_SE3_Struct
typedef struct Rox_Calibration_Stereo_CheckerBoard_SE3_Struct* Rox_Calibration_Stereo_CheckerBoard_SE3;

//! Create a new calibration object
//! \param  []  obj is the newly created calibration object
//! \param  []  model the checkerboard model
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_new (
    Rox_Calibration_Stereo_CheckerBoard_SE3 * obj, 
    const Rox_Model_CheckerBoard model
);

//! Delete a calibration object
//! \param  []  obj is the created calibration object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_del (
    Rox_Calibration_Stereo_CheckerBoard_SE3 * obj
);

//! Add the left image to the measure set
//! \param  []  obj is the calibration object
//! \param  []  left The image to add
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_add_left_image (
    Rox_Calibration_Stereo_CheckerBoard_SE3 obj, 
    const Rox_Image left
);

//! Add the right image to the measure set
//! \param  []  obj is the calibration object
//! \param  []  right The right image to add
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_add_right_image (
    Rox_Calibration_Stereo_CheckerBoard_SE3 obj, 
    const Rox_Image right
);

//! Add the stereo images to the measure set
//! \param  []  obj is the calibration object
//! \param  []  left The left image to add
//! \param  []  right The right image to add
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_add_stereo_images (
    Rox_Calibration_Stereo_CheckerBoard_SE3 obj, 
    const Rox_Image left, 
    const Rox_Image right
);

//! Perform the intrinsic parameter calibration for left and right camera and the intercamera pose calibration
//! \param  []  obj is the calibration object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_make (
    Rox_Calibration_Stereo_CheckerBoard_SE3 obj
);

//! Get a copy of the calibration results (intrinsic parameters plus pose)
//! \param  []  Kl The left intrinsic parameters
//! \param  []  Kr The right intrinsic parameters
//! \param  []  pose The pose between the two camera
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_se3_get_results (
    Rox_MatUT3 Kl, 
    Rox_MatUT3 Kr, 
    Rox_MatSE3 pose, 
    const Rox_Calibration_Stereo_CheckerBoard_SE3 obj
);

#endif // __OPENROX_CHECKERBOARD_STEREO_SE3__
