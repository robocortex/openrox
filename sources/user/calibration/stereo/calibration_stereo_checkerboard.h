//==============================================================================
//
//    OPENROX   : File calibration_stereo_checkerboard.h
//
//    Contents  : API of calibration_stereo_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CALIBRATION_CHECKERBOARD_STEREO__
#define __OPENROX_CALIBRATION_CHECKERBOARD_STEREO__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/image.h>

#include <core/calibration/stereo/stereo_calibration.h>
#include <core/model/model_checkerboard.h>

#include <user/identification/checkerboard/ident_checkerboard.h>

//! \brief The Rox_Calibration_Stereo_CheckerBoard_Struct object
//! \brief the calibration of Kl, Kr and lTr is made at the same time
struct Rox_Calibration_Stereo_CheckerBoard_Struct
{
   //! The stereo calibration object 
   Rox_Calibration_Stereo_Perspective stereo_calib;

   //! The left 3D-2D homography 
   Rox_MatSL3 Gl;

   //! The right 3D-2D homography 
   Rox_MatSL3 Gr;

   //! The checkerboard detector 
   Rox_Ident_CheckerBoard checkerdetector;

   //! The 3D model 
   Rox_Point3D_Double_Struct  *ref_pts_3D;
   
   //! The 2D model 
   Rox_Point2D_Double_Struct * ref_pts_2D;

   //! The detected model in the left image 
   Rox_Point2D_Double_Struct * left_detected_corners;
   
   //! The detected model in the right image 
   Rox_Point2D_Double_Struct * right_detected_corners;

   //! The model count 
   Rox_Sint nbpts;
};

//! Define the pointer of the Rox_Calibration_Stereo_CheckerBoard_Struct 
typedef struct Rox_Calibration_Stereo_CheckerBoard_Struct* Rox_Calibration_Stereo_CheckerBoard;

//! Create a new calibration object
//! \param  []   calibration_stereo_checkerBoard is the newly created calibration object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_new(Rox_Calibration_Stereo_CheckerBoard * calibration_stereo_checkerBoard, Rox_Model_CheckerBoard model);

//! Delete a calibration object
//! \param  [out] calibration_stereo_checkerBoard is the created calibration object to delete
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_del(Rox_Calibration_Stereo_CheckerBoard * calibration_stereo_checkerBoard);

//! Add the stereo images to the measure set
//! \param  [out] calibration_stereo_checkerBoard is the calibration object
//! \param  []   left The left image to add
//! \param  []   right The right image to add
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_add_images (
   Rox_Calibration_Stereo_CheckerBoard calibration_stereo_checkerBoard, 
   const Rox_Image left, 
   const Rox_Image right);

//! Perform the intrinsic parameter calibration for left and right camera and the intercamera pose calibration
//! \brief [out] calibration_stereo_checkerBoard is the calibration object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_make (
   Rox_Calibration_Stereo_CheckerBoard calibration_stereo_checkerBoard
);

//! Get a copy of the calibration results (intrinsic parameters plus pose)
//! \param  [out]  Kl The left intrinsic parameters
//! \param  [out]  Kr The right intrinsic parameters
//! \param  [out]  pose The pose between the two camera
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_stereo_checkerboard_get_results (
   Rox_MatUT3 Kl, 
   Rox_MatUT3 Kr, 
   Rox_MatSE3 pose, 
   const Rox_Calibration_Stereo_CheckerBoard calibration_stereo_checkerBoard
);

#endif // __OPENROX_CALIBRATION_CHECKERBOARD_STEREO__
