//==============================================================================
//
//    OPENROX   : File calibration_projector_checkerboard.h
//
//    Contents  : API of calibration_projector_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CALIBRATION_PROJECTOR_CHECKERBOARD__
#define __OPENROX_CALIBRATION_PROJECTOR_CHECKERBOARD__

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>

#include <core/calibration/projector/calibration_perspective.h>
#include <core/calibration/projector/calibration_perspective_struct.h>
#include <core/model/model_projector_checkerboard.h>

#include <user/identification/checkerboard/ident_checkerboard.h>

//! \ingroup  CamProj
//! \defgroup Projector Projector

//! \ingroup  Projector
//! \defgroup Calibration_Projector Projector calibration

//! \addtogroup Calibration_Projector
//! @}

//! \brief The Rox_Calibration_Projector_CheckerBoard_Struct object
struct Rox_Calibration_Projector_CheckerBoard_Struct
{
   //! The calibration object
   Rox_Calibration_Projector_Perspective calib;
   
   //! The tracking homography
   Rox_MatSL3 G;
   
   //! The instrinsics parameters
   Rox_MatUT3 intrinsics;

   //! The point list of the grid
   Rox_Point2D_Double ref_pts2D;

   //! The number of points to detect
	Rox_Sint nbpts;

   //! The image resolution
   Rox_Sint image_width;
   
   //! The image resolution
   Rox_Sint image_height;

   //! The checkerboard detector
   Rox_Ident_CheckerBoard checkerdetector;

};

//! Define the pointer of the rox_calibration_projector_checkerboard_Struct
typedef struct Rox_Calibration_Projector_CheckerBoard_Struct * Rox_Calibration_Projector_CheckerBoard;

//! Create a new checkerboard calibration object based on grid projection
//! \param  [out]  calibration    calibration object
//! \param  [in ]  model          The checkerboard model
//! \param  [in ]  image_width    width of projected image
//! \param  [in ]  image_height   height of projected image
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_projector_checkerboard_new(
   Rox_Calibration_Projector_CheckerBoard * calibration, 
   const Rox_Model_Projector_CheckerBoard model 
);

//! Delete a projector calibration
//! \param  [out]  calibration calibration object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_projector_checkerboard_del (
   Rox_Calibration_Projector_CheckerBoard * calibration
);

//! Add an image to the calibration set
//! \param  [out]  calibration  Calibration object
//! \param  [in ]  obs2D        Calibration measures: 3D points with Z (==0) and W sliced
//! \param  [in ]  nobs         number of calibration measures
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_projector_checkerboard_add_image (
   Rox_Calibration_Projector_CheckerBoard calibration, 
   Rox_Point2D_Double  obs2D, 
   const Rox_Sint nobs 
);

//! Perform projector calibration of desired parameter
//! \param  [out]  calibration calibration object
//! \param  [in ]  method number of parameters to be calibrated
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_projector_checkerboard_make (
   Rox_Calibration_Projector_CheckerBoard calibration, 
   const Rox_Sint method
);

//! Get a copy of the estimated intrinsic parameters
//! \param  [out]  intrinsics Copy of the estimated parameters
//! \param  [in ]  calibration calibration object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_projector_checkerboard_get_intrinsics (
   Rox_MatUT3 intrinsics, 
   const Rox_Calibration_Projector_CheckerBoard calibration
);

//! Get a copy of the estimated pose parameters
//! \param  [out]  intrinsics Copy of the estimated pose
//! \param  [in ]  calibration calibration object
//! \param  [in ]  index projector id
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_projector_checkerboard_get_pose (
   Rox_MatSE3 pose, 
   const Rox_Calibration_Projector_CheckerBoard calibration, 
   const Rox_Sint index
);

//! @} 

#endif // __OPENROX_CALIBRATION_PROJECTOR_CHECKERBOARD__
