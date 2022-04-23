//==============================================================================
//
//    OPENROX   : File ident_checkerboard.h
//
//    Contents  : API of ident_checkerboard module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_CHECKERBOARD__
#define __OPENROX_IDENT_CHECKERBOARD__

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/image/image.h>

#include <core/features/detectors/checkerboard/checkercorner_detect.h>
#include <core/features/detectors/checkerboard/checkerboard_detect.h>
#include <core/model/model_checkerboard.h>

//! \addtogroup Identification
//! @{

//! Checkerboard identifier structure 
struct Rox_Ident_CheckerBoard_Struct
{
    //! The corner detector 
    Rox_CheckerCorner_Detector detector;

    //! The checkerboard detector 
    Rox_CheckerBoard_Detector checkerdetector;

    //! The checkerboard width 
    Rox_Sint width;

    //! The checkerboard height 
    Rox_Sint height;
};

//! Checkerboard identifier structure pointer
typedef struct Rox_Ident_CheckerBoard_Struct * Rox_Ident_CheckerBoard;

//! Create a new object for checkerboard identification
//! \param ident_checkerboard is the newly created identification object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_checkerboard_new(
   Rox_Ident_CheckerBoard * ident_checkerboard
);

//! Delete an identification object
//! \param ident_checkerboard is the created identification object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_checkerboard_del(
   Rox_Ident_CheckerBoard * ident_checkerboard
);

//! Set the current model to detident_checkerboardect
//! \param  [out] ident_checkerboard 		The identification object
//! \param  [in] 	model_checkerboard 		The template to detect
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_checkerboard_set_model(
   Rox_Ident_CheckerBoard ident_checkerboard, 
   const Rox_Model_CheckerBoard model_checkerboard
);

//! Perform identification on an image
//! \param  [out] detected_corners  		The detected corners of the checkerboard
//! \param  [in]  ident_checkerboard     The identification object
//! \param  [in]  image             		The image to detect template into
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_checkerboard_make(
   Rox_Point2D_Double detected_corners, Rox_Ident_CheckerBoard ident_checkerboard, Rox_Image image
);

//! @} 

#endif // __OPENROX_IDENT_CHECKERBOARD__
