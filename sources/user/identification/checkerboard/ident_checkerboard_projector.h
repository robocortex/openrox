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

#ifndef __OPENROX_IDENT_CHECKERBOARD_PROJECTOR__
#define __OPENROX_IDENT_CHECKERBOARD_PROJECTOR__

#include "ident_checkerboard.h"

#include <core/features/detectors/checkerboard/checkercorner_detect.h>
#include <core/features/detectors/checkerboard/checkerboard_detect.h>
#include <core/model/model_projector_checkerboard.h>

//! \addtogroup Identification
//! @{

//! Checkerboard identifier structure 
struct Rox_Ident_CheckerBoard_Projector_Struct
{
   //! to be commented 
   Rox_Ident_CheckerBoard actual_detector;
};

//! Checkerboard identifier structure pointer
typedef struct Rox_Ident_CheckerBoard_Projector_Struct * Rox_Ident_CheckerBoard_Projector;

//! Create a new object for checkerboard identification
//! \param obj is the newly created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ident_checkerboard_projector_new(Rox_Ident_CheckerBoard_Projector * obj);

//! Delete an identification object
//! \param obj is the created identification object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ident_checkerboard_projector_del(Rox_Ident_CheckerBoard_Projector * obj);

//! Set the current model to detect
//! \param obj is the identification object
//! \param model the template to detect
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ident_checkerboard_projector_set_model(Rox_Ident_CheckerBoard_Projector obj, Rox_Model_Projector_CheckerBoard model);

//! Perform identification on an image
//! \param [out]   detected_corners    The detected corners of the checkerboard
//! \param [in]    obj                 The identification object
//! \param [in]    image               The image to detect template into
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ident_checkerboard_projector_make(
   Rox_Point2D_Double_Struct * detected_corners, Rox_Ident_CheckerBoard_Projector obj, Rox_Image image);

//! @} 

#endif
