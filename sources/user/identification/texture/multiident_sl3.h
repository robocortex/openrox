//==============================================================================
//
//    OPENROX   : File multiident_sl3.h
//
//    Contents  : API of multiident_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MULTIIDENT_SL3_OBJECT__
#define __OPENROX_MULTIIDENT_SL3_OBJECT__

#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/image/image.h>

#include <core/model/model_single_plane.h>
#include <core/identification/templateident_sl3.h>

#include <user/sensor/camera/camera.h>

//! \ingroup Identification
//! \addtogroup multiident
//! @{

//! SE3 identifier structure 
struct Rox_Multi_Ident_SL3_Struct
{
   //! The SE3 internal identifier 
   Rox_Template_Ident_SL3 identifier;
};

//! SL3 identifier object 
typedef struct Rox_Multi_Ident_SL3_Struct * Rox_Multi_Ident_SL3;

//! Create a new object for identification of planes on SL(3)
//! \param [out] obj is the newly created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_sl3_new ( Rox_Multi_Ident_SL3 * obj );

//! Delete an identification object
//! \param [in] obj is the created identification object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_sl3_del ( Rox_Multi_Ident_SL3 * obj );

//! Set the current model to detect
//! \param [out] obj is the identification object
//! \param [in] model the template to detect
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_sl3_add_model ( Rox_Multi_Ident_SL3 obj, Rox_Image model );

//! Compile templates together, must be called one after the templates have been added.
//! \param [in] obj is the created identification object to use
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_sl3_compile ( Rox_Multi_Ident_SL3 obj );

//! Perform the identification on an image
//! \param [out] obj is the identification object
//! \param [in] current the image to detect template into
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_sl3_make ( Rox_Multi_Ident_SL3 obj, Rox_Image current );

//! Get the estimated homography and the id of the detected template
//! \param [out] homography The estimated homography of the detected template
//! \param [out] detected_id The detected template id
//! \param [in] obj The identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_sl3_get_homography ( Rox_MatSL3 homography, Rox_Uint * detected_id, Rox_Multi_Ident_SL3 obj );

//! @} 

#endif
