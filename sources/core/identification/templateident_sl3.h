//==============================================================================
//
//    OPENROX   : File templateident_sl3.h
//
//    Contents  : API of templateident_sl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TEMPLATE_IDENT_SL3__
#define __OPENROX_TEMPLATE_IDENT_SL3__

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_sint.h>
#include <generated/dynvec_sraiddesc.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/objset_dynvec_sraid_feature.h>
#include <core/identification/multiident.h>

//! \addtogroup Identification
//! @{

//! SL3 identifier structure pointer
typedef struct Rox_Template_Ident_SL3_Struct * Rox_Template_Ident_SL3;

//! Create a new object for identification of a plane on SL(3)
//! \param [out] obj is the newly created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_sl3_new(Rox_Template_Ident_SL3 * obj);

//! Delete an identification object
//! \param [in] obj is the created identification object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_sl3_del(Rox_Template_Ident_SL3 * obj);

//! Reset templates
//! \param [in] obj is the created identification object to reset
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_sl3_reset(Rox_Template_Ident_SL3 obj);

//! Set the current model to detect
//! \param [out] obj is the identification object
//! \param [in] model_image the template to detect
//! \param [in] dbl_image do we double the original image size
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_sl3_add_model(Rox_Template_Ident_SL3 obj, Rox_Array2D_Float model_image, Rox_Uint dbl_image);

//! Set the current model to detect
//! \param [out] obj is the identification object
//! \param [in] model_image the template to detect
//! \param [in] dbl_image do we double the original image size
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_sl3_add_model_affine(Rox_Template_Ident_SL3 obj, Rox_Array2D_Float model_image, Rox_Uint dbl_image);

//! Perform identification on an image
//! \param [out] obj is the identification object
//! \param [in] current_image the image to detect template into
//! \param [in] dbl_image do we double the original image size
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_sl3_make(Rox_Template_Ident_SL3 obj, Rox_Array2D_Float current_image, Rox_Uint dbl_image);

//! Perform identification on an image
//! \param [out] homography the identified homography
//! \param [out] id the identified template id
//! \param [out] obj is the identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_sl3_get_next_best_homography(Rox_Array2D_Double homography, Rox_Uint *id, Rox_Template_Ident_SL3 obj);

//! Compile templates together, must be called one after the templates have been added.
//! \param [in] obj is the created identification object to use
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_sl3_compile(Rox_Template_Ident_SL3 obj);

//! @} 

#endif
