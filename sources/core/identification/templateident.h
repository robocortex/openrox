//==============================================================================
//
//    OPENROX   : File templateident.h
//
//    Contents  : API of templateident module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TEMPLATE_IDENT__
#define __OPENROX_TEMPLATE_IDENT__

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_sint.h>
#include <generated/dynvec_sraiddesc.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/objset_dynvec_sraid_feature.h>

//! \addtogroup Identification
//! @{

//! Information about template identification pointer
typedef struct Rox_Template_Ident_Struct * Rox_Template_Ident;

//! Create a new object for identification of a plane
//! \param [out] template_ident is the newly created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_new(Rox_Template_Ident * template_ident);

//! Delete an identification object
//! \param [in] template_ident is the created identification object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_del(Rox_Template_Ident * template_ident);

//! Set the current model to detect
//! \param [out] template_ident		The identification object
//! \param [in] model_image The template to detect
//! \param [in] calib_template
//! \param [in] dbl_image	Do we double the original image size
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_set_model(Rox_Template_Ident template_ident, Rox_Array2D_Float model_image, Rox_Array2D_Double calib_template, Rox_Uint dbl_image);

//! Set the current model to detect
//! \param [out] 	template_ident		The identification object
//! \param [in] 		model_image			The template to detect
//! \param [in] 		calib_template		The camera calibration associated to the template
//! \param [in] 		dbl_image 			Do we double the original image size
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_set_model_affine(Rox_Template_Ident template_ident, Rox_Array2D_Float model_image, Rox_Array2D_Double calib_template, Rox_Uint dbl_image);

//! @} 

#endif // __OPENROX_TEMPLATE_IDENT__
