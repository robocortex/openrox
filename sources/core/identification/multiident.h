//==============================================================================
//
//    OPENROX   : File multiident.h
//
//    Contents  : API of multiident module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MULTI_IDENT__
#define __OPENROX_MULTI_IDENT__

#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_uint.h>
#include <generated/objset_template_ident.h>
#include <generated/dynvec_point3d_double.h>
#include <generated/dynvec_point3d_float.h>

//! \addtogroup Identification
//! @{

//! Information about template identification pointer */
typedef struct Rox_Multi_Ident_Struct * Rox_Multi_Ident;

//! Create a new object for identification of a template
//! \param [out] multi_ident is the newly created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_new(Rox_Multi_Ident * multi_ident);

//! Delete an identification object
//! \param [in] multi_ident is the created identification object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_del(Rox_Multi_Ident * multi_ident);

//! Reset an identification object
//! \param [in] multi_ident is the created identification object to reset
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_reset(Rox_Multi_Ident multi_ident);

//! Add a template
//! \param [in] multi_ident is the created identification object to modify
//! \param [in] reference the reference image for the template
//! \param [in] calib_template the calibration of the reference image for the template
//! \param [in] use_affine try to match with affine distortion (longer)
//! \param [in] use_double_image try to match with larger zoom (longer)
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_add_template(Rox_Multi_Ident multi_ident, Rox_Array2D_Float reference, Rox_Array2D_Double calib_template, Rox_Uint use_affine, Rox_Uint use_double_image);

//! Compile templates together, must be called one after the templates have been added.
//! \param [in] multi_ident is the created identification object to use
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_compile(Rox_Multi_Ident multi_ident);

//! Try to identify templates in the live image
//! \param [in] multi_ident is the created identification object to use
//! \param [in] current the image to search inside
//! \param [in] dbl_image do we double current image ?
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_make(Rox_Multi_Ident multi_ident, Rox_Array2D_Float current, Rox_Uint dbl_image);

//! Get the template with most matches
//! \param [out] id the identified template with most matches
//! \param [in] multi_ident is the created identification object to use
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_find_next_best(Rox_Uint * id, Rox_Multi_Ident multi_ident);

//! Make 
//! \param [in] multi_ident is the created identification object to modify
//! \param [out] current_image
//! \param [out] dbl_image
//! \param [out] reference_points_meters_matched
//! \param [out] reference_points_meters_extracted
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_make_features(Rox_Multi_Ident multi_ident, Rox_Array2D_Float current_image, Rox_Uint dbl_image, Rox_DynVec_Point3D_Float reference_points_meters_matched, Rox_DynVec_Point3D_Double reference_points_meters_extracted);

//! Get
//! \return An error code
//! \param [out] 	sraid_ref
//! \param [in] 	list_match_flag
//! \param [in] 	multi_ident
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_get_selected_features(Rox_DynVec_SRAID_Feature sraid_ref, Rox_DynVec_Uint list_match_flag, Rox_Multi_Ident multi_ident);

//! @} 

#endif // __OPENROX_MULTI_IDENT_OBJECT__
