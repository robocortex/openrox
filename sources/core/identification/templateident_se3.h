//==============================================================================
//
//    OPENROX   : File templateident_se3.h
//
//    Contents  : API of templateident_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TEMPLATE_IDENT_SE3__
#define __OPENROX_TEMPLATE_IDENT_SE3__

#include <core/identification/multiident.h>
#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <generated/dynvec_sint.h>
#include <generated/dynvec_sraiddesc.h>
#include <generated/dynvec_point3d_float.h>
#include <generated/dynvec_point2d_float.h>
#include <generated/objset_dynvec_sraid_feature.h>

//! \addtogroup Identification
//! @{

//! SL3 identifier structure pointer
typedef struct Rox_Template_Ident_SE3_Struct * Rox_Template_Ident_SE3;

//! Create a new object for identification of a plane on SE(3)
//! \param [out] obj is the newly created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_se3_new(Rox_Template_Ident_SE3 * obj);

//! Delete an identification object
//! \param [in] obj is the created identification object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_se3_del(Rox_Template_Ident_SE3 * obj);

//! Reset templates
//! \param [in] obj is the created identification object to reset
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_se3_reset(Rox_Template_Ident_SE3 obj);

//! Set the current model to detect
//! \param [out] obj is the identification object
//! \param [in] model_image the template to detect
//! \param [in] calib_template the 2D-3D template calibration
//! \param [in] dbl_image do we double the original image size
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_se3_add_model(Rox_Template_Ident_SE3 obj, Rox_Array2D_Float model_image, Rox_Array2D_Double calib_template, Rox_Uint dbl_image);

//! Set the current model to detect
//! \param [out] obj is the identification object
//! \param [in] model_image the template to detect
//! \param [in] calib_template the 2D-3D template calibration
//! \param [in] dbl_image do we double the original image size
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_se3_add_model_affine(Rox_Template_Ident_SE3 obj, Rox_Array2D_Float model_image, Rox_Array2D_Double calib_template, Rox_Uint dbl_image);

//! Compile templates together, must be called one after the templates have been added.
//! \param [in] obj is the created identification object to use
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_se3_compile(Rox_Template_Ident_SE3 obj);

//! Perform identification on an image
//! \param [out] obj 			The identification object
//! \param [in] current_image	The image to detect template into
//! \param [in] calib_camera	Camera calibration 
//! \param [in] dbl_image		Do we double the original image size
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_template_ident_se3_make(Rox_Template_Ident_SE3 obj, Rox_Array2D_Float current_image, Rox_Array2D_Double calib_camera, Rox_Uint dbl_image);

//! Get next best pose
//! \param [out] pose    The pose of the identified template
//! \param [out] id      The id of the identified template
//! \param [in] obj      The identification object
//! \param [in] calib_camera The camera calibration
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_template_ident_se3_get_next_best_pose(Rox_Array2D_Double pose, Rox_Uint * id, Rox_Template_Ident_SE3 obj, Rox_Array2D_Double calib_camera);

//! @} 

#endif
