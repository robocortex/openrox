//==============================================================================
//
//    OPENROX   : File ident_texture_se3.h
//
//    Contents  : API of ident_texture_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_TEXTURE_SE3__
#define __OPENROX_IDENT_TEXTURE_SE3__

#include <generated/dynvec_point3d_double.h>

#include <baseproc/maths/linalg/matse3.h>

#include <core/model/model_single_plane.h>
#include <core/identification/templateident_se3.h>

#include <user/sensor/camera/camera.h>

//! \ingroup Identification
//! \defgroup Ident_Texture Identification Texture

//! \ingroup Ident_Texture 
//! \addtogroup Ident_Texture_SE3 Identification Texture SE3
//! @{

//! SL3 identifier structure pointer
typedef struct Rox_Ident_Texture_SE3_Struct * Rox_Ident_Texture_SE3;

//! \brief Create a new object for identification of a plane on SE(3)
//! \param  [out]  ident_texture        The newly created identification object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_se3_new (
   Rox_Ident_Texture_SE3 * ident_texture
);

//! Delete an identification object
//! \param  [out]  ident_texture        The created identification object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_se3_del (
   Rox_Ident_Texture_SE3 * ident_texture);

//! Set the current model to detect
//! \param  [out]  ident_texture The identification object
//! \param  [in ]  model                The template to detect
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_se3_set_model(
   Rox_Ident_Texture_SE3 ident_texture, Rox_Model_Single_Plane model);

//! Perform the identification on an image
//! \param  [out]  is_identified		    The identification result
//! \param  [out]  pose 					 The pose result
//! \param  [out]  ident_texture 		 The identification object
//! \param  [in ]  camera               The image to detect template into
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_se3_make (
   Rox_Sint * is_identified, 
   Rox_MatSE3 pose, 
   Rox_Ident_Texture_SE3 ident_texture, 
   Rox_Camera camera
);

//! Perform the identification on an image
//! \param  [out]  ident_texture           The identification result
//! \param  [in ]  sraid_ref_reloaded      The identification object
//! \param  [in ]  points_map_ref_reloaded The image to detect template into
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_ident_texture_se3_set_features (
   Rox_Ident_Texture_SE3 ident_texture, 
   Rox_DynVec_SRAID_Feature sraid_ref_reloaded, 
   Rox_DynVec_Point3D_Double points_map_ref_reloaded
);

//! @} 

#endif
