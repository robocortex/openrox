//==============================================================================
//
//    OPENROX   : File multiident_se3.h
//
//    Contents  : API of multiident_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_MULTIIDENT_SE3_OBJECT__
#define __OPENROX_MULTIIDENT_SE3_OBJECT__

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>

#include <core/model/model_single_plane.h>
#include <core/identification/templateident_se3.h>

#include <user/sensor/camera/camera.h>

//! \ingroup Identification
//! \addtogroup multiident
//! @{

//! SE3 identifier structure 
struct Rox_Multi_Ident_SE3_Struct
{
   //! The SE3 internal identifier 
   Rox_Template_Ident_SE3 identifier;

   //! The camera calibration
   Rox_MatUT3 calib_camera;
};

//! SL3 identifier structure pointer 
typedef struct Rox_Multi_Ident_SE3_Struct * Rox_Multi_Ident_SE3;

//! Create a new object for identification of planes on SE(3)
//! \param [out] obj is the newly created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_se3_new(Rox_Multi_Ident_SE3 * obj);

//! Delete an identification object
//! \param [in] obj is the created identification object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_se3_del(Rox_Multi_Ident_SE3 * obj);

//! Set the current model to detect
//! \param [out] obj is the identification object
//! \param [in] model the template to detect
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_se3_add_model(Rox_Multi_Ident_SE3 obj, Rox_Model_Single_Plane model);

//! Compile templates together, must be called one after the templates have been added.
//! \param [in] obj is the created identification object to use
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_se3_compile(Rox_Multi_Ident_SE3 obj);

//! Perform the identification on an image
//! \param [out] obj is the identification object
//! \param [in] camera the camera containing the image to detect template into
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_se3_make(Rox_Multi_Ident_SE3 obj, Rox_Camera camera);

//! Get the estimated pose and the id of the detected template
//! \param [out] pose The estimated pose of the detected template
//! \param [out] detected_id The detected template id
//! \param [in] obj The identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_multi_ident_se3_get_pose(Rox_MatSE3 pose, Rox_Uint * detected_id, Rox_Multi_Ident_SE3 obj);

//! @} 

#endif
