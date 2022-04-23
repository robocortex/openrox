//==============================================================================
//
//    OPENROX   : File ident_multiplane.h
//
//    Contents  : API of ident_multiplane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IDENT_MULTIPLANE__
#define __OPENROX_IDENT_MULTIPLANE__


#include <core/model/model_multi_plane.h>
#include <core/identification/templateident_se3.h>

#include <user/sensor/camera/camera.h>

//! \addtogroup Identification
//! @{

//! SE3 identifier structure 
struct Rox_Ident_Multi_Plane_Struct
{
   //! The SE3 internal identifier 
   Rox_Template_Ident_SE3 identifier;

   //! The camera calibration
   Rox_MatUT3 calib_camera;

   //! The pose 
   Rox_MatSE3 pose;
};

//! SL3 identifier structure pointer
typedef struct Rox_Ident_Multi_Plane_Struct * Rox_Ident_Multi_Plane;

//! Create a new object for identification of a 3D_model on SE(3)
//! \param [out] obj is the newly created identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ident_multi_plane_new(Rox_Ident_Multi_Plane * obj);

//! Delete an identification object
//! \param [in] obj is the created identification object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ident_multi_plane_del(Rox_Ident_Multi_Plane * obj);

//! Set the model to detect
//! \param [out] obj is the identification object
//! \param [in] model the 3D-model to detect
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ident_multi_plane_set_model(Rox_Ident_Multi_Plane obj, Rox_Model_Multi_Plane model);

//! Perform the identification on an image contained in a camera
//! \param [out] ident  The identification object
//! \param [in]    model  The identification result
//! \param [in]    camera The camera containing the image to detect template into
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ident_multi_plane_make(Rox_Ident_Multi_Plane ident, Rox_Model_Multi_Plane model, Rox_Camera camera);

//! Get the pose stored in the ident object
//! \param [out] pose  The pose
//! \param [in]  ident The identification object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ident_multi_plane_get_pose(Rox_MatSE3 pose, Rox_Ident_Multi_Plane ident);

//! @} 

#endif
