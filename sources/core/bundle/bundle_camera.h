//==============================================================================
//
//    OPENROX   : File bundle_camera.h
//
//  	Contents  : API of bundle_camera module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_BUNDLE_CAMERA__
#define __OPENROX_BUNDLE_CAMERA__

#include <generated/dynvec_bundle_measure.h>

//! \ingroup Vision
//! \addtogroup Bundle
//! @{

//! Bundle Camera struct 
struct Rox_Bundle_Camera_Struct
{
   //! pose relative to rig reference frame
   Rox_MatSE3 relative_pose;

   //! intrinsic parameters for camera 
   Rox_MatUT3 calib;

   //! Scale values 
   Rox_Double scaler;
};

//! Bundle Camera object 
typedef struct Rox_Bundle_Camera_Struct * Rox_Bundle_Camera;

//! Create a container object for a bundle camera
//! \param obj the created container pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_camera_new(Rox_Bundle_Camera * obj);

//! Delete a container object for a bundle camera
//! \param obj the container pointer to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_camera_del(Rox_Bundle_Camera * obj);

//! @} 

#endif
