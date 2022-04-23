//==============================================================================
//
//    OPENROX   : File bundle.h
//
//    Contents  : API of bundle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BUNDLE__
#define __OPENROX_BUNDLE__

#include <generated/objset_bundle_camera.h>
#include <generated/objset_bundle_frame.h>
#include <generated/objset_bundle_measure.h>
#include <generated/objset_bundle_point.h>
#include <baseproc/geometry/point/point3d.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>

//! \ingroup Optimization
//! \addtogroup Bundle
//! @{

//! Checkerboard object 
typedef struct Rox_Bundle_Struct * Rox_Bundle;

//! Create a container object for a bundle
//! \param  [out]  bundle the created container pointer
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_new ( Rox_Bundle * bundle );

//! Delete a container object for a bundle
//! \param  [out]  bundle the container pointer to delete
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_del ( Rox_Bundle * bundle );

//! Add a new camera to the bundle
//! \param  []  inserted_idx returned index of added object
//! \param  []  bundle the bundle object
//! \param  []  relative pose of this camera relative to the reference frame of the rig
//! \param  []  calib the calibration matrix of the camera
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_add_camera ( 
   Rox_Uint * inserted_idx, 
   Rox_Bundle bundle, 
   Rox_MatSE3 relative, 
   Rox_MatUT3 calib
);

//! Add a new camera to the bundle
//! \param  []  inserted_idx returned index of added object
//! \param  []  bundle the bundle object
//! \param  []  pose pose of this rig frame to the reference frame
//! \param  []  is_fixed is this frame to be estimated
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_add_frame ( 
   Rox_Uint * inserted_idx, Rox_Bundle bundle, Rox_MatSE3 pose, Rox_Uint is_fixed);

//! Add a new point to the bundle
//! \param  []  inserted_idx returned index of added object
//! \param  []  bundle the bundle object
//! \param  []  coords coordinate of this point relative to the reference frame
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_add_point (
   Rox_Uint * inserted_idx, 
   Rox_Bundle bundle, 
   Rox_Point3D_Double coords
);

//! Add a new measurement to the bundle
//! \param  []  inserted_idx 	Index of the inserted measurement
//! \param  []  bundle 				The bundle object
//! \param  []  coords 			Coordinate of the measurement in pixels
//! \param  []  idx_camera		Index of the added camera
//! \param  []  idx_frame		Index of the added frame
//! \param  []  idx_point		Index of the added point
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_add_measurement (
   Rox_Uint * inserted_idx, 
   Rox_Bundle bundle, 
   Rox_Point2D_Double coords, 
   Rox_Uint idx_camera, 
   Rox_Uint idx_frame, 
   Rox_Uint idx_point
);

//! Optimize the bundle
//! \param  []  bundle 					The bundle object
//! \param  []  max_iterations 	Maximum number of iterations
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_optimize(Rox_Bundle bundle, Rox_Uint max_iterations);

//! Get the updated point coordinates
//! \param  [out] coords a pointer to the coordinates to update
//! \param  []  bundle the bundle object
//! \param  []  id the point id
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_get_point_result(Rox_Point3D_Double  coords, Rox_Bundle bundle, Rox_Uint id);

//! Get the updated camera pose
//! \param  [out] pose the pose to update
//! \param  []  bundle the bundle object
//! \param  []  id the camera id
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_get_pose_result(Rox_MatSE3 pose, Rox_Bundle bundle, Rox_Uint id);

//! Get the measure validity
//! \param  [out] validity a flag pointer
//! \param  []  bundle the bundle object
//! \param  []  id the measure id
//! \return An error code
ROX_API Rox_ErrorCode rox_bundle_get_measure_validity(Rox_Uint * validity, Rox_Bundle bundle, Rox_Uint id);

//! @} 

#endif
