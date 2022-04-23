//============================================================================
//
//    OPENROX   : File odometry_singleplane_struct.h
//
//    Contents  : Structure of odometry_singleplane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_ODOMETRY_SINGLE_PLANE_STRUCT__
#define __OPENROX_ODOMETRY_SINGLE_PLANE_STRUCT__

#include "odometry_singleplane.h"
#include "odometry_singleplane_params.h"

#include <generated/array2d_float.h>

#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/maths/linalg/matsl3.h>

#include <core/model/model_single_plane.h>

#include <user/sensor/camera/camera.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Single_Plane
//! \brief Structure and functions of the model 2D based odometry
//! @{

//! The Rox_Odometry_Single_Plane_Struct object
struct Rox_Odometry_Single_Plane_Struct
{
   //! The pose cTm representing the transformation of the model frame relative to the camera frame
   Rox_MatSE3 pose;

   //! Buffer pose intermediate
   Rox_MatSE3 posebuffer;

   //! The 2D - 3D calibration matrix
   Rox_MatUT3 calibration_template;

   //! The intermediate pose to move into the template pyramid
   Rox_MatUT3 zoom_calibration;

   //! Homography matrix
   Rox_MatSL3 homography;

   //! The matrix of the intrinsic parameters
   Rox_MatUT3 K;

   //! The normalized template
   Rox_Array2D_Float normalized_ref;

   //! The normalized image
   Rox_Array2D_Float normalized_cur;

   //! The zncc score between the reference and the current warped template
   Rox_Double score;

   //! The score threshold to decide if the template is tracked
   Rox_Double min_score;

   //! The prediction radius
   Rox_Sint prediction_radius;

   //! Maximal iterations of the ESM algorithm
   Rox_Sint miter;

   //! The higher pyramid level
   Rox_Sint init_pyr;

   //! The lower pyramid level
   Rox_Sint stop_pyr;

   //! Function pointer of the specific make function (depends on the definer usecase)
   Rox_ErrorCode (*_fptr_make)(Rox_Odometry_Single_Plane, Rox_Camera);

   //! Function pointer of the specific delete function (depends on the definer usecase)
   Rox_ErrorCode (*_fptr_del)(Rox_Odometry_Single_Plane *);

   //! Function pointer of the specific set_mask function (depends on the definer usecase)
   Rox_ErrorCode (*_fptr_set_mask) (Rox_Odometry_Single_Plane, Rox_Imask);

   //! The 3D corners of the model plane
   Rox_Point3D_Double_Struct  model_corners[4];
};

//! @}

#endif // __OPENROX_ODOMETRY_SINGLE_PLANE_STRUCT__
