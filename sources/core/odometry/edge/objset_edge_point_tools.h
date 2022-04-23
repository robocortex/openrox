//==============================================================================
//
//    OPENROX   : File objset_edge_point_tools.h
//
//    Contents  : API of odometry_points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_OBJSET_EDGE_POINT_TOOLS__
#define __OPENROX_OBJSET_EDGE_POINT_TOOLS__

#include <generated/objset_edge_point.h>
#include <generated/objset_edge_point_struct.h>

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/line/line3d.h>

#include <core/tracking/edge/tracking_epoint.h>
#include <core/tracking/edge/edge_point_site.h>
#include <core/tracking/edge/edge_point.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Points
//! @{

//! Append a 3D Point to an obj set
//! \param  [out]  objset_edge_point     The objset object
//! \param  [in ]  point                 The 3D point to add in world coordinates
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_point_add_point (
   Rox_ObjSet_Edge_Point objset_edge_point, 
   const Rox_Point2D_Double point2d,
   const Rox_Line3D_Planes line3d
);

//! Get measures from an objset
//! \param  [out]  objset_edge_point   The objset object
//! \param  [in ]  tracker               The tracker used
//! \param  [in ]  pose                  The pose to apply
//! \param  [in ]  calibration           The camera calibration
//! \param  [in ]  image                 The image to parse
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_point_get_measures (
   Rox_ObjSet_Edge_Point objset_edge_point, 
   const Rox_Tracking_EPoint tracker, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 calibration, 
   const Rox_Image image
);

ROX_API Rox_ErrorCode rox_objset_edge_point_get_measures_gradient (
   Rox_ObjSet_Edge_Point objset_edge_point,
   const Rox_Tracking_EPoint tracker,
   const Rox_MatSE3 pose,
   const Rox_MatUT3 calibration,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle
);

//! Get valid measures from an objset
//! \param  [out]  valid_measures       The variable to fill
//! \param  [in ]  objset_edge_point  The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_point_get_valid_measures (
   Rox_Sint * valid_measures, 
   const Rox_ObjSet_Edge_Point objset_edge_point
);

//! Transform and project an objset
//! \param  [out]  objset_edge_point  The objset object
//! \param  [in ]  K                    The calibration matrix
//! \param  [in ]  cTo                  The pose to transform
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_point_transform_project(
   Rox_ObjSet_Edge_Point objset_edge_point, 
   const Rox_MatUT3 K, 
   const Rox_MatSE3 cTo
);

//! Build errors from an objset
//! \param  [out]  res_error             The resulting errors
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  objset_edge_point   The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_point_build_error (
   Rox_Double ** res_error, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Point objset_edge_point
);

//! Build an interaction from an objset
//! \param  [out]  L                     The resulting interaction matrix
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  objset_edge_point   The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_point_build_interaction_matrix (
   Rox_Matrix L, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Point objset_edge_point
);

//! Build a linear system from an objset object
//! \param  [out]  LtL                   TODO
//! \param  [out]  LtL                   TODO
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  objset_edge_point     The objset object
//! \param  [out]  dverr                 TODO
//! \param  [out]  dvw                   TODO
ROX_API Rox_ErrorCode rox_objset_edge_point_build_linear_system (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Point objset_edge_point, 
   Rox_Double ** dverr, 
   Rox_Double ** dvw
);

//! Get the score (the best score is 0)
//! \param  [out]  score             The score
//! \param  [in ]  odometry_points The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_point_get_score (
   Rox_Double * score, 
   const Rox_ObjSet_Edge_Point objset_edge_point, 
   const Rox_Double score_max
);

//! @} 

#endif
