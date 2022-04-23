//==============================================================================
//
//    OPENROX   : File objset_edge_segment_tools.h
//
//    Contents  : API of odometry_segments module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_OBJSET_EDGE_SEGMENT_TOOLS__
#define __OPENROX_OBJSET_EDGE_SEGMENT_TOOLS__

#include <generated/objset_edge_segment.h>
#include <generated/objset_edge_segment_struct.h>

#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/segment/segment3d.h>

#include <core/tracking/edge/tracking_segment.h>
#include <core/tracking/edge/edge_segment_site.h>
#include <core/tracking/edge/edge_segment.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Segments
//! @{

//! Append a 3D Segment to an obj set
//! \param  [out]  objset_edge_segment     The objset object
//! \param  [in ]  segment                 The 3D segment to add in world coordinates
//! \param  [in ]  sampling_step           The sampling step for a segment
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_segment_add_segment3d (
   Rox_ObjSet_Edge_Segment objset_edge_segment, 
   const Rox_Segment3D segment3d, 
   const Rox_Double sampling_step
);

//! Get measures from an objset
//! \param  [out]  objset_edge_segment   The objset object
//! \param  [in ]  tracker               The tracker used
//! \param  [in ]  pose                  The pose to apply
//! \param  [in ]  calibration           The camera calibration
//! \param  [in ]  image                 The image to parse
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_segment_get_measures (
   Rox_ObjSet_Edge_Segment objset_edge_segment, 
   const Rox_Tracking_Segment tracker, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 calibration, 
   const Rox_Image image
);

ROX_API Rox_ErrorCode rox_objset_edge_segment_get_measures_gradient (
   Rox_ObjSet_Edge_Segment objset_edge_segment,
   const Rox_Tracking_Segment tracker,
   const Rox_MatSE3 pose,
   const Rox_MatUT3 calibration,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle
);

//! Get valid measures from an objset
//! \param  [out]  valid_measures       The variable to fill
//! \param  [in ]  objset_edge_segment  The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_segment_get_valid_measures (
   Rox_Sint * valid_measures, 
   const Rox_ObjSet_Edge_Segment objset_edge_segment
);

//! Transform and project an objset
//! \param  [out]  objset_edge_segment  The objset object
//! \param  [in ]  K                    The calibration matrix
//! \param  [in ]  cTo                  The pose to transform
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_segment_transform_project(
   Rox_ObjSet_Edge_Segment objset_edge_segment, 
   const Rox_MatUT3 K, 
   const Rox_MatSE3 cTo
);

//! Build errors from an objset
//! \param  [out]  res_error             The resulting errors
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  objset_edge_segment   The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_segment_build_error (
   Rox_Double ** res_error, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Segment objset_edge_segment
);

//! Build an interaction from an objset
//! \param  [out]  L                     The resulting interaction matrix
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  objset_edge_segment   The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_segment_build_interaction_matrix (
   Rox_Matrix L, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Segment objset_edge_segment
);

//! Build a linear system from an objset object
//! \param  [out]  LtL                   TODO
//! \param  [out]  LtL                   TODO
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  objset_edge_segment   The objset object
//! \param  [out]  dverr                 TODO
//! \param  [out]  dvw                   TODO
ROX_API Rox_ErrorCode rox_objset_edge_segment_build_linear_system (
   Rox_Matrix LtL, 
   Rox_Matrix Lte, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Segment objset_edge_segment, 
   Rox_Double ** dverr, 
   Rox_Double ** dvw
);

//! Get the score (the best score is 0)
//! \param  [out]  score             The score
//! \param  [in ]  odometry_segments The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_segment_get_score (
   Rox_Double * score, 
   const Rox_ObjSet_Edge_Segment objset_edge_segment, 
   const Rox_Double score_max
);

//! @} 

#endif
