//==============================================================================
//
//    OPENROX   : File objset_edge_ellipse_tools.h
//
//    Contents  : API of objset_edge_ellipse_toolso module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_OBJSET_EDGE_ELLIPSE_TOOLS__
#define __OPENROX_OBJSET_EDGE_ELLIPSE_TOOLS__

#include <generated/array2d_double.h>
#include <generated/objset_edge_ellipse.h>
#include <generated/objset_edge_ellipse_struct.h>

#include <baseproc/geometry/ellipse/ellipse3d.h>

#include <core/tracking/edge/tracking_ellipse.h>
#include <core/tracking/edge/edge_ellipse.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Ellipses
//! @{

//! Append a 3D ellipse to an objset object
//! \param  [out]  objset_edge_ellipse     The objset object
//! \param  [in ]  ellipse                 The 3D ellipse to add in world coordinates
//! \param  [in ]  sampling_step           The sampling step for an ellipse
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_ellipse_add_ellipse3d (
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse, 
   const Rox_Ellipse3D ellipse3d, 
   const Rox_Double sampling_step
);

//! Get measures from an objset
//! \param  [out]  objset_edge_ellipse     The objset object
//! \param  [in ]  tracker                 The tracker used
//! \param  [in ]  pose                    The pose to apply
//! \param  [in ]  calibration             The camera calibration
//! \param  [in ]  image                   The image to parse
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_ellipse_get_measures (
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse, 
   Rox_Tracking_Ellipse tracker, 
   Rox_MatSE3 pose, 
   Rox_MatUT3 calibration, 
   Rox_Image image
);

//! Get valid measures from an objset
//! \param  [out]  valid_measures       The variable to fill
//! \param  [in ]  objset_edge_segment  The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_ellipse_get_valid_measures (
   Rox_Sint * valid_measures, Rox_ObjSet_Edge_Ellipse objset_edge_ellipse);

//! Transform and project an objset
//! \param  [out]  objset_edge_ellipse     The objset object
//! \param  [in ]  K                       The calibration matrix
//! \param  [in ]  cTo                     The pose to transform
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_ellipse_transform_project (
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse, 
   Rox_MatUT3 K, 
   Rox_MatSE3 cTo
);

//! Build errors from an objset
//! \param  [out]  res_error               The resulting errors
//! \param  [in ]  K                       The calibration matrix
//! \param  [in ]  objset_edge_ellipse     The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_ellipse_build_error (
   Rox_Double ** res_error, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse
);

//! Build an interaction from an objset
//! \param  [out]  L                       The resulting interaction matrix
//! \param  [in ]  K                       The calibration matrix
//! \param  [in ]  objset_edge_ellipse     The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_ellipse_build_interaction_matrix (
   Rox_Matrix L, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse
);

//! Build a linear system from an objset object
//! \param  [out]  JtJ                     TODO
//! \param  [out]  JtJ                     TODO
//! \param  [in ]  K                       The calibration matrix
//! \param  [in ]  objset_edge_ellipse     The objset object
//! \pa ram [out]  dverr                   TODO
//! \param  [out]  dvw                     TODO
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_ellipse_build_linear_system (
   Rox_Matrix JtJ, 
   Rox_Matrix Jte, 
   Rox_MatUT3 K, 
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse, 
   Rox_Double ** dverr, 
   Rox_Double ** dvw
);

//! Get the score (the best score is 0)
//! \param  [out]  score                   The score
//! \param  [in ]  objset_edge_ellipse     The objset edge ellipse
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_ellipse_get_score (
   Rox_Double * score, 
   const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse, 
   const Rox_Double score_max
);

//! Get the valid site points
//! \param  [out]  dynvec_point2d          The score
//! \param  [in ]  objset_edge_ellipse     The objset edge ellipse
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_ellipse_get_valid_sites (
   Rox_DynVec_Point2D_Double dynvec_point2d, 
   const Rox_ObjSet_Edge_Ellipse objset_edge_ellipse
);

//! @} 

#endif
