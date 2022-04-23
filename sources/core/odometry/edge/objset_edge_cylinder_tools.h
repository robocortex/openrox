//==============================================================================
//
//    OPENROX   : File objset_edge_cylinder_tools.h
//
//    Contents  : API of objset edge cylinders module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_OBJSET_EDGE_CYLINDERS_TOOLS__
#define __OPENROX_OBJSET_EDGE_CYLINDERS_TOOLS__

#include <generated/objset_edge_cylinder.h>
#include <generated/objset_edge_cylinder_struct.h>

#include <baseproc/image/image.h>

#include <core/tracking/edge/tracking_cylinder.h>
#include <core/tracking/edge/edge_cylinder_site.h>
#include <core/tracking/edge/edge_cylinder.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Cylinders
//! @{

//! Append a 3D Cylinder to an objset set
//! \param  [in]  objset_edge_cylinder   objset object
//! \param  [in]  cylinder               the 3D cylinder to add in world coordinates
//! \param  [in]  sampling_step          The sampling step for a cylinder
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_cylinder_add_cylinder3d(
   Rox_ObjSet_Edge_Cylinder objset_edge_cylinder, 
   const Rox_Cylinder3D cylinder3d, 
   const Rox_Double sampling_step
);

//! Transform and project an objset
//! \param  [out]  objset_edge_cylinder  The objset object
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  cTo                   The pose to transform
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_cylinder_transform_project(
   Rox_ObjSet_Edge_Cylinder objset_edge_cylinder, 
   const Rox_MatUT3 K, 
   const Rox_MatSE3 cTo
);

//! Build errors from an objset
//! \param  [out]  res_error             The resulting errors
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  objset_edge_cylinder  The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_cylinder_build_error(
   Rox_Double ** res_error, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Cylinder objset_edge_cylinder
);

//! Build an interaction from an objset
//! \param  [out]  L                     The resulting interaction matrix
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  objset_edge_cylinder  The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_cylinder_build_interaction_matrix(
   Rox_Matrix L, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Cylinder objset_edge_cylinder
);

//! Build a linear system from an objset object
//! \param  [out]  JtJ                   TODO
//! \param  [out]  JtJ                   TODO
//! \param  [in ]  K                     The calibration matrix
//! \param  [in ]  objset_edge_cylinder  The objset object
//! \param  [out]  dverr                 TODO
//! \param  [out]  dvw                   TODO
ROX_API Rox_ErrorCode rox_objset_edge_cylinder_build_linear_system(
   Rox_Matrix JtJ, 
   Rox_Matrix Jte, 
   const Rox_MatUT3 K, 
   const Rox_ObjSet_Edge_Cylinder objset_edge_cylinder, 
   Rox_Double ** dverr, 
   Rox_Double ** dvw);

//! Get measures from an objset
//! \param  [out]  objset_edge_cylinder  The objset object
//! \param  [in ]  tracker               The tracker used
//! \param  [in ]  pose                  The pose to apply
//! \param  [in ]  calibration           The camera calibration
//! \param  [in ]  image                 The image to parse
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_cylinder_get_measures(
   Rox_ObjSet_Edge_Cylinder objset_edge_cylinder, 
   const Rox_Tracking_Cylinder tracker, 
   const Rox_MatSE3 pose, 
   const Rox_MatUT3 calibration, 
   const Rox_Image image
);

//! Get valid measures from an objset
//! \param  [out]  valid_measures        The variable to fill
//! \param  [in ]  objset_edge_cylinder  The objset object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_cylinder_get_valid_measures(
   Rox_Sint * valid_measures, 
   const Rox_ObjSet_Edge_Cylinder objset_edge_cylinder
);

//! Get the score (the best score is 0)
//! \param  [out]  score                 The score
//! \param  [in ]  odometry_cylinder     The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_objset_edge_cylinder_get_score(
   Rox_Double * score, 
   const Rox_ObjSet_Edge_Cylinder objset_edge_cylinder, 
   const Rox_Double score_max
);

//! @} 

#endif
