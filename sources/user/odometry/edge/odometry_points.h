//==============================================================================
//
//    OPENROX   : File odometry_points.h
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

#ifndef __OPENROX_ODOMETRY_POINTS__
#define __OPENROX_ODOMETRY_POINTS__

#include <generated/objset_edge_point.h>
#include <generated/objset_edge_point_struct.h>

#include <baseproc/geometry/line/line3d.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/image.h>

#include <core/tracking/edge/tracking_epoint.h>
#include <core/tracking/edge/edge_point_site.h>
#include <core/tracking/edge/edge_point.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Segments
//! @{

//! 3D Segments based odometry structure
struct Rox_Odometry_Points_Struct
{
   //! odometry estimated pose 
   Rox_MatSE3 pose;

   //! odometry virtual camera calibration 
   Rox_MatUT3 calibration;

   //! Set of points
   Rox_ObjSet_Edge_Point objset_edge_point;

   //! Point edges tracker 
   Rox_Tracking_EPoint tracker;

   //! Convergence flag
   Rox_Sint convergence;
};

//! CAD Model based odometry structure
typedef struct Rox_Odometry_Points_Struct * Rox_Odometry_Points;

//! Create a new odometry with 3D points model constraint object
//! \param  [out]  odometry_points       The pointer to the newly created object
//! \param  [in ]  search_range            The distance in number of pixels to search from each site
//! \param  [in ]  contrast_threshold      The contrast threshold to validate a convolution result
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_points_new (
   Rox_Odometry_Points * odometry_points,
   const Rox_Double fu,
   const Rox_Double fv,
   const Rox_Double cu,
   const Rox_Double cv,
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold
);

//! Delete an odometry object with 3D points model constraint
//! \param  [out]  odometry_points       The pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_points_del (
   Rox_Odometry_Points * odometry_points
);

//! Append a 3D Segment to the set
//! \param  [out]  odometry_points       The odometry object
//! \param  [in ]  point2d               The 2D point to add in camera coordinates
//! \param  [in ]  line3d                The 3D line to add in world coordinates
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_points_add_point (
   Rox_Odometry_Points  odometry_points, 
   const Rox_Point2D_Double          point2d,
   const Rox_Line3D_Planes           line3d
);


ROX_API Rox_ErrorCode rox_odometry_points_set_points (
   Rox_Odometry_Points odometry_points,
   const Rox_Point2D_Double          points2d,
   const Rox_Line3D_Planes           lines3d,
   const Rox_Sint             nbp
);

//! Set the pose cTo
//! \param  [out]  odometry_points       The odometry object
//! \param  [in ]  cTo                     The pose cTo
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_points_set_pose (
   Rox_Odometry_Points odometry_points,
   const Rox_MatSE3 cTo
);

//! Append a 3D Segment to the set
//! \param  [out]  odometry_points       The odometry object
//! \param  [in ]  image                   The image to track into
//! \param  [in ]  max_iters               The number of iteration to make vvs estimation
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_points_make (
   Rox_Odometry_Points odometry_points, 
   const Rox_Image image, 
   const Rox_Sint max_iters
);

ROX_API Rox_ErrorCode rox_odometry_points_make_gradient (
   Rox_Odometry_Points odometry_points,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle,
   const Rox_Sint max_iters
);

//! Get the score (the best score is 0)
//! \param  [out]  score             The score
//! \param  [in ]  odometry_points The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_points_get_score (
   Rox_Double * score, 
   const Rox_Odometry_Points odometry_points
);

ROX_API Rox_ErrorCode rox_odometry_points_get_pose (
   Rox_MatSE3 cTo,
   const Rox_Odometry_Points odometry_points
);

//! @} 

#endif
