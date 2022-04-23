//==============================================================================
//
//    OPENROX   : File odometry_segments.h
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

#ifndef __OPENROX_ODOMETRY_SEGMENTS__
#define __OPENROX_ODOMETRY_SEGMENTS__

#include <generated/objset_edge_segment.h>
#include <generated/objset_edge_segment_struct.h>

#include <baseproc/geometry/segment/segment3d.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/image.h>

#include <core/tracking/edge/tracking_segment.h>
#include <core/tracking/edge/edge_segment_site.h>
#include <core/tracking/edge/edge_segment.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Segments
//! @{

//! 3D Segments based odometry structure
struct Rox_Odometry_Segments_Struct
{
   //! odometry estimated pose 
   Rox_MatSE3 pose;

   //! odometry virtual camera calibration 
   Rox_MatUT3 calibration;

   //! Set of segments 
   Rox_ObjSet_Edge_Segment objset_edge_segment;

   //! Segment edges tracker 
   Rox_Tracking_Segment tracker;

   //! Convergence flag
   Rox_Sint convergence;
};

//! CAD Model based odometry structure
typedef struct Rox_Odometry_Segments_Struct * Rox_Odometry_Segments;

//! Create a new odometry with 3D segments model constraint object
//! \param  [out]  odometry_segments       The pointer to the newly created object
//! \param  [in ]  search_range            The distance in number of pixels to search from each site
//! \param  [in ]  contrast_threshold      The contrast threshold to validate a convolution result
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_segments_new (
   Rox_Odometry_Segments * odometry_segments,
   const Rox_Double fu,
   const Rox_Double fv,
   const Rox_Double cu,
   const Rox_Double cv,
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold
);

//! Delete an odometry object with 3D segments model constraint
//! \param  [out]  odometry_segments       The pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_segments_del (
   Rox_Odometry_Segments * odometry_segments
);

//! Append a 3D Segment to the set
//! \param  [out]  odometry_segments       The odometry object
//! \param  [in ]  segment                 The 3D segment to add in world coordinates
//! \param  [in ]  sampling_step           The sampling step for a segment
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_segments_add_segment (
   Rox_Odometry_Segments odometry_segments, 
   Rox_Segment3D segment, 
   const Rox_Double sampling_step
);


ROX_API Rox_ErrorCode rox_odometry_segments_set_segments (
   Rox_Odometry_Segments odometry_segments,
   const Rox_Segment3D segment3d,
   const Rox_Sint nbs,
   const Rox_Double sampling_step
);

//! Set the pose cTo
//! \param  [out]  odometry_segments       The odometry object
//! \param  [in ]  cTo                     The pose cTo
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_segments_set_pose (
   Rox_Odometry_Segments odometry_segments,
   const Rox_MatSE3 cTo
);

//! Append a 3D Segment to the set
//! \param  [out]  odometry_segments       The odometry object
//! \param  [in ]  image                   The image to track into
//! \param  [in ]  max_iters               The number of iteration to make vvs estimation
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_segments_make (
   Rox_Odometry_Segments odometry_segments, 
   const Rox_Image image, 
   const Rox_Sint max_iters
);

ROX_API Rox_ErrorCode rox_odometry_segments_make_gradient (
   Rox_Odometry_Segments odometry_segments,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle,
   const Rox_Sint max_iters
);

//! Get the score (the best score is 0)
//! \param  [out]  score             The score
//! \param  [in ]  odometry_segments The odometry object
//! \return An error code
ROX_API Rox_ErrorCode rox_odometry_segments_get_score (
   Rox_Double * score, 
   const Rox_Odometry_Segments odometry_segments
);

ROX_API Rox_ErrorCode rox_odometry_segments_get_pose (
   Rox_MatSE3 cTo,
   const Rox_Odometry_Segments odometry_segments
);

//! @} 

#endif
