//==============================================================================
//
//    OPENROX   : File tracking_segment.h
//
//    Contents  : API of tracking_segment module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//=============================================================================

#ifndef __OPENROX_TRACKING_SEGMENT__
#define __OPENROX_TRACKING_SEGMENT__

#include <core/tracking/edge/moving_edge_params.h>
#include <core/tracking/edge/moving_edge.h>
#include <core/tracking/edge/search_edge.h>
#include <core/tracking/edge/edge_segment.h>

//! \ingroup Tracking
//! \addtogroup Line
//! @{

//! Parameter that can be set by user (rox_tracking_segment_new)
typedef enum Rox_Tracking_Segment_Method
{
   RoxTrackingSegmentMethod_Moving_Edge,
   RoxTrackingSegmentMethod_Search_Edge,
} Rox_Tracking_Segment_Method;

//! Segment tracker structure
struct Rox_Tracking_Segment_Struct
{
   //! method for matching edges to segments
   Rox_Tracking_Segment_Method   method;
   
   //! Search edge tracker (only allocated if RoxTrackingSegmentMethod_Search_Edge method is set)
   Rox_Search_Edge               search_edge;

   //! Parameters for moving edges (only allocated if RoxTrackingSegmentMethod_Moving_Edge method is set)
   Rox_Moving_Edge_Params        params;

   //! Moving edge tracker (only allocated if RoxTrackingSegmentMethod_Moving_Edge method is set)
   Rox_Moving_Edge               medge;
};

//! Segment tracker pointer to structure
typedef struct Rox_Tracking_Segment_Struct * Rox_Tracking_Segment;

//! Create Segments tracker object
//! \param  [out]  tracking_segment     The pointer to the segmen tracker object
//! \param  [in ]  method               The method to use when trying to match segments to edges
//! \param  [in ]  search_range         The distance in number of pixels to search from each site
//! \param  [in ]  contrast_threshold   The contrast threshold to validate a convolution result
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_segment_new (
   Rox_Tracking_Segment * tracking_segment, 
   const Rox_Tracking_Segment_Method method, 
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold
);

//! Delete Segments tracker object
//! \param  [out]  tracking_segment   The pointer to the segment tracker object
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_segment_del ( Rox_Tracking_Segment * tracking_segment );

//! Perform tracking for this segment
//! \param  [out]  tracking_segment     The segment tracker object
//! \param  [in ]  image                The image to track into
//! \param  [in ]  segment              The segment to track
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_segment_make ( 
   Rox_Tracking_Segment tracking_segment, 
   Rox_Image image, 
   Rox_Edge_Segment edge_segment
);

ROX_API Rox_ErrorCode rox_tracking_segment_make_gradient (
   Rox_Tracking_Segment tracking_segment,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle,
   const Rox_Edge_Segment segment
);

//! Perform initialization for this segment
//! \param  [out]  tracking_segment     The segment tracker object
//! \param  [in ]  segment              The segment to track
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_segment_initialize (
   Rox_Tracking_Segment tracking_segment, 
   Rox_Edge_Segment segment
);

//! @} 

#endif // __OPENROX_TRACKING_SEGMENT__
