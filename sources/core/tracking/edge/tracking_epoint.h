//==============================================================================
//
//    OPENROX   : File tracking_epoint.h
//
//    Contents  : API of tracking_epoint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//=============================================================================

#ifndef __OPENROX_TRACKING_EPOINT__
#define __OPENROX_TRACKING_EPOINT__

#include <core/tracking/edge/moving_edge_params.h>
#include <core/tracking/edge/moving_edge.h>
#include <core/tracking/edge/search_edge.h>
#include <core/tracking/edge/edge_point.h>

//! \ingroup Tracking
//! \addtogroup Line
//! @{

//! Parameter that can be set by user (rox_tracking_epoint_new)
typedef enum Rox_Tracking_EPoint_Method
{
   RoxTrackingEPointMethod_Moving_Edge,
   RoxTrackingEPointMethod_Search_Edge,
} Rox_Tracking_EPoint_Method;

//! Point tracker structure
struct Rox_Tracking_EPoint_Struct
{
   //! method for matching edges to points
   Rox_Tracking_EPoint_Method   method;
   
   //! Search edge tracker (only allocated if RoxTrackingPointMethod_Search_Edge method is set)
   Rox_Search_Edge               search_edge;

   //! Parameters for moving edges (only allocated if RoxTrackingPointMethod_Moving_Edge method is set)
   Rox_Moving_Edge_Params        params;

   //! Moving edge tracker (only allocated if RoxTrackingPointMethod_Moving_Edge method is set)
   Rox_Moving_Edge               medge;
};

//! Point tracker pointer to structure
typedef struct Rox_Tracking_EPoint_Struct * Rox_Tracking_EPoint;

//! Create Points tracker object
//! \param  [out]  tracking_epoint     The pointer to the segmen tracker object
//! \param  [in ]  method               The method to use when trying to match points to edges
//! \param  [in ]  search_range         The distance in number of pixels to search from each site
//! \param  [in ]  contrast_threshold   The contrast threshold to validate a convolution result
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_epoint_new (
   Rox_Tracking_EPoint * tracking_epoint, 
   const Rox_Tracking_EPoint_Method method, 
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold
);

//! Delete Points tracker object
//! \param  [out]  tracking_epoint   The pointer to the point tracker object
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_epoint_del ( Rox_Tracking_EPoint * tracking_epoint );

//! Perform tracking for this point
//! \param  [out]  tracking_epoint     The point tracker object
//! \param  [in ]  image                The image to track into
//! \param  [in ]  point              The point to track
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_epoint_make ( 
   Rox_Tracking_EPoint tracking_epoint, 
   Rox_Image image, 
   Rox_Edge_Point edge_point
);

ROX_API Rox_ErrorCode rox_tracking_epoint_make_gradient (
   Rox_Tracking_EPoint tracking_epoint,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle,
   const Rox_Edge_Point point
);

//! Perform initialization for this point
//! \param  [out]  tracking_epoint     The point tracker object
//! \param  [in ]  point              The point to track
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_epoint_initialize (
   Rox_Tracking_EPoint tracking_epoint, 
   Rox_Edge_Point point
);

//! @} 

#endif // __OPENROX_TRACKING_SEGMENT__
