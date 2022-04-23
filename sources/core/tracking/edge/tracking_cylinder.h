//==============================================================================
//
//    OPENROX   : File tracking_cylinder.h
//
//  	Contents  : API of tracking_cylinder module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//=============================================================================

#ifndef __OPENROX_TRACKING_CYLINDER__
#define __OPENROX_TRACKING_CYLINDER__

#include <core/tracking/edge/moving_edge_params.h>
#include <core/tracking/edge/moving_edge.h>
#include <core/tracking/edge/search_edge.h>
#include <core/tracking/edge/edge_cylinder.h>

//! \ingroup Tracking
//! \addtogroup Ellipse
//! @{

//! Parameter that can be set by user (rox_tracking_cylinder_new)
typedef enum Rox_Tracking_Cylinder_Method
{
   RoxTrackingCylinderMethod_Moving_Edge,
   RoxTrackingCylinderMethod_Search_Edge,
} Rox_Tracking_Cylinder_Method;

//! Ellipse tracker structure
struct Rox_Tracking_Cylinder_Struct
{
   //! method for matching edges to segments
   Rox_Tracking_Cylinder_Method   method;

   //! Moving edge tracker (only allocated if RoxTrackingCylinderMethod_SearchEllipse method is set)
   Rox_Search_Edge               search_edge;

   //! Parameters for moving edges (only allocated if RoxTrackingCylinderMethod_MovingEdge method is set)
   Rox_Moving_Edge_Params         params;

   //! Moving edge tracker (only allocated if RoxTrackingCylinderMethod_MovingEdge method is set)
   Rox_Moving_Edge                medge;
};

//! Ellipse tracker pointer to structure
typedef struct Rox_Tracking_Cylinder_Struct * Rox_Tracking_Cylinder;

//! Create Ellipses tracker object
//! \param [out] 	tracking_cylinder 		The pointer to the segmen tracker object
//! \param [in]   method               The method to use when trying to match segments to edges
//! \param [in]   search_range         The distance in number of pixels to search from each site
//! \param [in]   contrast_threshold   The contrast threshold to validate a convolution result
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_cylinder_new (
   Rox_Tracking_Cylinder * tracking_cylinder, 
   const Rox_Tracking_Cylinder_Method method, 
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold
);

//! Delete Ellipses tracker object
//! \param [out] 	tracking_cylinder 		The pointer to the segment tracker object
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_cylinder_del(Rox_Tracking_Cylinder * tracking_cylinder);

//! Perform tracking for this segment
//! \param  [out]  tracking_cylinder 		 The segment tracker object
//! \param  [in ]  image 		             The image to track into
//! \param  [in ]  segment 		          The segment to track
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_cylinder_make (
   Rox_Tracking_Cylinder tracking_cylinder, 
   Rox_Image image, 
   Rox_Edge_Cylinder cylinder
);

//! Perform initialization for this segment
//! \param  [out]  tracking_cylinder 	    The segment tracker object
//! \param  [in ]  image 		             The image to track into
//! \param  [in ]	 segment 		          The segment to track
//! \return An error code
ROX_API Rox_ErrorCode rox_tracking_cylinder_initialize (
   Rox_Tracking_Cylinder tracking_cylinder, 
   Rox_Image image, 
   Rox_Edge_Cylinder cylinder
);

//! @} 

#endif // __OPENROX_TRACKING_CYLINDER__
