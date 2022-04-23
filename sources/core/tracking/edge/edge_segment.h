//==============================================================================
//
//    OPENROX   : File edge_segment.h
//
//    Contents  : API of edge segment module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGE_SEGMENT__
#define __OPENROX_EDGE_SEGMENT__

#include <generated/dynvec_edge_segment_site.h>

#include <baseproc/geometry/segment/segment2d_struct.h>
#include <baseproc/geometry/segment/segment3d.h>
#include <baseproc/geometry/segment/segment3d_struct.h>
#include <baseproc/geometry/line/line2d_struct.h>
#include <baseproc/geometry/line/line3d_struct.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>

//!  \ingroup Odometry
//!  \addtogroup EdgeSegment
//!  @{

//! Edge Segment Structure
struct Rox_Edge_Segment_Struct
{
   //! 3D segment in reference coordinates 
   Rox_Segment3D_Struct segment_world;

   //! 3D segment in camera coordinates 
   Rox_Segment3D_Struct segment_cam;

   //! 3D line in camera coordinates 
   Rox_Line3D_Planes_Struct line;

   //! 2D line in image coordinates (meters) 
   Rox_Line2D_Normal_Struct line_image_meters;

   //! 2D line in image coordinates (pixels)
   Rox_Line2D_Normal_Struct line_image_pixels;

   //! Segment extremities in image
   Rox_Segment2D_Struct segment_image;

   //! Sampling 
   Rox_DynVec_Edge_Segment_Site sites;

   //! Bounding box minimal
   Rox_Point2D_Sint_Struct pmin;

   //! Bounding box minimal
   Rox_Point2D_Sint_Struct pmax;

   //! How many sites should be present
   Rox_Double expected_density;

   //! The step when sampling segments
   Rox_Sint sampling_step;
};

//! CAD Model  structure
typedef struct Rox_Edge_Segment_Struct * Rox_Edge_Segment;

//! Create a new cad model constraint object
//! \param  [out]  edge_segment      The pointer to the newly created object
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_segment_new(Rox_Edge_Segment * edge_segment, const Rox_Sint sampling_step);

//! Delete a cad model constraint
//! \param  [out]  edge_segment    The pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_segment_del(Rox_Edge_Segment * edge_segment);

//! Transform an edge segment from world coordinates to camera coordinates and compute its projection in the image
//! \param  [out]  edge_segment      The edge segment object
//! \param  [in]   pose              The cTo pose
//! \param  [in]   calibration       The camera calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_segment_transform_project(Rox_Edge_Segment edge_segment, Rox_MatSE3 pose, Rox_MatUT3 calibration);

//! Sample sites along edge segment
//! \param [in]   edge_segment      A edge segment object
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_segment_sample(Rox_Edge_Segment edge_segment);

//! Remove invalid state sites
//! \param  [in]   edge_segment      An edge segment object
//! \return An error code 
ROX_API Rox_ErrorCode rox_edge_segment_clean(Rox_Edge_Segment edge_segment);

//! Set the edge segment3d world coordinates
//! \param  [out]  edge_segment    The edge segment object
//! \param  [in]   segment
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_segment_set_segment3d (
   Rox_Edge_Segment edge_segment, 
   Rox_Segment3D segment3d
);

// ----------------------------------------------------------------------------
// TODO : the following functions are never used, are they really needed ?

//! Update states of sites
//! \param [in]   edge_segment      An edge segment object
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_segment_suppress_points(Rox_Edge_Segment edge_segment);
// ----------------------------------------------------------------------------

ROX_API Rox_ErrorCode rox_edge_segment_get_valid_measures(Rox_Sint * valid_measures, Rox_Edge_Segment edge_segment);

ROX_API Rox_ErrorCode rox_edge_segment_log_segments(Rox_Edge_Segment seg, Rox_Sint id, Rox_Sint iter);

//! @} 

#endif // __OPENROX_EDGE_SEGMENT__
