//==============================================================================
//
//    OPENROX   : File edge_point.h
//
//    Contents  : API of edge point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGE_POINT__
#define __OPENROX_EDGE_POINT__

#include <generated/dynvec_edge_point_site.h>

#include <baseproc/geometry/point/point2d.h>
#include <baseproc/geometry/line/line3d.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/line/line2d_struct.h>
#include <baseproc/geometry/line/line3d_struct.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>

//!  \ingroup Odometry
//!  \addtogroup EdgeSegment
//!  @{

//! Edge Point Structure
struct Rox_Edge_Point_Struct
{
   //! 3D line in reference coordinates 
   Rox_Line3D_Planes_Struct line3d_world;

   //! 3D line in camera coordinates 
   Rox_Line3D_Planes_Struct line3d_camera;

   //! 2D line in image coordinates (meters) 
   Rox_Line2D_Normal_Struct line2d_image_meters;

   //! 2D line in image coordinates (pixels)
   Rox_Line2D_Normal_Struct line2d_image_pixels;

   //! Point in image
   Rox_Point2D_Double_Struct point2d_image;

   //! Sampling 
   Rox_DynVec_Edge_Point_Site sites;

   //! Bounding box minimal
   Rox_Point2D_Sint_Struct pmin;

   //! Bounding box minimal
   Rox_Point2D_Sint_Struct pmax;
};

//! CAD Model  structure
typedef struct Rox_Edge_Point_Struct * Rox_Edge_Point;

//! Create a new cad model constraint object
//! \param  [out]  edge_point      The pointer to the newly created object
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_point_new ( Rox_Edge_Point * edge_point );

//! Delete a cad model constraint
//! \param  [out]  edge_point    The pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_point_del ( Rox_Edge_Point * edge_point );

//! Transform an edge point from world coordinates to camera coordinates and compute its projection in the image
//! \param  [out]  edge_point      The edge point object
//! \param  [in]   pose              The cTo pose
//! \param  [in]   calibration       The camera calibration object
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_point_transform_project ( Rox_Edge_Point edge_point, Rox_MatSE3 pose, Rox_MatUT3 calibration );

//! Remove invalid state sites
//! \param  [in]   edge_point      An edge point object
//! \return An error code 
ROX_API Rox_ErrorCode rox_edge_point_clean ( Rox_Edge_Point edge_point );

//! Set the edge point3d world coordinates
//! \param  [out]  edge_point    The edge point object
//! \param  [in]   point
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_point_set_point (
   Rox_Edge_Point edge_point, 
   Rox_Point2D_Double point2d,
   Rox_Line3D_Planes line3d
);

// ----------------------------------------------------------------------------
// TODO : the following functions are never used, are they really needed ?

//! Update states of sites
//! \param [in]   edge_point      An edge point object
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_point_suppress_points ( Rox_Edge_Point edge_point );
// ----------------------------------------------------------------------------

ROX_API Rox_ErrorCode rox_edge_point_get_valid_measures ( Rox_Sint * valid_measures, Rox_Edge_Point edge_point );

ROX_API Rox_ErrorCode rox_edge_point_log_points ( Rox_Edge_Point point, Rox_Sint id, Rox_Sint iter );

ROX_API Rox_ErrorCode rox_edge_point_sample(Rox_Edge_Point edge_point);
//! @} 

#endif
