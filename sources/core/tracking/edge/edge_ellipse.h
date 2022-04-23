//==============================================================================
//
//    OPENROX   : File edge_ellipse.h
//
//    Contents  : API of edge ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGE_ELLIPSE__
#define __OPENROX_EDGE_ELLIPSE__

#include <generated/dynvec_edge_ellipse_site.h>
#include <generated/dynvec_point2d_double.h>

#include <baseproc/geometry/ellipse/ellipse2d_struct.h>
#include <baseproc/geometry/ellipse/ellipse3d.h>
#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/maths/linalg/matut3.h>

//#include <core/tracking/edge/movingedge.h>

//!  \ingroup Odometry
//!  \addtogroup EdgeEllipse
//!  @{

//! Edge Ellipse Structure
struct Rox_Edge_Ellipse_Struct
{
   //! 3D ellipse in object coordinates 
   Rox_Ellipse3D ellipse3d_o;

   //! 3D ellipse in camera coordinates 
   Rox_Ellipse3D ellipse3d_c;

   //! Ellipse in the image (meters) 
   Rox_Ellipse2D_Double_Struct ellipse2d_meters;

   //! Ellipse in the image (pixels)
   Rox_Ellipse2D_Double_Struct ellipse2d_pixels;

   //! Measures at sampled points
   Rox_DynVec_Edge_Ellipse_Site sites;

   //! Bounding box minimal
   Rox_Point2D_Sint_Struct pmin;

   //! Bounding box minimal
   Rox_Point2D_Sint_Struct pmax;

   //! How many sites should be present
   Rox_Double expected_density;

   //! The step when sampling ellipses
   Rox_Sint sampling_step;
};

//! CAD Model structure
typedef struct Rox_Edge_Ellipse_Struct * Rox_Edge_Ellipse;

//! Create a new cad model constraint object
//! \param  [out] edge_ellipse      The pointer to the newly created object
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_ellipse_new(Rox_Edge_Ellipse * edge_ellipse, const Rox_Sint sampling_step);

//! Delete a cad model constraint
//! \param  [out] edge_ellipse      The pointer to the created object
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_ellipse_del(Rox_Edge_Ellipse * edge_ellipse);

//! Transform an edge ellipse from object coordinates to camera coordinates and compute its projection in the image
//! \param  [out] edge_ellipse      The edge ellipse
//! \param  [in]  pose              The cTo pose to transform from object coordinates to camera coordinates
//! \param  [in]  calibration       The camera calibration for the image projection
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_ellipse_transform_project(Rox_Edge_Ellipse edge_ellipse, Rox_MatSE3 pose, Rox_MatUT3 calibration);

//! Sample sites along edge ellipse
//! \param  [in]  edge_ellipse      The edge ellipse object
//! \param  [in]  sampling_step     The distance between samples
//! \return An error code
ROX_API Rox_ErrorCode rox_edge_ellipse_sample(Rox_Edge_Ellipse edge_ellipse);

//! Remove invalid state sites
//! \param [in]  edge_ellipse       An edge ellipse object
//! \return an error code 
ROX_API Rox_ErrorCode rox_edge_ellipse_clean(Rox_Edge_Ellipse edge_ellipse);

// TODO : the following functions are never used, are they really needed ?
// ----------------------------------------------------------------------------
//! Set an edge ellipse world coordinates
//! \param [out] edge_ellipse       The edge ellipse object
//! \param [in]  ellipse
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_ellipse_set_ellipse3d(Rox_Edge_Ellipse edge_ellipse, Rox_Ellipse3D ellipse3d);
// ----------------------------------------------------------------------------

//! Update states of sites
//! \param [in]  edge_ellipse       An edge ellipse object
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_ellipse_suppress_points(Rox_Edge_Ellipse edge_ellipse);

ROX_API Rox_ErrorCode rox_edge_ellipse_get_valid_sites(Rox_DynVec_Point2D_Double valid_sites, Rox_Edge_Ellipse edge_ellipse);

ROX_API Rox_ErrorCode rox_edge_ellipse_append_valid_sites(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Edge_Ellipse edge_ellipse);

ROX_API Rox_ErrorCode rox_edge_ellipse_separated_inliers_outliers(Rox_Edge_Ellipse edge_ellipse, Rox_Ellipse2D_Parametric ellipse2d_parametric, const Rox_Double distance_threshold);

//! @} 

#endif
