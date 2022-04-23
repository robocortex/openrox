//==============================================================================
//
//    OPENROX   : File edge_cylinder.h
//
//    Contents  : API of edge cylinder module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGE_CYLINDER__
#define __OPENROX_EDGE_CYLINDER__

#include <generated/dynvec_edge_cylinder_site.h>

#include <baseproc/geometry/cylinder/cylinder2d.h>
#include <baseproc/geometry/cylinder/cylinder3d.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/segment/segment3d_struct.h>

//!  \ingroup Odometry
//!  \addtogroup EdgeCylinder
//!  @{

//! Edge Cylinder Structure
struct Rox_Edge_Cylinder_Struct
{
   //! 3D cylinder in object coordinates 
   Rox_Cylinder3D cylinder3d_o;

   //! 3D cylinder in camera coordinates 
   Rox_Cylinder3D cylinder3d_c;

   //! Cylinder in the image (meters) 
   Rox_Cylinder2D cylinder2d_meters;

   //! Cylinder in the image (pixels)
   Rox_Cylinder2D cylinder2d_pixels;

   //! Tangent segment 1
   Rox_Segment3D_Struct tangent_segment3d_1;

   //! Tangent segment 2
   Rox_Segment3D_Struct tangent_segment3d_2;

   //! Measures at sampled points
   Rox_DynVec_Edge_Cylinder_Site sites_ellipse_1;

   //! Measures at sampled points
   Rox_DynVec_Edge_Cylinder_Site sites_ellipse_2;

   //! Measures at sampled points
   Rox_DynVec_Edge_Cylinder_Site sites_segment_1;

   //! Measures at sampled points
   Rox_DynVec_Edge_Cylinder_Site sites_segment_2;

   //! Bounding box minimal
   Rox_Point2D_Sint_Struct pmin;

   //! Bounding box minimal
   Rox_Point2D_Sint_Struct pmax;

   //! How many sites should be present
   Rox_Double expected_density;

   //! The step when sampling cylinders
   Rox_Sint sampling_step;
};

//! CAD Model structure
typedef struct Rox_Edge_Cylinder_Struct * Rox_Edge_Cylinder;

//! Create a new cad model constraint object
//! \param [out]  edge_cylinder         The pointer to the newly created object
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_cylinder_new(Rox_Edge_Cylinder * edge_cylinder, const Rox_Sint sampling_step);

//! Delete a cad model constraint
//! \param [out]  edge_cylinder         The pointer to the created object
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_cylinder_del(Rox_Edge_Cylinder * edge_cylinder);

//! Transform an edge cylinder from object coordinates to camera coordinates and compute its projection in the image
//! \param [out]  edge_cylinder         The edge cylinder
//! \param [in]   pose                 The cTo pose to transform from object coordinates to camera coordinates
//! \param [in]   calibration          The camera calibration for the image projection
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_cylinder_transform_project(Rox_Edge_Cylinder edge_cylinder, Rox_MatSE3 pose, Rox_MatUT3 calibration);

//! Sample sites along edge cylinder
//! \param [in]   edge_cylinder         The edge cylinder object
//! \param [in]   sampling_step        The distance between samples
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_cylinder_sample(Rox_Edge_Cylinder edge_cylinder);

//! Remove invalid state sites
//! \param [in]   edge_cylinder         An edge cylinder object
//! \return an error code 
ROX_API Rox_ErrorCode rox_edge_cylinder_clean(Rox_Edge_Cylinder edge_cylinder);

// ----------------------------------------------------------------------------
// TODO : the following functions are never used, are they really needed ?

//! Set an edge cylinder world coordinates
//! \param [out]  edge_cylinder         The edge cylinder object
//! \param [in]   cylinder
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_cylinder_set_cylinder3d(Rox_Edge_Cylinder edge_cylinder, Rox_Cylinder3D cylinder3d);

//! Update states of sites
//! \param [in]   edge_cylinder         An edge cylinder object
//! \return an error code
ROX_API Rox_ErrorCode rox_edge_cylinder_suppress_points(Rox_Edge_Cylinder edge_cylinder);
// ----------------------------------------------------------------------------

ROX_API Rox_ErrorCode rox_edge_cylinder_get_valid_measures(Rox_Sint * valid_measures, Rox_Edge_Cylinder edge_cylinder);

//! @} 

#endif
