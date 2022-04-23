//==============================================================================
//
//    OPENROX   : File edge_cylinder_site.h
//
//    Contents  : API of Edge Cylinder Site module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGE_CYLINDER_SITE__
#define __OPENROX_EDGE_CYLINDER_SITE__

#include <baseproc/geometry/point/point2d_struct.h>

//! \ingroup Odometry
//! \addtogroup EdgeCylinder
//! @{

//! Segment site structure
struct Rox_Edge_Cylinder_Site_Struct
{
   //! Site coordinates in image 
   Rox_Point2D_Double_Struct coords;

   //! State of site in last processing 
   Rox_Uint state;

   //! Last convolution result 
   Rox_Double previous_convolution;

   //! Expected rho 
   Rox_Double rhostar;

   //! Used tangent 
   Rox_Double alpha;
};

//! CAD Model  structure
typedef struct Rox_Edge_Cylinder_Site_Struct Rox_Edge_Cylinder_Site_Struct;

//!@} 

#endif
