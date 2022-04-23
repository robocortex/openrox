//==============================================================================
//
//    OPENROX   : File edge_point_site.h
//
//    Contents  : API of EdgePointSite module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EDGE_POINT_SITE__
#define __OPENROX_EDGE_POINT_SITE__

#include <baseproc/geometry/point/point2d_struct.h>

//! \ingroup Odometry
//! \addtogroup EdgePoint
//! @{

//! Segment site structure
struct Rox_Edge_Point_Site_Struct
{
   //! Site coordinates in image (in pixels)
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
typedef struct Rox_Edge_Point_Site_Struct Rox_Edge_Point_Site_Struct;

typedef struct Rox_Edge_Point_Site_Struct * Rox_Edge_Point_Site;

//!@} 

#endif
