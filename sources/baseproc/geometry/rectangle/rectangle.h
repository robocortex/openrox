//==============================================================================
//
//    OPENROX   : File rectangle.h
//
//    Contents  : Structure of rectangle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RECTANGLE__
#define __OPENROX_RECTANGLE__

#include "rectangle_struct.h"

#include <baseproc/geometry/point/point3d.h>
#include <baseproc/geometry/point/point2d.h>

//! \ingroup Geometry
//! \addtogroup Rectangle
//! @{

//! Alias to a pointer to a rectangle structure
typedef struct Rox_Rect_Sint_Struct * Rox_Rect_Sint;

//! Alias to a pointer to a rectangle structure
typedef struct Rox_Rect_Real_Struct * Rox_Rect_Real;


ROX_API Rox_ErrorCode rox_rectangle3d_create_centered_plane_xright_ydown ( 
   Rox_Point3D_Double vertices, 
   const Rox_Double sizex, 
   const Rox_Double sizey 
);


ROX_API Rox_ErrorCode rox_rectangle3d_create_centered_plane_xright_yup ( 
   Rox_Point3D_Double vertices, 
   const Rox_Double sizex, 
   const Rox_Double sizey 
);


ROX_API Rox_ErrorCode rox_rectangle2d_create_centered_plane_xright_ydown ( 
   Rox_Point2D_Double vertices, 
   const Rox_Double sizex, 
   const Rox_Double sizey 
);


ROX_API Rox_ErrorCode rox_rectangle2d_create_centered_plane_xright_yup ( 
   Rox_Point2D_Double vertices, 
   const Rox_Double sizex, 
   const Rox_Double sizey 
);

ROX_API Rox_ErrorCode rox_rectangle2d_create_image_coordinates ( 
   Rox_Point2D_Double vertices, 
   const Rox_Double cols, 
   const Rox_Double rows 
);

//! @}

#endif
