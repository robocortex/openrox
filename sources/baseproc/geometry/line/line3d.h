//==============================================================================
//
//    OPENROX   : File line_3d.h
//
//    Contents  : Structure of line 3D module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE_3D__
#define __OPENROX_LINE_3D__

#include <baseproc/geometry/point/point3d.h>

//! \addtogroup Line
//! @{

//! 2 points index line pointer to structure
typedef struct Rox_Line3D_Index_Struct * Rox_Line3D_Index;

//! 2 planes based line pointer to structure
typedef struct Rox_Line3D_Planes_Struct * Rox_Line3D_Planes;

//! plucker based line pointer to structure
typedef struct Rox_Line3D_Plucker_Struct * Rox_Line3D_Plucker;

//! parametric based line pointer to structure
typedef struct Rox_Line3D_Parametric_Struct * Rox_Line3D_Parametric;

//! Compute a point on the 3D line given the parameter lambda
ROX_API Rox_ErrorCode rox_line3d_parametric_get_point3d (
   Rox_Point3D_Double point3d, 
   const Rox_Line3D_Parametric line3d_parametric, 
   const Rox_Double lambda
);

ROX_API Rox_ErrorCode rox_line3d_get_farthest_plane ( Rox_Double * a, Rox_Double * b, Rox_Double * c, Rox_Double * d, const Rox_Line3D_Planes line3D );

//! @}

#endif
