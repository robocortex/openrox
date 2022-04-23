//==============================================================================
//
//    OPENROX   : File linsys_point_to_line_matse3.h
//
//    Contents  : API of linsys_point_to_line_matse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINSYS_POINT_TO_LINE_MATSE3__
#define __OPENROX_LINSYS_POINT_TO_LINE_MATSE3__

#include <baseproc/geometry/line/line3d.h>
#include <baseproc/geometry/line/line2d.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Interactions
//! \addtogroup Interaction_MatSE3
//! @{

//! \brief Weighted Linear System A = L'*W^2*L and b = L'*W^2*e
//! \todo This function should be moved in file interaction_texture_matse3_light_affine_model3d
//! and named "rox_linsys_weighted_texture_matse3_light_affine_model3d"
ROX_API Rox_ErrorCode rox_linsys_weighted_point_to_line_matse3 (
   Rox_Matrix LtL, 
   Rox_Matrix Lte,
   const Rox_Line3D_Planes line3D, 
   const Rox_Line2D_Normal line2D, 
   const Rox_Double x, 
   const Rox_Double y,
   const Rox_Double e,
   const Rox_Double w
);

//! \brief Linear System A = L'*L and b = L'*e
ROX_API Rox_ErrorCode rox_linsys_point_to_line_matse3 (
   Rox_Matrix LtL, 
   Rox_Matrix Lte,
   const Rox_Line3D_Planes line3D, 
   const Rox_Line2D_Normal line2D, 
   const Rox_Double x, 
   const Rox_Double y,
   const Rox_Double e
);

//! @} 

#endif // __OPENROX_LINSYS_POINT_TO_LINE_MATSE3__
