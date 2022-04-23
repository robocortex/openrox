//==============================================================================
//
//    OPENROX   : File warp_grid_matse3_z.h
//
//    Contents  : API of warp_grid_matse3_z module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_WARP_GRID_MATSE3_Z__
#define __OPENROX_WARP_GRID_MATSE3_Z__

#include <generated/array2d_float.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matut3.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Geometry
//! \addtogroup Warp
//! @{

//! Compute the remap parameters for a given pose cTr
//! [pu; pv] = projection ( Zr * Kc * cRr * inv(Kr) * [i j 1] + Kc * ctr )
//! grid[i][j*2] = pu, grid[i][j*2+1] = pv 
//! \param  [out]  grid_warp      The result warped grid
//! \param  [out]  grid_mask      The result grid mask
//! \param  [in ]  Zr             The depth map in the reference frame
//! \param  [in ]  cTr            The pose used to transform pixels (T = [R, t; 0 1])
//! \param  [in ]  Kr             Calibration of the reference image 
//! \param  [in ]  Kc             Calibration of the current image
//! \return An error code
ROX_API Rox_ErrorCode rox_warp_grid_float_matse3_z_float (
   Rox_MeshGrid2D_Float grid_warp, 
   Rox_Imask grid_mask, 
   const Rox_Array2D_Float depth, 
   const Rox_MatSE3 cTr, 
   const Rox_MatUT3 Kr, 
   const Rox_MatUT3 Kc
);

//! @} 

#endif
