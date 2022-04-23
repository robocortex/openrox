//==============================================================================
//
//    OPENROX   : File warp_grid_matsl3.h
//
//    Contents  : API of warp_grid_matsl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_WARP_GRID_SL3__
#define __OPENROX_WARP_GRID_SL3__

#include <generated/array2d_point2d_sshort.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

//! \ingroup Geometry
//! \addtogroup Warp
//! @{

//! Compute the remap parameters for a given homography (grid[i][j*2] = nx, grid[i][j*2+1] = ny <-- [nx,ny,1] = homography * [i j 1]'
//! \param  [out]  grid           The result map
//! \param  [in ]  homography     The projective homography
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_warp_grid_sl3_float (
   Rox_MeshGrid2D_Float grid, 
   const Rox_MatSL3 homography
);

//! @}

#endif
