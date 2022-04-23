//==============================================================================
//
//    OPENROX   : File remap_bilinear_nomask.h
//
//    Contents  : API of remap_bilinear_nomask module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_REMAP_BILINEAR_NOMASK_FLOAT_TO_FLOAT__
#define __OPENROX_REMAP_BILINEAR_NOMASK_FLOAT_TO_FLOAT__

#include <generated/array2d_uint.h>
#include <generated/array2d_float.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>
#include <generated/array2d_point2d_sshort.h>
#include <baseproc/image/image.h>

//! \ingroup Image
//! \addtogroup Remap
//! @{

//! Given an input array, remap it using bilinear interpolation : result(i,j) = source(i+v,j+u)
//! \param  [out]  image_out          The output array
//! \param  [in ]  image_inp          The input array
//! \param  [in ]  grid               The grid containing the (u,v) shift.
//! \return An error code
ROX_API Rox_ErrorCode rox_remap_bilinear_nomask_float_to_float (
   Rox_Array2D_Float output, 
   const Rox_Array2D_Float image_inp, 
   const Rox_MeshGrid2D_Float grid
);

//! @} 

#endif
