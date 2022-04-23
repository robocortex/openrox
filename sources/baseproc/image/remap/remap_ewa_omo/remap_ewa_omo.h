//============================================================================
//
//    OPENROX   : File remapewa_omo.h
//
//    Contents  : API of remapewa_omo module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_REMAPEWA_OMO__
#define __OPENROX_REMAPEWA_OMO__

#include <generated/array2d_uint.h>
#include <generated/array2d_float.h>
#include <generated/array2d_double.h>
#include <baseproc/image/image.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

//! \ingroup Image
//! \addtogroup Remap
//! @{

//! Given an input array, remap it using EWA ((Eelliptical Weighted Average) ) interpolation : result(i,j) = source(i+v,j+u)
//! \param  [out] output               The result array
//! \param  [out] output_mask_output   The result array validity mask
//! \param  [in]  input                The input array
//! \param  [in]  map                  The map containing the (u,v) shift. It's width is twice the width of the input, u,v being interleaved.
//! \return An error code
ROX_API Rox_ErrorCode rox_remap_ewa_omo_uchar (
   Rox_Image output, 
   Rox_Array2D_Uint output_mask_output, 
   Rox_Image input, 
   Rox_MeshGrid2D_Float map
);

//! @}

#endif
