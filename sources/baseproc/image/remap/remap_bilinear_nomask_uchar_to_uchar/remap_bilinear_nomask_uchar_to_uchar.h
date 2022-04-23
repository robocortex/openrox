//==============================================================================
//
//    OPENROX   : File remap_bilinear_nomask_uchar_to_uchar.h
//
//    Contents  : API of remap_bilinear_nomask_uchar_to_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_REMAP_BILINEAR_NOMASK_UCHAR_TO_UCHAR__
#define __OPENROX_REMAP_BILINEAR_NOMASK_UCHAR_TO_UCHAR__

#include <generated/array2d_uint.h>
#include <generated/array2d_float.h>
#include <baseproc/geometry/pixelgrid/meshgrid2d.h>
#include <generated/array2d_point2d_sshort.h>
#include <baseproc/image/image.h>

//! \ingroup Image
//! \addtogroup Remap
//! @{

//! Given an input array, remap it using bilinear interpolation : result(i,j) = source(i+v,j+u)
//! \param  [out]  output         The result array
//! \param  [in ]  input          The input array
//! \param  [in ]  map            The map containing the (u,v) shift. It's width is twice the width of the input, u,v being interleaved.
//! \return An error code
ROX_API Rox_ErrorCode rox_remap_bilinear_nomask_uchar_to_uchar (
   Rox_Image output, 
   const Rox_Image input, 
   const Rox_MeshGrid2D_Float grid
);

//! Given an input array, remap it using bilinear interpolation : result(i,j) = source(i+v,j+u).
//! \param  [out]  output         The result array (Must be an image with dimensions < 2048)
//! \param  [in ]  input          The input array (Must be an image with dimensions < 2048)
//! \param  [in ]  map            The map containing the (u,v) shift. It's width is twice the width of the input, u,v being interleaved.
//! \return An error code
ROX_API Rox_ErrorCode rox_remap_bilinear_nomask_uchar_to_uchar_fixed (
   Rox_Image output, 
   const Rox_Image input, 
   const Rox_Array2D_Point2D_Sshort map
);

//! @} 

#endif
