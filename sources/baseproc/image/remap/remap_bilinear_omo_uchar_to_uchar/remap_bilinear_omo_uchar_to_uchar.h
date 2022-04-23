//==============================================================================
//
//    OPENROX   : File remap_bilinear_omo_uchar_to_uchar.h
//
//    Contents  : API of remap_bilinear_omo_uchar_to_uchar module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_REMAP_BILINEAR_OMO_UCHAR_TO_UCHAR__
#define __OPENROX_REMAP_BILINEAR_OMO_UCHAR_TO_UCHAR__

#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

#include <baseproc/image/image.h>
#include <baseproc/image/imask/imask.h>

//! \ingroup Image
//! \addtogroup RemapBilinearOmo
//! @{

//! Given an input array, remap it using bilinear interpolation : result(i,j) = source(i+v,j+u)
//! \param  [out]  output               the result array
//! \param  [out]  mask_output   the result array validity mask
//! \param  [in ]  input                the input array
//! \param  [in ]  map                  the map containing the (u,v) shift.
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_remap_bilinear_omo_uchar_to_uchar (
   Rox_Image output, 
   Rox_Imask mask_output, 
   const Rox_Image input, 
   const Rox_MeshGrid2D_Float grid
);

//! @} 

#endif
