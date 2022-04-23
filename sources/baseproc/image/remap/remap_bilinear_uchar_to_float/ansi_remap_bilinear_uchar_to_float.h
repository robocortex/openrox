//==============================================================================
//
//    OPENROX   : File ansi_remap_bilinear_uchar_to_float.h
//
//    Contents  : API of remap_bilinear_uchar_to_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_REMAP_BILINEAR_UCHAR_TO_FLOAT__
#define __OPENROX_ANSI_REMAP_BILINEAR_UCHAR_TO_FLOAT__

//#include <array2d_float.h>
//#include <array2d_double.h>
//#include <baseproc/geometry/pixelgrid/meshgrid2d.h>

//#include <baseproc/image/image.h>
//#include <baseproc/image/imask/imask.h>

//! \ingroup Image
//! \addtogroup Remap
//! @{

//! Given an input array, remap it using bilinear interpolation : result(i,j) = source(i+v,j+u)
//! \param  [out]  output         The result array
//! \param  [out]  mask_out       The result array validity mask
//! \param  [in ]  mask_out_ini   The input validity mask in destination space
//! \param  [in ]  mask_inp       The input validity mask in source space
//! \param  [in ]  input          The input array
//! \param  [in ]  map            The map containing the (u,v) shift. It's width is twice the width of the input, u,v being interleaved.
//! \return An error code
//! \todo   To be tested

int rox_ansi_remap_bilinear_uchar_to_float (
   float ** image_out_data,
   unsigned int ** imask_out_data,
   unsigned int ** imask_out_ini_data,
   int rows_out,
   int cols_out,
   unsigned char ** image_inp_data,
   unsigned int ** inp_mask_data,
   int rows_inp,
   int cols_inp,
   float ** grid_u_data,
   float ** grid_v_data
);


//! @} 

#endif
