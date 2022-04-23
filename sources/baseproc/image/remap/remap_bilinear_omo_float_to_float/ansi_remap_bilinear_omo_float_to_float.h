//==============================================================================
//
//    OPENROX   : File ansi_remap_bilinear_omo_float_to_float.h
//
//    Contents  : API of remap_bilinear_omo_float_to_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ANSI_REMAP_BILINEAR_OMO_FLOAT_TO_FLOAT__
#define __OPENROX_ANSI_REMAP_BILINEAR_OMO_FLOAT_TO_FLOAT__

//! \ingroup Image
//! \addtogroup RemapBilinearOmo
//! @{

//! Given an input array, remap it using bilinear interpolation : result(i,j) = source(i+v,j+u)
//! \param  [out]  output               the result array
//! \param  [out]  mask_output          the result array validity mask
//! \param  [in ]  input                the input array
//! \param  [in ]  map                  the map containing the (u,v) shift.
//! \return An error code
//! \todo   To be tested

int rox_ansi_remap_bilinear_omo_float_to_float (
   float ** image_out_data,
   unsigned int ** imask_out_data,
   int rows_out,
   int cols_out,
   float ** image_inp_data,
   int rows_inp,
   int cols_inp,
   float ** grid_u_data,
   float ** grid_v_data
);

//! @} 

#endif
