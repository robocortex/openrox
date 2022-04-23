//============================================================================
//
//    OPENROX   : File ansi_array2d_float_symmetric_separable_convolve.h
//
//    Contents  : API of array2d_float_symmetric_separable_convolve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_ANSI_ARRAY2D_FLOAT_SYMMETRIC_SEPARABLE_CONVOLVE__
#define __OPENROX_ANSI_ARRAY2D_FLOAT_SYMMETRIC_SEPARABLE_CONVOLVE__

int rox_ansi_array2d_float_symmetric_seperable_convolve ( 
   float ** output_data,
   float ** input_data,
   int rows,
   int cols,
   float ** kernel_data,
   int hksize,
   float ** buffer_data,
   float ** imborder_data
);

#endif
