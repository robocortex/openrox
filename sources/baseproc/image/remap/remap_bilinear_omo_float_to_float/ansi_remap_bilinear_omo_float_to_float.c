//==============================================================================
//
//    OPENROX   : File ansi_remap_bilinear_omo_float_to_float.c
//
//    Contents  : Implementation of remap_bilinear_omo_float_to_float module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_remap_bilinear_omo_float_to_float.h"

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
)
{
   int error = 0;

   for (int i = 0; i < rows_out; i++)
   {
      for (int j = 0; j < cols_out; j++)
      {    
         float I00 = 0.0f;
         float I01 = 0.0f;
         float I10 = 0.0f;
         float I11 = 0.0f;

         // Get the float coordinates
         float uf = grid_u_data[i][j];
         float vf = grid_v_data[i][j];

         image_out_data[i][j] = 0.0f;
         imask_out_data[i][j] = 0;
         
         // Get the integer coordinates
         int ui = (int) uf;
         int vi = (int) vf;

         if ((ui < 0) || (vi < 0)) continue;
         if ((ui > cols_inp - 1) || (vi > rows_inp - 1)) continue;

         // Compute the residuals
         float du = uf - ui;
         float dv = vf - vi;

#ifndef EXTRAPOLATION_ON_BORDERS
         if ((( uf < 0.0 ) && ( vf < 0.0 )) || (( uf >= cols_inp-1 ) && ( vf >= rows_inp-1 )) || (( uf >= cols_inp-1 ) && ( vf < 0 )) || (( uf < 0 ) && ( vf >= rows_inp-1 )))
         {
            image_out_data[i][j] = image_inp_data[vi][ui];
            imask_out_data[i][j] = ~0;
            continue;
         }
         
         if ((( uf < 0.0 ) && ( vf >= 0.0 )) || (( uf >= cols_inp-1 ) && ( vf < rows_inp-1 )))
         {
            I00 = image_inp_data[vi][ui];
            I10 = image_inp_data[vi+1][ui];
            image_out_data[i][j] = I00 * (1 - dv) + dv * I10;
            imask_out_data[i][j] = ~0;
            continue;
         }

         if ((( uf >= 0.0 ) && ( vf < 0.0 )) || (( uf < cols_inp-1 ) && ( vf >= rows_inp-1 )))
         {
            I00 = image_inp_data[vi][ui];
            I01 = image_inp_data[vi][ui+1];
            image_out_data[i][j] = I00 * (1 - du) + du * I01;
            imask_out_data[i][j] = ~0;
            continue;
         }
         
         I00 = image_inp_data[vi][ui];
         I01 = image_inp_data[vi][ui+1];
         I10 = image_inp_data[vi+1][ui];
         I11 = image_inp_data[vi+1][ui+1];
#else
         I00 = image_inp_data[vi][ui];
         I01 = 0.0f;
         I10 = 0.0f;
         I11 = 0.0f;

         // Added test for borders special case
         if ( ui != cols_inp - 1 ) 
         {
            I01 = image_inp_data[vi][ui + 1];
         }

         if ( vi != rows_inp - 1 ) 
         {
            I10 = image_inp_data[vi + 1][ui];
         }

         if ((ui != cols_inp - 1) && (vi != rows_inp - 1))
         {
            I11 = image_inp_data[vi + 1][ui + 1];
         }
#endif
         // Bilinear interpolation
         float b1 = I00;
         float b2 = I01 - b1;
         float b3 = I10 - b1;
         float b4 = b1 + I11 - I10 - I01;

         image_out_data[i][j] = b1 + b2 * du + b3 * dv + b4 * du * dv;
         imask_out_data[i][j] = ~0;
      }
   }

   return error;
}
