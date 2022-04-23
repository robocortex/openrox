//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matse3_z.c
//
//    Contents  : Implementation of warp_grid_matsl3_z module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_warp_grid_matse3_z.h"
#include <float.h>

int rox_ansi_warp_grid_float_matse3_z_float (
   float ** grid_u_data, 
   float ** grid_v_data,
   unsigned int ** grid_mask_data,
   float ** Zr_data,
   int rows,
   int cols,
   double ** cQr_data
)
{
   int error = 0;
   for ( int v = 0; v < rows; v++)
   {      
      for ( int u = 0; u < cols; u++)
      {
         float vZr = (float) v * Zr_data[v][u];

         grid_mask_data[v][u] = 0;

         grid_u_data[v][u] = 0.0f;
         grid_v_data[v][u] = 0.0f;

         float Zr = Zr_data[v][u];

         if (Zr < DBL_EPSILON) continue;

         float uZr = (float) u * Zr;
                  
         float pu = cQr_data[0][0] * uZr + cQr_data[0][1] * vZr + cQr_data[0][2] * Zr + cQr_data[0][3];
         float pv = cQr_data[1][0] * uZr + cQr_data[1][1] * vZr + cQr_data[1][2] * Zr + cQr_data[1][3];
         float pw = cQr_data[2][0] * uZr + cQr_data[2][1] * vZr + cQr_data[2][2] * Zr + cQr_data[2][3];
         
         if (pw < DBL_EPSILON) continue;
         
         float pwi = 1.0f / pw;
         
         grid_u_data[v][u] = pu * pwi;
         grid_v_data[v][u] = pv * pwi;

         grid_mask_data[v][u] = ~0;
      }
   }

   return error;
}
