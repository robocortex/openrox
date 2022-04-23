//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matse3_zi.c
//
//    Contents  : Implementation of warp_grid_matsl3_zi module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_warp_grid_matse3_zi.h"
#include <float.h>

int rox_ansi_warp_grid_float_matse3_zi_float (
   float ** grid_u_data, 
   float ** grid_v_data,
   unsigned int ** grid_mask_data,
   float ** Zir_data,
   int rows,
   int cols,
   double ** cQr_data
)
{
   int error = 0;

   for ( int v = 0; v < rows; v++)
   {
      float vf = (float) v;
      
      for ( int u = 0; u < cols; u++)
      {
         grid_mask_data[v][u] = 0;

         grid_u_data[v][u] = 0.0f;
         grid_v_data[v][u] = 0.0f;

         float Zir = Zir_data[v][u];

         if (Zir < DBL_EPSILON) continue;

         float uf = (float) u;
         
         float pu = cQr_data[0][0] * uf + cQr_data[0][1] * vf + cQr_data[0][2] + cQr_data[0][3] * Zir;
         float pv = cQr_data[1][0] * uf + cQr_data[1][1] * vf + cQr_data[1][2] + cQr_data[1][3] * Zir;
         float pw = cQr_data[2][0] * uf + cQr_data[2][1] * vf + cQr_data[2][2] + cQr_data[2][3] * Zir;
         
         if (pw < DBL_EPSILON) continue;
         
         float pwi = 1.0f / pw;
         
         grid_u_data[v][u] = pu * pwi;
         grid_v_data[v][u] = pv * pwi;

         grid_mask_data[v][u] = ~0;
      }
   }

   return error;
}