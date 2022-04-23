//==============================================================================
//
//    OPENROX   : File ansi_warp_grid_matsl3_fixed12_4.c
//
//    Contents  : Implementation of warp_grid_matsl3_fixed12_4 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ansi_warp_grid_matsl3_fixed12_4.h"

#include <math.h>
#include <float.h>

int rox_ansi_warp_grid_sl3_fixed12_4 (
   short ** grid_u_data, 
   short ** grid_v_data,
   int rows,
   int cols,
   double ** H_data
)
{
   int error = 0;

   for ( int i = 0; i < rows; i++)
   {
      double v = (double) i;
      double ru = H_data[0][1] * v + H_data[0][2];
      double rv = H_data[1][1] * v + H_data[1][2];
      double rw = H_data[2][1] * v + H_data[2][2];

      for ( int j = 0; j < cols; j++)
      {
         double u = (double) j;

         double nu = H_data[0][0] * u + ru;
         double nv = H_data[1][0] * u + rv;
         double nw = H_data[2][0] * u + rw;

         if (fabs(nw) > DBL_EPSILON)
         {
            double iw = 1.0 / nw;
            nu = nu * iw;
            nv = nv * iw;
         }

         short snu = (short) (16.0 * nu);
         short snv = (short) (16.0 * nv);

         if (snu < 0) snu = 0;
         if (snv < 0) snv = 0;

         grid_u_data[i][j] = snu;
         grid_v_data[i][j] = snv;
      }
   }

   return error;
}