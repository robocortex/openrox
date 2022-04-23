//============================================================================
//
//    OPENROX   : File ansi_basegradient.c
//
//    Contents  : Implementation of basegradient module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "ansi_basegradient.h"

int rox_ansi_array2d_float_basegradient (
   float ** Iu_data,
   float ** Iv_data,
   unsigned int ** Gm_data,
   float ** I_data,
   unsigned int ** Im_data,
   int rows,
   int cols
)
{
   int error = 0;
   int i = 0, j =0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) privatei, j)
#endif

   for ( i = 0; i < rows; i++ )
   {
      int ni = i + 1;
      int pi = i - 1;

      for ( j = 0; j < cols; j++ )
      {
         Gm_data[i][j] = 0;

         if ( (i == 0) || (i == rows - 1) ) continue;
         if ( (j == 0) || (j == cols - 1) ) continue;

         int nj = j + 1;
         int pj = j - 1;

         if ( Im_data[i][j] && Im_data[pi][j] && Im_data[ni][j] && Im_data[i][pj] && Im_data[i][nj] )
         {
            Gm_data[i][j] = ~0;
            Iu_data[i][j] = 0.5f * ( I_data[i][nj] - I_data[i][pj] );
            Iv_data[i][j] = 0.5f * ( I_data[ni][j] - I_data[pi][j] );
         }
      }
   }

   return error;
}

int rox_ansi_array2d_float_basegradient_nomask (
   float ** Iu_data,
   float ** Iv_data,
   float ** I_data,
   int rows,
   int cols
)
{
   int error = 0;
   int i = 0, j = 0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) privatei, j)
#endif

   for (i = 0; i < rows; i++)
   {
      int ni = i + 1;
      int pi = i - 1;

      for (j = 0; j < cols; j++)
      {
         if (i == 0 || i == rows - 1) continue;
         if (j == 0 || j == cols - 1) continue;

         int nj = j + 1;
         int pj = j - 1;

         Iu_data[i][j] = 0.5f*(I_data[i][nj] - I_data[i][pj]);
         Iv_data[i][j] = 0.5f*(I_data[ni][j] - I_data[pi][j]);
      }
   }

   return error;
}
