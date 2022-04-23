//==============================================================================
//
//    OPENROX   : File housebidiag.c
//
//    Contents  : Implementation of housebidiag module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "housebidiag.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

#define ROX_SIGN_ABS(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

Rox_ErrorCode rox_array2d_householder_bidiagonalization(Rox_Array2D_Double U, Rox_Array2D_Double S, Rox_Array2D_Double V, Rox_Array2D_Double e, Rox_Double * ptr_eps)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint uh, sw, vw, ew;
   Rox_Double **dU = NULL, **dV = NULL, **dS = NULL, **de = NULL;
   Rox_Double eps = 1e-16;
   Rox_Sint i = 0, j = 0, k = 0, l = 0;
   Rox_Double f, h, s;
   Rox_Double x = 0.0;
   Rox_Double g = 0.0;
   Rox_Double scale = 0.0;
   Rox_Sint m, n;

   if (!U || !V || !S || !e || !ptr_eps) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_rows(&uh, U); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_cols(&vw, V); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_cols(&sw, S); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_cols(&ew, e); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (ew != 1) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (sw != 1) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer(&dU, U);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer(&dV, V);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer(&dS, S);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer(&de, e);
   ROX_ERROR_CHECK_TERMINATE ( error );

   m = uh;
   n = vw;

   for (i = 1; i <= n; i++)
   {
      l = i + 1;
      de[i - 1][0] = scale * g;

      g = s = scale = 0.0;
      if (i <= m)
      {
         for (k = i; k <= m; k++)
         {
            scale += fabs(dU[k - 1][i - 1]);
         }

         if (scale > 0.0)
         {
            for (k = i; k <= m; k++)
            {
               dU[k - 1][i - 1] /= scale;
               s += dU[k - 1][i - 1] * dU[k - 1][i - 1];
            }

            f = dU[i - 1][i - 1];
            g = -ROX_SIGN_ABS(sqrt(s), f);
            h = f * g - s;
            dU[i - 1][i - 1] = f - g;

            if (i != n)
            {
               for (j = l; j <= n; j++)
               {
                  for (s = 0.0, k = i; k <= m; k++)
                  {
                     s += dU[k - 1][i - 1] * dU[k - 1][j - 1];
                  }

                  f = s / h;

                  for (k = i; k <= m; k++)
                  {
                     dU[k - 1][j - 1] += f * dU[k - 1][i - 1];
                  }
               }
            }

            for (k = i; k <= m; k++)
            {
               dU[k - 1][i - 1] *= scale;
            }
         }
      }

      dS[i - 1][0] = scale * g;

      g = s = scale = 0.0;

      if (i <= m && i != n)
      {
         for (k = l; k <= n; k++)
         {
            scale += fabs(dU[i - 1][k - 1]);
         }

         if (scale > 0.0)
         {
            for (k = l; k <= n; k++)
            {
               dU[i - 1][k - 1] /= scale;
               s += dU[i - 1][k - 1] * dU[i - 1][k - 1];
            }

            f = dU[i - 1][l - 1];
            g = -ROX_SIGN_ABS(sqrt(s), f);
            h = f * g - s;
            dU[i - 1][l - 1] = f - g;

            if (fabs(h) > 0.0)
            {
               for (k = l; k <= n; k++)
               {
                  de[k - 1][0] = dU[i - 1][k - 1] / h;
               }
            }

            if (i != m)
            {
               for (j = l; j <= m; j++)
               {
                  for (s = 0.0, k = l; k <= n; k++)
                  {
                     s += dU[j - 1][k - 1] * dU[i - 1][k - 1];
                  }

                  for (k = l; k <= n; k++)
                  {
                     dU[j - 1][k - 1] += s * de[k - 1][0];
                  }
               }
            }

            for (k = l; k <= n; k++)
            {
               dU[i - 1][k - 1] *= scale;
            }
         }
      }

      x = ROX_MAX(x, (fabs(dS[i - 1][0]) + fabs(de[i - 1][0])));
   }

   for (i = n; i >= 1; i--)
   {
      if (i < n)
      {
         if ((g > 0.0) || (g < 0.0))
         {
            for (j = l; j <= n; j++)
            {
               dV[j - 1][i - 1] = (dU[i - 1][j - 1] / dU[i - 1][l - 1]) / g;
            }

            for (j = l; j <= n; j++)
            {
               for (s = 0.0, k = l; k <= n; k++)
               {
                  s += dU[i - 1][k - 1] * dV[k - 1][j - 1];
               }

               for (k = l; k <= n; k++)
               {
                  dV[k - 1][j - 1] += s * dV[k - 1][i - 1];
               }
            }
         }

         for (j = l; j <= n; j++)
         {
            dV[i - 1][j - 1] = dV[j - 1][i - 1] = 0.0;
         }
      }

      dV[i - 1][i - 1] = 1.0;
      g = de[i - 1][0];
      l = i;
   }

   for (i = n; i >= 1; i--)
   {
      l = i + 1;
      g = dS[i - 1][0];

      if (i < n)
      {
         for (j = l; j <= n; j++) dU[i - 1][j - 1] = 0.0;
      }

      if ((g > 0.0) || (g < 0.0))
      {
         g = 1.0 / g;
         if (i != n)
         {
            for (j = l; j <= n; j++)
            {
               for (s = 0.0, k = l; k <= m; k++)
               {
                  s += dU[k - 1][i - 1] * dU[k - 1][j - 1];
               }

               f = (s / dU[i - 1][i - 1]) * g;

               for (k = i; k <= m; k++)
               {
                  dU[k - 1][j - 1] += f * dU[k - 1][i - 1];
               }
            }
         }

         for (j = i; j <= m; j++)
         {
            dU[j - 1][i - 1] *= g;
         }
      }
      else
      {
         for (j = i; j <= m; j++)
         {
            dU[j - 1][i - 1] = 0.0;
         }
      }

      ++dU[i - 1][i - 1];
   }

   *ptr_eps = eps * x;

function_terminate:
   return error;
}