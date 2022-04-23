//==============================================================================
//
//    OPENROX   : File svd.c
//
//    Contents  : Implementation of svd module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "svd.h"
#include <baseproc/maths/maths_macros.h>

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/decomposition/housebidiag.h>
#include <inout/system/errors_print.h>

#define ROX_MIN(A,B) (((A)<(B))?(A):(B))
#define ROX_MAX_ITER_SVD 75

Rox_ErrorCode rox_array2d_double_svd(Rox_Array2D_Double U, Rox_Array2D_Double S, Rox_Array2D_Double V, const Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint uw = 0, uh = 0, sw = 0, sh = 0, vw = 0, vh = 0, iw = 0, ih = 0;
   Rox_Double **dU = NULL, **dV = NULL, **dS = NULL, **dM = NULL, **dE = NULL;
   Rox_Double eps = 0.0;
   Rox_Array2D_Double E = NULL;

   Rox_Sint k, l = 0;
   Rox_Sint l1, retval;
   Rox_Double c, f, g, h, scale, x, y, z;
   Rox_Uint flag;

   if (!U || !V || !S || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_size(&uh, &uw, U);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&vh, &vw, V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&sh, &sw, S);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&ih, &iw, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We don't compute the sdv for matrices with more cols than rows
   if (ih < iw)  { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // The V matrix must have size (ih x ih)
   if (uh != ih) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (uw != ih) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // The S matrix must have size (ih x 1)
   if (sw != 1)  { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (sh != ih) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // The V matrix must have size (iw x iw)
   if (vh != iw) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (vw != iw) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer( &dU, U);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dV, V);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_get_data_pointer_to_pointer( &dS, S);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_get_data_pointer_to_pointer( &dM, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy input in a possibly bigger U
   error = rox_array2d_double_fillval(U, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   for (Rox_Sint i = 0; i < ih; i++)
   {
      for (Rox_Sint j = 0; j < iw; j++)
      {
         dU[i][j] = dM[i][j];
      }
   }

   error = rox_array2d_double_fillunit(V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&E, iw, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dE, E);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_householder_bidiagonalization(U, S, V, E, &eps);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   retval = 0;
   //  Diagonalization of the bidiagonal form 
   for (k = iw - 1 ; k >= 0; k--)
   {
      for (Rox_Sint iter = 1; iter <= ROX_MAX_ITER_SVD; iter++)
      {
         flag = 1;
         for (l = k; l >= 1; l--)
         {
            if (fabs(dE[l][0]) <= eps)
            {
               flag = 0;
               break;
            }
            if (fabs(dS[l - 1][0]) <= eps)
            {
               break; // go to cancellation;
            }
         } //  end l 

         if (flag && l > 0)
         {
            //  cancellation of e_data[l] if l > 0 
            c = 0.0;
            scale = 1.0;

            l1 = l - 1;
            for (Rox_Sint i = l; i <= k; i++)
            {
               f = scale * dE[i][0];
               dE[i][0] *= c;

               if (fabs(f) <= eps)
               {
                  break; // goto test_f_convergence;
               }

               g = dS[i][0];
               h = dS[i][0] = sqrt(f * f + g * g);
               c = g / h;
               scale = -f / h;
               for (Rox_Sint j = 0; j < ih; j++)
               {
                  y = dU[j][l1];
                  z = dU[j][i];
                  dU[j][l1] = y * c + z * scale;
                  dU[j][i] = -y * scale + z * c;
               } //  end for j 
            } //  end for i 
         } //  end if 

         z = dS[k][0];
         if (l == k)
         {
            if (z < 0.0)
            {
               //  s_data[k] is made non-negative 
               dS[k][0] = - z;
               for (Rox_Sint j = 0; j < iw; j++)
               {
                  dV[j][k] = -dV[j][k];
               }
            } //  end if z 
            break;
         }

         //  shift from bottom 2x2 minor 
         if (iter == ROX_MAX_ITER_SVD)
         {
            retval = k;
            break;
         }

         x = dS[l][0];
         y = dS[k - 1][0];
         g = dE[k - 1][0];
         h = dE[k][0];
         f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
         g = sqrt(f * f + 1.0);
         f = ((x - z) * (x + z) + h * (y / ((f < 0) ? (f - g) : (f + g)) - h)) / x;
         //  next QR transformation 
         c = scale = 1.0;
         for (Rox_Sint i = l + 1; i <= k; i++)
         {
            g = dE[i][0];
            y = dS[i][0];
            h = scale * g;
            g *= c;
            dE[i - 1][0] = z = sqrt(f * f + h * h);
            if((z > 0.0) || (z<0.0))
            {
               c = f / z;
               scale = h / z;
            }
            else
            {
               c = scale = sqrt(0.5);
            }
            f = x * c + g * scale;
            g = -x * scale + g * c;
            h = y * scale;
            y *= c;
            for (Rox_Sint j = 0; j < iw; j++)
            {
               x = dV[j][i - 1];
               z = dV[j][i];
               dV[j][i - 1] = x * c + z * scale;
               dV[j][i] = -x * scale + z * c;
            } //  end j 
            z = sqrt(f * f + h * h);
            dS[i - 1][0] = z;
            if((z > 0.0) || (z<0.0))
            {
               c = f / z;
               scale = h / z;
            }
            else
            {
               c = scale = sqrt(0.5);
            }
            f = c * g + scale * y;
            x = -scale * g + c * y;
            for (Rox_Sint j = 0; j < ih; j++)
            {
               y = dU[j][i - 1];
               z = dU[j][i];
               dU[j][i - 1] = y * c + z * scale;
               dU[j][i] = -y * scale + z * c;
            } //  end j 
         } //  end i 
         dE[l][0] = 0.0;
         dE[k][0] = f;
         dS[k][0] = x;
      } // end for iter 
   } //  end for k 

   if (retval) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   rox_array2d_double_del(&E);

   return error;
}
