//==============================================================================
//
//    OPENROX   : File qr.c
//
//    Contents  : Implementation of qr module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "qr.h"

#include <stdio.h>

#include <generated/array2d_uint.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_qr ( 
   Rox_Array2D_Double Q, 
   Rox_Array2D_Double R, 
   const Rox_Array2D_Double M
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint bdiagrows;

   Rox_Sint r, c;
   Rox_Double colnorm, pivot, scale, alpha, swap;

   Rox_Array2D_Double housevector = NULL, housebuffer = NULL;

   if (!Q || !R || !M) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, M); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint maxsize = ROX_MAX(cols, rows);
   Rox_Sint iter = ROX_MIN(rows - 1, cols);

   error = rox_array2d_double_check_size(Q, rows, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(R, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Zeroing pointers to make it safe to goto function_terminate
   housevector = 0;
   housebuffer = 0;

   // Initialize buffers
   error = rox_array2d_double_new(&housebuffer, maxsize, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&housevector, rows, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dQ = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dQ, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dR = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dR, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** d2hb = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&d2hb, housebuffer);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** d2hv = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&d2hv, housevector);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dhv = d2hv[0];
   Rox_Double * dhb = d2hb[0];

   error = rox_array2d_double_fillunit(Q);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(R, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint k = 0; k < iter; k++)
   {
      pivot = dR[k][k];
      bdiagrows = rows - k;

      // Current column norm
      colnorm = 0;
      for (r = k + 1; r < rows; r++)
      {
         colnorm += dR[r][k] * dR[r][k];
      }

      alpha = sqrt(colnorm + pivot * pivot);

      // colnorm must have the opposite sign of the diagonal element
      if (dR[k][k] > 0.0) alpha = -alpha;

      // Update pivot
      pivot = pivot + alpha;
      colnorm = sqrt(colnorm + pivot * pivot);
      if (colnorm < DBL_EPSILON) scale = 0.0;
      else scale = 1.0 / colnorm;

      // Store householder vector
      dhv[0] = pivot * scale;
      for (r = 1; r < bdiagrows; r++)
      {
         dhv[r] = dR[k + r][k] * scale;
      }

      // housebuffer = housevec'*R
      for (c = 0; c < cols - k; c++)
      {
         dhb[c] = 0;

         for (r = 0; r < rows - k; r++)
         {
            dhb[c] += dhv[r] * dR[k + r][k + c];
         }
      }

      // update = R - 2*housevec*housebuffer
      for (r = 0; r < rows - k; r++)
      {
         for (c = 0; c < cols - k; c++)
         {
            dR[k + r][k + c] -= 2.0 * dhv[r] * dhb[c];
         }
      }

      // Update Q
      // housebuffer = housevec'*Q
      for (c = 0; c < rows; c++)
      {
         dhb[c] = 0;

         for (r = 0; r < rows - k; r++)
         {
            dhb[c] += dhv[r] * dQ[k + r][c];
         }
      }

      for (r = 0; r < rows - k; r++)
      {
         for (c = 0; c < rows; c++)
         {
            dQ[k + r][c] -= 2.0 * dhv[r] * dhb[c];
         }
      }
   }

   for (r = 0; r < rows; r++)
   {
      for (c = 0; c <= r; c++)
      {
         swap = dQ[r][c];
         dQ[r][c] = dQ[c][r];
         dQ[c][r] = swap;
      }
   }

function_terminate:
   rox_array2d_double_del(&housebuffer);
   rox_array2d_double_del(&housevector);

   return error;
}

Rox_ErrorCode rox_array2d_double_qrp(Rox_Array2D_Double Q, Rox_Array2D_Double R, Rox_Array2D_Double P, const Rox_Array2D_Double M)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint bdiagrows, iter, iswap, maxsize;

   Rox_Sint k, r, c;
   Rox_Sint maxcol;
   Rox_Double colnorm, pivot, scale, alpha, swap, maxcolnorm;

   Rox_Array2D_Double housevector, housebuffer, colnorms;
   Rox_Array2D_Uint columnids;
   
   if (!Q || !R || !P || !M) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, M); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   iter = ROX_MIN(rows - 1, cols);
   maxsize = ROX_MAX(cols, rows);

   error = rox_array2d_double_check_size(Q, rows, rows); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(R, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(P, cols, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Zeroing pointers to make it safe to goto function_terminate
   housevector = 0;
   housebuffer = 0;
   columnids = 0;
   colnorms = 0;

   // Initialize buffers
   error = rox_array2d_double_new(&housebuffer, maxsize, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&housevector, rows, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&colnorms, cols, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_new(&columnids, cols, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dQ = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dQ, Q );
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double ** dR = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dR, R );
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double ** dP = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dP, P );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** d2hb = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &d2hb, housebuffer );
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double * dhb = d2hb[0];

   Rox_Double ** d2hv = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &d2hv, housevector );
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double * dhv = d2hv[0];

   Rox_Double ** d2cn = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &d2cn, colnorms );
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double * dcn = d2cn[0];

   Rox_Uint ** d2ci = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &d2ci, columnids );
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Uint * dci = d2ci[0];

   rox_array2d_double_fillval(P, 0);
   rox_array2d_double_fillunit(Q);
   rox_array2d_double_copy(R, M);

   for (k = 0; k < cols; k++)
   {
      dci[k] = k;
   }

   for (k = 0; k < iter; k++)
   {
      maxcolnorm = -1;
      maxcol = 0;

      for (c = k; c < cols; c++)
      {
         dcn[c] = 0;

         for (r = k; r < rows; r++)
         {
            dcn[c] += dR[r][c] * dR[r][c];
         }

         if (maxcolnorm < dcn[c])
         {
            maxcol = c;
            maxcolnorm = dcn[c];
         }
      }

      if (maxcol != k)
      {
         // swap columns
         for (r = 0; r < rows; r++)
         {
            iswap = dci[k];
            dci[k] = dci[maxcol];
            dci[maxcol] = iswap;

            swap = dR[r][k];
            dR[r][k] = dR[r][maxcol];
            dR[r][maxcol] = swap;
         }
      }

      pivot = dR[k][k];
      bdiagrows = rows - k;

      // Current column norm
      colnorm = 0;
      for (r = k + 1; r < rows; r++)
      {
         colnorm += dR[r][k] * dR[r][k];
      }

      alpha = sqrt(colnorm + pivot * pivot);

      // Update pivot
      if (pivot >= 0.0) alpha = -alpha;
      pivot = pivot + alpha;

      colnorm = sqrt(colnorm + pivot * pivot);
      if (colnorm < DBL_EPSILON) scale = 0.0;
      else scale = 1.0 / colnorm;

      // Store householder vector
      dhv[0] = pivot * scale;
      for (r = 1; r < bdiagrows; r++)
      {
         dhv[r] = dR[k + r][k] * scale;
      }

      // housebuffer = housevec'*R
      for (c = 0; c < cols - k; c++)
      {
         dhb[c] = 0;

         for (r = 0; r < rows - k; r++)
         {
            dhb[c] += dhv[r] * dR[k + r][k + c];
         }
      }

      // update = R - 2*housevec*housebuffer
      for (r = 0; r < rows - k; r++)
      {
         for (c = 0; c < cols - k; c++)
         {
            dR[k + r][k + c] -= 2.0 * dhv[r] * dhb[c];
         }
      }

      // Update Q
      // housebuffer = housevec'*Q
      for (c = 0; c < rows; c++)
      {
         dhb[c] = 0;

         for (r = 0; r < rows - k; r++)
         {
            dhb[c] += dhv[r] * dQ[k + r][c];
         }
      }

      for (r = 0; r < rows - k; r++)
      {
         for (c = 0; c < rows; c++)
         {
            dQ[k + r][c] -= 2.0 * dhv[r] * dhb[c];
         }
      }
   }

   for (r = 0; r < rows; r++)
   {
      for (c = 0; c <= r; c++)
      {
         swap = dQ[r][c];
         dQ[r][c] = dQ[c][r];
         dQ[c][r] = swap;
      }
   }

   // Fill permutation matrix
   for (c = 0; c < cols; c++)
   {
      dP[c][dci[c]] = 1;
   }
   
function_terminate:
   rox_array2d_double_del(&housebuffer);
   rox_array2d_double_del(&housevector);
   rox_array2d_double_del(&colnorms);
   rox_array2d_uint_del(&columnids);

   return error;
}
