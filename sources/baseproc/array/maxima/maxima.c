//==============================================================================
//
//    OPENROX   : File maxima.c
//
//    Contents  : Implementation of maxima module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "maxima.h"

#include <generated/dynvec_double_struct.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#include <math.h>

Rox_ErrorCode rox_array2d_double_local_maxima (
  Rox_DynVec_Double local_maxima_values, 
  Rox_DynVec_Double local_maxima_coords, 
  Rox_DynVec_Point2D_Uint local_maxima_indexes, 
  Rox_Array2D_Double yv, Rox_Double y_thresh
)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // The input signal y must be a row-vector (i.e. col = 1)
   Rox_Double * y = NULL;

   error = rox_array2d_double_get_data_pointer ( &y, yv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Init values to empty
   //local_maxima = []; 
   //local_maxima_indexes = [];
   //local_maxima_coordinates = [];

   Rox_Double noise_thresh = 0.0;

   // Get the number of elements
   Rox_Sint n = 0;
   error = rox_array2d_double_get_cols(&n, yv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Always work on row-vectors with min zize = 3
   if (n < 3)
   { error = ROX_ERROR_INSUFFICIENT_DATA; goto function_terminate; }

   Rox_Sint shift_next = 0;

   // Init k to second element
   Rox_Sint k = 1;
   
   while (k < (n-1)) // The extrema will never be assigned to cent
   {
     Rox_Double prev = y[k-1];
     Rox_Double cent = y[ k ];
     Rox_Double next = y[k+1];
     
     // be sure that next is different from cent (up to noise)
     shift_next = 1;
     Rox_Double cent_tmp = cent;

     while ((fabs(cent_tmp - next) <= noise_thresh))
     {
        shift_next = shift_next+1;
        if (k+shift_next <= n)
        {
           // change cent_tmp to bo sure that big gaps (above threshold) will not occur
           cent_tmp = next; 
           next = y[k+shift_next];
        }
        else
        {
           // stop since we are at the end of the array 
           break;
        }
      }
     
     if ((cent > prev) && (cent > next) && (cent > y_thresh))
     {
        Rox_Double local_maxima_value = -1.0;
        Rox_Double local_maxima_coord = -1.0;

        local_maxima_value = cent;
        local_maxima_coord = (double)k+((double)shift_next-1.0)/2.0; // optimal coordinates are at the center of the peak

        error = rox_dynvec_double_append(local_maxima_values, &local_maxima_value);
        ROX_ERROR_CHECK_TERMINATE ( error );

        error = rox_dynvec_double_append(local_maxima_coords, &local_maxima_coord);
        ROX_ERROR_CHECK_TERMINATE ( error );

        Rox_Point2D_Uint_Struct indexes;

        indexes.u = k;
        indexes.v = k+shift_next-1;

        error = rox_dynvec_point2d_uint_append(local_maxima_indexes, &indexes);
        ROX_ERROR_CHECK_TERMINATE ( error );

      }
     
     k = k + shift_next ;
   }

function_terminate:
   return error;
}
