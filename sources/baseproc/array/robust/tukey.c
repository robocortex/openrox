//==============================================================================
//
//    OPENROX   : File tukey.c
//
//    Contents  : Implementation of tukey module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tukey.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <baseproc/array/mad/mad.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_tukey_bounded ( Rox_Array2D_Double weights, Rox_Array2D_Double workbuffer1, Rox_Array2D_Double workbuffer2, Rox_Array2D_Double input, Rox_Double sigma_minimal, Rox_Double sigma_maximal )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double mad = 0.0;
   Rox_Sint nbi = 0; // number of inliers
   Rox_Sint nbo = 0; // number of outliers

   if (!weights || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(weights, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(workbuffer1, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(workbuffer2, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute MAD
   error = rox_array2d_double_mad ( &mad, workbuffer1, workbuffer2, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // get data
   Rox_Double * dad = NULL;
   error = rox_array2d_double_get_data_pointer ( &dad, input );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double * dw = NULL;
   error = rox_array2d_double_get_data_pointer ( &dw, weights );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute properties
   Rox_Double sigma = mad * 1.4826;
   if (sigma < sigma_minimal) sigma = sigma_minimal;
   if (sigma > sigma_maximal) sigma = sigma_maximal;

   Rox_Double thresh = 4.6851 * sigma;
   if (thresh < 1e-5) thresh = 1e-5;

   // fabien : be carefull of the range of values in input
   // One may need to scale the vector to make sure the MAD is not too small because of the vector range

   for ( Rox_Sint i = 0; i < rows; i++ )
   {
      Rox_Double val = fabs(dad[i]);
      if (val > thresh)
      {
         dw[i] = 0.0;
         nbo++;
      }
      else
      {
         Rox_Double valoverthresh = val / thresh;
         Rox_Double p = 1.0 - (valoverthresh * valoverthresh);
         dw[i] = p * p;
         nbi++;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_tukey ( Rox_Array2D_Double weights, Rox_Array2D_Double workbuffer1, Rox_Array2D_Double workbuffer2, Rox_Array2D_Double input )
{
   return rox_array2d_double_tukey_bounded ( weights, workbuffer1, workbuffer2, input, -DBL_MAX, DBL_MAX );
}
