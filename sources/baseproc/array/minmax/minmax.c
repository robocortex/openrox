//==============================================================================
//
//    OPENROX   : File minmax.c
//
//    Contents  : Implementation of minmax module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "minmax.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_minmax(Rox_Double *min, Rox_Double *max, Rox_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input || !min || !max) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *min = 0.0;
   *max = 0.0;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dd, input ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double valmin = dd[0][0];
   Rox_Double valmax = dd[0][0];

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
          if (dd[i][j] < valmin) valmin = dd[i][j];
          if (dd[i][j] > valmax) valmax = dd[i][j];
      }
   }

    *min = valmin;
    *max = valmax;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_minmax(Rox_Float *min, Rox_Float *max, Rox_Array2D_Float input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input || !min || !max) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *min = 0.0;
   *max = 0.0;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dd = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dd, input ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float valmin = dd[0][0];
   Rox_Float valmax = dd[0][0];

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         if (dd[i][j] < valmin) valmin = dd[i][j];
         if (dd[i][j] > valmax) valmax = dd[i][j];
      }
   }

   *min = valmin;
   *max = valmax;

function_terminate:
   return error;
}

