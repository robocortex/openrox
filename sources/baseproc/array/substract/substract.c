//==============================================================================
//
//    OPENROX   : File substract.c
//
//    Contents  : Implementation of substract module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "substract.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uchar_substract ( Rox_Array2D_Uchar res, const Rox_Array2D_Uchar one, const Rox_Array2D_Uchar two )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_uchar_get_size(&rows, &cols, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_check_size(one, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_check_size(two, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar **dres = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dres, res);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uchar **done = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &done, one);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uchar **dtwo = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dtwo, two);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         if (done[i][j] >= dtwo[i][j])
         {
            dres[i][j] = done[i][j] - dtwo[i][j];
         }
         else
         {
            dres[i][j] = dtwo[i][j] - done[i][j];
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_double_substract(Rox_Array2D_Double res, Rox_Array2D_Double one, Rox_Array2D_Double two)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Sint cols = 0, rows = 0;
   
   error = rox_array2d_double_get_size(&rows, &cols, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(one, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(two, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **dres = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dres, res);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double **done = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &done, one);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **dtwo = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dtwo, two);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         dres[i][j] = done[i][j] - dtwo[i][j];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_substract(Rox_Array2D_Float res, Rox_Array2D_Float one, Rox_Array2D_Float two)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Sint cols = 0, rows = 0;
   
   error = rox_array2d_float_get_size(&rows, &cols, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(one, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(two, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float **dres = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dres, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float **done = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &done, one);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float **dtwo = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dtwo, two);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         dres[i][j] = done[i][j] - dtwo[i][j];
      }
   }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_float_substract_uchar(Rox_Array2D_Float res, Rox_Array2D_Uchar one, Rox_Array2D_Uchar two)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!res || !one || !two) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_array2d_uchar_match_size(two, one); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_match_size((Rox_Array2D) res, (Rox_Array2D) two); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dres = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dres, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** done = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &done, one);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dtwo = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dtwo, two);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         dres[i][j] = (Rox_Float)done[i][j] - (Rox_Float)dtwo[i][j];
      }
   }
   
function_terminate:
   return error;
}
