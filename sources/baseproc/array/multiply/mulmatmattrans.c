//==============================================================================
//
//    OPENROX   : File mulmatmattrans.c
//
//    Contents  : Implementation of mulmatmattrans module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "mulmatmattrans.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_mulmatmattrans(Rox_Array2D_Double res, Rox_Array2D_Double one, Rox_Array2D_Double two)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !one || !two) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint wr = 0, hr = 0;
   error = rox_array2d_double_get_size(&hr, &wr, res); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint w1 = 0, h1 = 0;
   error = rox_array2d_double_get_size(&h1, &w1, one); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint w2 = 0, h2 = 0;
   error = rox_array2d_double_get_size(&h2, &w2, two); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (hr != h1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (wr != h2) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (w1 != w2) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dres = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dres, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** done = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &done, one );
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   Rox_Double ** dtwo = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dtwo, two );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < hr; i++)
   {
      for (Rox_Sint j = 0; j < wr; j++)
      {
         dres[i][j] = 0;

         for (Rox_Sint k = 0; k < w1; k++)
         {
            dres[i][j] += done[i][k] * dtwo[j][k];
         }
      }
   }

function_terminate:
   return error;
}