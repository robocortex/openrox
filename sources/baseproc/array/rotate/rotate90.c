//==============================================================================
//
//    OPENROX   : File rotate90.c
//
//    Contents  : Implementation of rotate90 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "rotate90.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uchar_rotate90 ( Rox_Array2D_Uchar res, Rox_Array2D_Uchar input )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint wr = 0, hr = 0;
   error = rox_array2d_uchar_get_size(&hr, &wr, res); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint wi = 0, hi = 0;
   error = rox_array2d_uchar_get_size(&hi, &wi, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (hr != wi) 
   {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   
   if (wr != hi) 
   {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Uchar **dres = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &dres, res);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar **dinput = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &dinput, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < hr; i++)
   {
      for (Rox_Sint j = 0; j < wr; j++)
      {
         dres[i][j] = dinput[hi - 1 - j][i];
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uint_rotate90(Rox_Array2D_Uint res, Rox_Array2D_Uint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint wr = 0, hr = 0;
   error = rox_array2d_uint_get_size ( &hr, &wr, res ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint wi = 0, hi = 0;
   error = rox_array2d_uint_get_size ( &hi, &wi, input ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (hr != wi) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (wr != hi) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint **dres   = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dres, res );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint **dinput = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dinput, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < hr; i++)
   {
      for (Rox_Sint j = 0; j < wr; j++)
      {
         dres[i][j] = dinput[hi - 1 - j][i];
      }
   }

function_terminate:
   return error;
}
