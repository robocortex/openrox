//============================================================================
//
//    OPENROX   : File remap_bilinear_trans.c
//
//    Contents  : Implementation of remapbilinear_trans module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "remap_bilinear_trans.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_remap_bilinear_trans_uchar_to_float (
   Rox_Array2D_Float output, 
   Rox_Array2D_Uint output_mask_output, 
   const Rox_Image input, 
   const Rox_Float tx, 
   const Rox_Float ty
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !output || !output_mask_output )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   if ( !input )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, output); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint scols = 0, srows = 0;
   error = rox_array2d_uchar_get_size(&srows, &scols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(output_mask_output, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, input);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Float ** dd = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dd, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dm, output_mask_output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint itx = (Rox_Sint) tx;
   Rox_Sint ity = (Rox_Sint) ty;

   Rox_Float dx = tx - (Rox_Float) itx;
   Rox_Float dy = ty - (Rox_Float) ity;

   Rox_Float b1 = (Rox_Float)((1.0 - dx)*(1.0 - dy));
   Rox_Float b2 = (Rox_Float)(dx * (1.0 - dy));
   Rox_Float b3 = (Rox_Float)((1.0 - dx) * dy);
   Rox_Float b4 = dx * dy;

   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Sint ni = i + ity;

      for (Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Sint nj = j + itx;
         dm[i][j] = 0;

         if ( (ni < 0) || (ni > srows - 1) ) continue;
         if ( (nj < 0) || (nj > scols - 1) ) continue;

         // Added test for borders special case
         if ((nj == scols - 1) || (ni == srows - 1))
         {
            dm[i][j] = ~0;
            dd[i][j] = ds[ni][nj];
            continue;
         }

         Rox_Float v1 = (Rox_Float) ds[ni][nj];
         Rox_Float v2 = (Rox_Float) ds[ni][nj+1];
         Rox_Float v3 = (Rox_Float) ds[ni+1][nj];
         Rox_Float v4 = (Rox_Float) ds[ni+1][nj+1];

         Rox_Float val = b1 * v1 + b2 * v2 + b3 * v3 + b4 * v4;

         dm[i][j] = ~0;
         dd[i][j] = val;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_remap_bilinear_trans_uchar_to_uchar (
   Rox_Image output, 
   Rox_Array2D_Uint output_mask_output, 
   const Rox_Image input, 
   const Rox_Float tx, 
   const Rox_Float ty
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !output_mask_output || !input)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size(&rows, &cols, output); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint scols = 0, srows = 0;
   error = rox_array2d_uchar_get_size(&srows, &scols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(output_mask_output, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** dd = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dd, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint  ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dm, output_mask_output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint itx = (Rox_Sint)tx;
   Rox_Sint ity = (Rox_Sint)ty;

   Rox_Float dx = tx - (Rox_Float) itx;
   Rox_Float dy = ty - (Rox_Float) ity;

   Rox_Float b1 = (Rox_Float)((1.0 - dx)*(1.0 - dy));
   Rox_Float b2 = (Rox_Float)(dx * (1.0 - dy));
   Rox_Float b3 = (Rox_Float)((1.0 - dx) * dy);
   Rox_Float b4 = dx * dy;

   for (Rox_Sint i = 0; i < rows; i++)
   {
      Rox_Sint ni = i + ity;

      for (Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Sint nj = j + itx;

         dm[i][j] = 0;

         if (ni < 0 || ni >= srows - 1) continue;
         if (nj < 0 || nj >= scols - 1) continue;

         // Added test for borders special case
         if ((nj == scols - 1) || (ni == srows - 1))
         {
            dm[i][j] = ~0;
            dd[i][j] = (Rox_Uchar) (ds[ni][nj] + 0.5f);
            continue;
         }

         Rox_Float v1 = (Rox_Float) ds[ni][nj];
         Rox_Float v2 = (Rox_Float) ds[ni][nj+1];
         Rox_Float v3 = (Rox_Float) ds[ni+1][nj];
         Rox_Float v4 = (Rox_Float) ds[ni+1][nj+1];

         Rox_Float val = b1 * v1 + b2 * v2 + b3 * v3 + b4 * v4;

         dm[i][j] = ~0;
         dd[i][j] = (Rox_Uchar)(val + 0.5f);
      }
   }

function_terminate:
   return error;
}
