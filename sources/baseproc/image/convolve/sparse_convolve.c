//==============================================================================
//
//    OPENROX   : File sparse_convolve.c
//
//    Contents  : Implementation of sparse_convolve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sparse_convolve.h"

#include <generated/dynvec_sparse_value_struct.h>
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_float_convolve_with_sparse_kernel (
   Rox_Array2D_Float dest,
   const Rox_Array2D_Float source,
   const Rox_DynVec_Sparse_Value kernel
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sparse_Value_Struct * sv = NULL;

   if (!kernel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, dest); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_check_size(source, rows, cols); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dsource = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &dsource, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** ddest = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer( &ddest, dest);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < rows; i++)
   {
      for (Rox_Sint j = 0; j < cols; j++)
      {
         Rox_Double sum = 0.0;
         Rox_Double normer = 0.0;

         sv = &kernel->data[0];
         for (Rox_Uint idsparse = 0; idsparse < kernel->used; idsparse++)
         {
            Rox_Double ker;
            Rox_Sint k = sv->v + i;
            Rox_Sint l = sv->u + j;
            if (k < 0 || l < 0 || k >= rows || l >= cols) continue;

            ker = sv->value;
            normer += ker;

            sum += dsource[k][l] * ker;
            sv++;
         }

         ddest[i][j] = (Rox_Float) (sum / normer);
      }
   }

function_terminate:
   return error;
}
