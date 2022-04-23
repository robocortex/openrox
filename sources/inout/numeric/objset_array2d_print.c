//============================================================================
//
//    OPENROX   : File objset_array2d_print.c
//
//    Contents  : Implementation of array2d_serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "objset_array2d_print.h"

#include <generated/objset_array2d_double.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

ROX_API Rox_ErrorCode rox_objset_array2d_double_print(Rox_ObjSet_Array2D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint nb_used = 0;
   error = rox_objset_array2d_double_get_used(&nb_used, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Array2D_Double * A = NULL;
   error = rox_objset_array2d_double_get_data_pointer ( &A, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k = 0; k < nb_used; k++)
   {
      rox_array2d_double_print(A[k]);
   }

function_terminate:
   return error;
}
