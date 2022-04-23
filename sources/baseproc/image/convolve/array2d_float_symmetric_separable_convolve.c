//============================================================================
//
//    OPENROX   : File array2d_float_symmetric_separable_convolve.c
//
//    Contents  : Implementation of array2d_float_symmetric_separable_convolve module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "array2d_float_symmetric_separable_convolve.h"
#include "ansi_array2d_float_symmetric_separable_convolve.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>
#include <string.h>

// #define SYMM_IDX(P,MAX) ((P) < 0)?-(P):(((P) > (MAX))?(MAX)-((P)-(MAX)):(P))
// #define SYMM_IDX_MIN(P) (((P) < 0)?-(P):(P))
// #define SYMM_IDX_MAX(P,MAX) (((P) > (MAX))?(MAX)-((P)-(MAX)):(P))

Rox_ErrorCode rox_array2d_float_symmetric_seperable_convolve (
   Rox_Array2D_Float output, 
   Rox_Array2D_Float input, 
   Rox_Array2D_Float kernel
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float **input_data = NULL, **output_data = NULL; 
   Rox_Float **buffer_data = NULL, **imborder_data = NULL;
   
   Rox_Array2D_Float buffer = NULL;
   Rox_Array2D_Float imborder = NULL;

   if (!output || !input || !kernel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Image to convolve size

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_float_get_size(&rows, &cols, output); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Square shaped kernel size
   Rox_Sint ksize = 0;
   error = rox_array2d_float_get_cols(&ksize, kernel);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check input sizes
   if (ksize % 2 == 0) 
   { error = ROX_ERROR_VALUE_NOT_ODD; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint hksize = ksize / 2;

   error = rox_array2d_float_check_size(input, rows, cols);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &buffer  , cols, rows + 2 * hksize );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &imborder, rows, cols + 2 * hksize );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Buffer accessors
   error = rox_array2d_float_get_data_pointer_to_pointer ( &input_data, input );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer ( &output_data, output );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** kernel_data = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &kernel_data, kernel );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_get_data_pointer_to_pointer ( &buffer_data, buffer );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_get_data_pointer_to_pointer ( &imborder_data, imborder );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ansi_array2d_float_symmetric_seperable_convolve ( output_data, input_data, rows, cols, kernel_data, hksize, buffer_data, imborder_data);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_array2d_float_del(&buffer);
   rox_array2d_float_del(&imborder);
   return error;
}
