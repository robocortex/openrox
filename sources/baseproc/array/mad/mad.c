//==============================================================================
//
//    OPENROX   : File mad.c
//
//    Contents  : Implementation of madrow module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "mad.h"
#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/median/median.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_double_mad (
   Rox_Double *ret_mad, 
   Rox_Array2D_Double adfrommedian, 
   Rox_Array2D_Double workbuffer, 
   Rox_Array2D_Double input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double median = 0.0;

   if (!ret_mad || !input || !adfrommedian || !workbuffer) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); } 

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_double_get_size(&rows, &cols, input); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check we are computing mad on a vector (rows or cols)
   if (cols < 1 || rows < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (cols > 1 && rows > 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(adfrommedian, rows, cols); 
	ROX_ERROR_CHECK_TERMINATE ( error );
   
	error = rox_array2d_double_check_size(workbuffer, rows, cols); 
	ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve first row pointer
   Rox_Double ** dad2 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dad2, adfrommedian);
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double * dad = dad2[0];

   Rox_Double ** data2 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &data2, input );
   ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double * data = data2[0];

   *ret_mad = 0.0;

   // Compute median
   error = rox_array2d_double_copy(workbuffer, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_median(&median, workbuffer);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // For each element, compute absolute deviation (don't care if the order was changed by median)
   for ( Rox_Sint i = 0; i < cols*rows; i++)
   {
      dad[i] = fabs(data[i] - median);
	}

   // Compute median of absolute deviations
   error = rox_array2d_double_copy ( workbuffer, adfrommedian );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_median(ret_mad, workbuffer);
   ROX_ERROR_CHECK_TERMINATE ( error );

 function_terminate:
  return error;
}
