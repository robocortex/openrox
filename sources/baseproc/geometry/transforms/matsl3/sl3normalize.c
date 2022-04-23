//==============================================================================
//
//    OPENROX   : File sl3normalize.c
//
//    Contents  : Implementation of sl3normalize module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sl3normalize.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <baseproc/array/determinant/detgl3.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matsl3_normalize ( Rox_Array2D_Double homography, Rox_Array2D_Double matrix33 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Double onethird = -1.0 / 3.0;
   Rox_Double det = 0.0;
   Rox_Double scale = 0.0;

   if (!homography || !matrix33) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(matrix33, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dh, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dm = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dm, matrix33 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute input determinant
   error = rox_array2d_double_detgl3(&det, matrix33); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Is determinant equal to one ?
   if (fabs(det - 1.0) < DBL_EPSILON && homography != matrix33)
   {
      error = rox_array2d_double_copy(homography, matrix33);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      // Make it determinant equal to one (definition of SL(3) group)
      if (det < -DBL_EPSILON)
      {
         scale = -pow(-det, onethird);
      }
      else if (det > DBL_EPSILON)
      {
         scale = pow(det, onethird);
      }
      else
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; 
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      dh[0][0] = dm[0][0] * scale; dh[0][1] = dm[0][1] * scale; dh[0][2] = dm[0][2] * scale;
      dh[1][0] = dm[1][0] * scale; dh[1][1] = dm[1][1] * scale; dh[1][2] = dm[1][2] * scale;
      dh[2][0] = dm[2][0] * scale; dh[2][1] = dm[2][1] * scale; dh[2][2] = dm[2][2] * scale;
   }
   
function_terminate:
   return error;
}
