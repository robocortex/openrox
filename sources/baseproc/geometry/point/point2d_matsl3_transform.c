//==============================================================================
//
//    OPENROX   : File point2d_matsl3_transform.c
//
//    Contents  : Implementation of point2D transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point2d_matsl3_transform.h"
#include <float.h>
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_point2d_double_homography (
   Rox_Point2D_Double output,
   Rox_Point2D_Double input,
   Rox_MatSL3 homography,
   Rox_Sint count
   )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input || !homography) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dh, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < count; i++)
   {
      Rox_Double x = dh[0][0] * input[i].u + dh[0][1] * input[i].v + dh[0][2];
      Rox_Double y = dh[1][0] * input[i].u + dh[1][1] * input[i].v + dh[1][2];
      Rox_Double w = dh[2][0] * input[i].u + dh[2][1] * input[i].v + dh[2][2];

      output[i].u = x / w;
      output[i].v = y / w;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_float_homography (
   Rox_Point2D_Float output,
   Rox_Point2D_Float input,
   Rox_MatSL3 homography,
   Rox_Sint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input || !homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dh = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dh, homography );

   for (Rox_Sint i = 0; i < count; i++)
   {
      Rox_Float x = (Rox_Float) (dh[0][0] * input[i].u + dh[0][1] * input[i].v + dh[0][2]);
      Rox_Float y = (Rox_Float) (dh[1][0] * input[i].u + dh[1][1] * input[i].v + dh[1][2]);
      Rox_Float w = (Rox_Float) (dh[2][0] * input[i].u + dh[2][1] * input[i].v + dh[2][2]);

      output[i].u = x / w;
      output[i].v = y / w;
   }

function_terminate:
   return error;
}
