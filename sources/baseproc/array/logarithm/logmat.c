//==============================================================================
//
//    OPENROX   : File logmat.c
//
//    Contents  : Implementation of logmat module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "logmat.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <baseproc/array/add/add.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>

#include <inout/system/errors_print.h>


Rox_ErrorCode rox_array2d_double_logmat(Rox_Double *axis_x, Rox_Double *axis_y, Rox_Double *axis_z, Rox_Double *angle, Rox_Array2D_Double rotation)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!rotation || !axis_x || !axis_y || !axis_z || !angle) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *axis_x = 1;
   *axis_y = 0;
   *axis_z = 0;
   *angle = 0;

   // We work on the top left 3*3 sub matrix, to make it work on pose matrix or rotation matrix
   Rox_Sint cols = 0, rows = 0;

   error = rox_array2d_double_get_size(&rows, &cols, rotation); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   if (cols < 3 || rows < 3) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); } 

   Rox_Double ** dr = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dr, rotation );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double cos_theta = 0.5 * (dr[0][0] + dr[1][1] + dr[2][2] - 1.0);

   Rox_Double ax = 0.5 * (dr[2][1] - dr[1][2]);
   Rox_Double ay = 0.5 * (dr[0][2] - dr[2][0]);
   Rox_Double az = 0.5 * (dr[1][0] - dr[0][1]);

   Rox_Double sin_theta = sqrt(ax * ax + ay * ay + az * az);

   if (sin_theta > DBL_EPSILON)
   {
      *axis_x = ax / sin_theta;
      *axis_y = ay / sin_theta;
      *axis_z = az / sin_theta;
      *angle = atan2(sin_theta, cos_theta);
   }
   else
   {
      *axis_x = 0;
      *axis_y = 0;
      *axis_z = 1;
      *angle = atan2(sin_theta, cos_theta);
   }
   
function_terminate:
   return error;
}