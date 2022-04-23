//==============================================================================
//
//    OPENROX   : File pointsfromtemplate.c
//
//    Contents  : Implementation of pointsfromtemplate module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point3d_from_template.h"
#include <generated/dynvec_point3d_float_struct.h>
#include <generated/dynvec_point2d_float_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_points3d_float_from_template (
   Rox_DynVec_Point3D_Float output, 
   const Rox_DynVec_Point2D_Float input, 
   const Rox_Array2D_Double calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output || !input || !calib) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (rox_array2d_double_check_size(calib, 3, 3))
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   output->used = 0;
   rox_dynvec_point3d_float_usecells(output, input->used);

   Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute meters to pixels transformation parameters
   Rox_Double px = dk[0][0];
   Rox_Double py = dk[1][1];
   Rox_Double u0 = dk[0][2];
   Rox_Double v0 = dk[1][2];

   // Compute pixel to meteres transformation parameters
   Rox_Double ipx = 1.0 / px;
   Rox_Double ipy = 1.0 / py;
   Rox_Double iu0 = - u0 * ipx;
   Rox_Double iv0 = - v0 * ipy;

   // Transform all points
   for (Rox_Uint i = 0; i < input->used; i++)
   {
      output->data[i].X = (Rox_Float) (input->data[i].u * ipx + iu0);
      output->data[i].Y = (Rox_Float) (input->data[i].v * ipy + iv0);
      output->data[i].Z = 0;
   }

function_terminate:
   return error;
}
