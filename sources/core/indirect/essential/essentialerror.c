//==============================================================================
//
//    OPENROX   : File essentialerror.c
//
//    Contents  : API of essentialerror module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "essentialerror.h"

#include <baseproc/maths/maths_macros.h>

#include <generated/dynvec_point2d_float_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_essential_geometric_error(Rox_Array2D_Double vecerrors, Rox_Array2D_Double E, Rox_DynVec_Point2D_Float ref, Rox_DynVec_Point2D_Float cur)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!vecerrors || !E || !ref || !cur)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (ref->used != cur->used)
   {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_check_size(E, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(vecerrors, ref->used * 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, E);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * de = NULL;
   error = rox_array2d_double_get_data_pointer (&de, vecerrors);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < ref->used; i++)
   {
      Rox_Double a = dt[0][0] * ref->data[i].u + dt[0][1] * ref->data[i].v + dt[0][2];
      Rox_Double b = dt[1][0] * ref->data[i].u + dt[1][1] * ref->data[i].v + dt[1][2];
      Rox_Double c = dt[2][0] * ref->data[i].u + dt[2][1] * ref->data[i].v + dt[2][2];
      Rox_Double dist = (a * cur->data[i].u + b * cur->data[i].v + c) / sqrt(a*a+b*b);

      de[i * 2] = dist;

      a = dt[0][0] * cur->data[i].u + dt[1][0] * cur->data[i].v + dt[2][0];
      b = dt[0][1] * cur->data[i].u + dt[1][1] * cur->data[i].v + dt[2][1];
      c = dt[0][2] * cur->data[i].u + dt[1][2] * cur->data[i].v + dt[2][2];
      dist = (a * ref->data[i].u + b * ref->data[i].v + c) / sqrt(a*a+b*b);

      de[i * 2 + 1] = dist;
   }

function_terminate:
   return error;
}

