//==============================================================================
//
//    OPENROX   : File remap_bilinear_onepixel.c
//
//    Contents  : Implementation of remap_bilinear_onepixel module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "remap_bilinear_onepixel.h"

#include <baseproc/geometry/point/point2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_remap_onepixel_bilinear_uchar (
   Rox_Uchar * output, 
   const Rox_Image input, 
   const Rox_Point2D_Float pos
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!output || !input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint incols = 0, inrows = 0;
   error = rox_array2d_uchar_get_size(&inrows, &incols, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** in = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &in, input);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cx = (Rox_Sint) pos->u;
   Rox_Sint cy = (Rox_Sint) pos->v;

   Rox_Double ix = cx;
   Rox_Double iy = cy;

   if (cx < 0 || cy < 0) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (cx >= incols - 1 || cy >= inrows - 1) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double dx = pos->u - ix;
   Rox_Double dy = pos->v - iy;

   Rox_Double b1 = (Rox_Double) in[cy][cx];
   Rox_Double b2 = (Rox_Double) in[cy][cx + 1] - b1;
   Rox_Double b3 = (Rox_Double) in[cy + 1][cx] - b1;
   Rox_Double b4 = b1 + (Rox_Double) in[cy + 1][cx + 1] - (Rox_Double) in[cy + 1][cx] - (Rox_Double) in[cy][cx + 1];

   Rox_Double val = b1 + b2 * dx + b3 * dy + b4 * dx * dy;

   *output = (Rox_Uchar) val;

function_terminate:
   return error;
}
