//============================================================================
//
//    OPENROX   : File color.c
//
//    Contents  : Implementation of color module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "color.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_color_get_uchar_r_g_b_a_from_uint_rgba (
   Rox_Uchar * r, 
   Rox_Uchar * g, 
   Rox_Uchar * b, 
   Rox_Uchar * a, 
   Rox_Uint rgba
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   *a = (rgba & 0xFF000000) >> 24;
   *b = (rgba & 0x00FF0000) >> 16;
   *g = (rgba & 0x0000FF00) >>  8;
   *r = (rgba & 0x000000FF);

   return error;
}
