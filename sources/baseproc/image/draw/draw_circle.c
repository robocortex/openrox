//============================================================================
//
//    OPENROX   : File draw_circle.c
//
//    Contents  : Implementation of draw_circle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "draw_circle.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_rgba_draw_circle(Rox_Array2D_Uint output, Rox_Sint center_u, Rox_Sint center_v, Rox_Sint radius, Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if (!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (radius < 1) 
   {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &data, output);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_uint_get_size(&rows, &cols, output); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double supradius = radius + .5;
   Rox_Double infradius = radius - .5;

   Rox_Double supradiussq = supradius * supradius;
   Rox_Double infradiussq = infradius * infradius;

   for (Rox_Sint i = (Rox_Sint) -supradius; i <= (Rox_Sint)supradius; i++)
   {
      Rox_Double isq = i * i;
      Rox_Sint pv = center_v + i;
      if (pv < 0) continue;
      if (pv >= rows) continue;

      for (Rox_Sint j = (Rox_Sint) -supradius; j <= (Rox_Sint) supradius; j++)
      {
         Rox_Double jsq = j * j;
         Rox_Sint pu = center_u + j;
         if (pu < 0) continue;
         if (pu >= cols) continue;

         Rox_Double val = isq+jsq;

         if (val > infradiussq && val < supradiussq)
         {
            data[pv][pu] = color;
         }
      }
   }

function_terminate:
   return error;
}
