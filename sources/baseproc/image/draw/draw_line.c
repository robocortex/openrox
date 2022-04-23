//==============================================================================
//
//    OPENROX   : File draw_line.c
//
//    Contents  : Implementation of draw_line module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "draw_line.h"

#include <float.h>
#include "color.h"

#include <generated/array2d_uint.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/segment/segment2d_struct.h>

#include <inout/system/errors_print.h>

#define setPixel(u,v) if ((u)>=0&&(v)>=0&&(u)<cols&&(v)<rows) dout[(v)][(u)] = lc;

Rox_ErrorCode rox_image_rgba_draw_segment2d (
   Rox_Image_RGBA image_rgba, 
   const Rox_Segment2D segment2d, 
   const Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct one = segment2d->points[0];
   Rox_Point2D_Double_Struct two = segment2d->points[1];
   
   rox_image_rgba_draw_line ( image_rgba, &one, &two, color);
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_line_thick (
   Rox_Image_RGBA image_rgba, 
   const Rox_Point2D_Sint one, 
   const Rox_Point2D_Sint two, 
   const Rox_Uint color, 
   const Rox_Uint thickness
);

Rox_ErrorCode rox_image_rgba_draw_line (
   Rox_Image_RGBA image_rgba, 
   const Rox_Point2D_Double one, 
   const Rox_Point2D_Double two, 
   const Rox_Uint color
)
{
   Rox_Point2D_Sint_Struct one_sint;
   Rox_Point2D_Sint_Struct two_sint;
   one_sint.u = (Rox_Sint) one->u;
   one_sint.v = (Rox_Sint) one->v;
   two_sint.u = (Rox_Sint) two->u;
   two_sint.v = (Rox_Sint) two->v;
   return rox_image_rgba_draw_line_thick(image_rgba, &one_sint, &two_sint, color, 1);
}

Rox_ErrorCode rox_image_rgba_draw_line_float (
   Rox_Image_RGBA image_rgba, 
   Rox_Point2D_Float one, 
   Rox_Point2D_Float two, 
   Rox_Uint color
)
{
   Rox_Point2D_Sint_Struct one_sint;
   Rox_Point2D_Sint_Struct two_sint;
   one_sint.u = (Rox_Sint) one->u;
   one_sint.v = (Rox_Sint) one->v;
   two_sint.u = (Rox_Sint) two->u;
   two_sint.v = (Rox_Sint) two->v;
   return rox_image_rgba_draw_line_thick(image_rgba, &one_sint, &two_sint, color, 1);
}

Rox_ErrorCode rox_image_rgba_draw_line_sint (
   Rox_Image_RGBA image_rgba, 
   const Rox_Point2D_Sint one, 
   const Rox_Point2D_Sint two, 
   const Rox_Uint color
)
{
   return rox_image_rgba_draw_line_thick(image_rgba, one, two, color, 1);
}

Rox_ErrorCode rox_image_rgba_draw_line_thick (
   Rox_Image_RGBA image_rgba, 
   const Rox_Point2D_Sint one, 
   const Rox_Point2D_Sint two, 
   const Rox_Uint color, 
   const Rox_Uint thickness
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Sint dx, dy, sx, sy;
   Rox_Sint err, e2, x2, y2;
   Rox_Double ed, wd;
   Rox_Uint lc = color;

   if (!image_rgba) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (thickness == 0) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** dout = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dout, image_rgba);
   
   Rox_Sint rows = 0, cols = 0; 

   error = rox_array2d_uint_get_size(&rows, &cols, image_rgba); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint x0 = one->u;
   Rox_Sint y0 = one->v;
   Rox_Sint x1 = two->u;
   Rox_Sint y1 = two->v;

   dx = abs(x1-x0);
   if (x0 < x1)
   {
      sx = 1;
   }
   else
   {
      sx = -1;
   }

   dy = abs(y1-y0);
   if (y0 < y1)
   {
      sy = 1;
   }
   else
   {
      sy = -1;
   }

   err = dx - dy;
   if (dx+dy == 0)
   {
      ed = 0;
   }
   else
   {
      ed = sqrt((double)(dx*dx+dy*dy));
   }

   wd = thickness;
   wd = (wd+1.0)/2.0;

   while (1)
   {
      setPixel((int)x0,(int)y0);

      e2 = err;
      x2 = x0;

      if (2 * e2 >= -dx)
      {
         for (e2 += dy, y2 = y0; e2 < ed*wd && (y1 != y2 || dx > dy); e2 += dx)
         {
            y2 += sy;
            setPixel((int)x0,(int)y2);
         }

         if (x0 == x1) break;
         e2 = err;
         err -= dy;
         x0 += sx;

      }

      if (2 *e2 <= dy)
      {
         for (e2 = dx-e2; e2 < ed*wd && (x1 != x2 || dx < dy); e2 += dy)
         {
            x2 += sx;
            setPixel((int)x2,(int)y0);
         }

         if (y0 == y1) break;

         err += dx;
         y0 += sy;
      }
   }

function_terminate:
   return error;
}
