//==============================================================================
//
//    OPENROX   : File draw_rectangle.c
//
//    Contents  : Implementation of draw_rectangle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "draw_rectangle.h"
#include "draw_line.h"

#include <baseproc/geometry/rectangle/rectangle_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_rgba_draw_rectangle(Rox_Image_RGBA obj, Rox_Rect_Sint rectangle, Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct pts[4];

   if(!obj) 
	{error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   pts[0].u = rectangle->x;
   pts[0].v = rectangle->y;

   pts[1].u = rectangle->x + rectangle->width - 1;
   pts[1].v = rectangle->y;

   pts[2].u = rectangle->x + rectangle->width - 1;
   pts[2].v = rectangle->y + rectangle->height - 1;

   pts[3].u = rectangle->x;
   pts[3].v = rectangle->y + rectangle->height - 1;

   error = rox_image_rgba_draw_line(obj, &pts[0], &pts[1], color); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_rgba_draw_line(obj, &pts[1], &pts[2], color); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_rgba_draw_line(obj, &pts[2], &pts[3], color); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_rgba_draw_line(obj, &pts[3], &pts[0], color); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

