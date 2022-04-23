//==============================================================================
//
//    OPENROX   : File draw_warp_rectangle.c
//
//    Contents  : Implementation of draw_warp_rectangle module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "draw_warp_rectangle.h"
#include "draw_polygon.h"

#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_rgba_draw_warp_rectangle(Rox_Image_RGBA output, Rox_Array2D_Double H, Rox_Rect_Sint rectangle, Rox_Uint rgbvalue)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct pts_in[4];
   Rox_Point2D_Double_Struct pts_out[4];

   if (!output || !H) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pts_in[0].u = rectangle->x;
   pts_in[0].v = rectangle->y;

   pts_in[1].u = rectangle->x + rectangle->width;
   pts_in[1].v = rectangle->y;

   pts_in[2].u = rectangle->x + rectangle->width;
   pts_in[2].v = rectangle->y + rectangle->height;

   pts_in[3].u = rectangle->x;
   pts_in[3].v = rectangle->y + rectangle->height;

   error = rox_point2d_double_homography(pts_out, pts_in, H, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_rgba_draw_polygon ( output, pts_out, 4, rgbvalue ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
