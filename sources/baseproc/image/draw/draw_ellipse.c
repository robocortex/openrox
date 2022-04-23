//============================================================================
//
//    OPENROX   : File draw_ellipse.c
//
//    Contents  : Implementation of draw_ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "draw_ellipse.h"

#include <generated/array2d_uint.h>

#include "baseproc/geometry/ellipse/ellipse2d_struct.h"
#include "baseproc/geometry/ellipse/ellipse3d_struct.h"
#include "baseproc/geometry/ellipse/ellipse_project.h"
#include "baseproc/geometry/measures/intersection_line_ellipse.h"
#include "baseproc/geometry/point/point2d_tools.h"
#include "baseproc/geometry/measures/distance_point_to_point.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>


Rox_ErrorCode rox_image_rgba_draw_ellipse2d ( Rox_Image_RGBA image_rgba, const Rox_Ellipse2D ellipse2d, const Rox_Uint color )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if (!image_rgba || !ellipse2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Uint ** data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &data, image_rgba );

   Rox_Sint cols = 0, rows = 0; 
   error = rox_array2d_uint_get_size(&rows, &cols, image_rgba); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double xc = ellipse2d->xc;
   Rox_Double yc = ellipse2d->yc;
   Rox_Double nxx = ellipse2d->nxx;
   Rox_Double nyy = ellipse2d->nyy;
   Rox_Double nxy = ellipse2d->nxy;

   // Get bounding box
   Rox_Double d  = nxx*nyy - nxy*nxy;
   Rox_Double bx = sqrt(nyy/d);
   Rox_Double by = sqrt(nxx/d);

   for (Rox_Sint y = (Rox_Sint) (yc-by); y <= (Rox_Sint) (yc+by); y++)
   {
      // Check if we are in the image
      if (y < 0) continue;
      if (y >= rows) continue;

      for (Rox_Sint x = (Rox_Sint) (xc-bx); x <= (Rox_Sint) (xc+bx); x++)
      {
         // Check if we are in the image
         if (x < 0) continue;
         if (x >= cols) continue;

         Rox_Point2D_Double_Struct point_intersection;
         Rox_Point2D_Double_Struct point_input;

         point_input.u = x;
         point_input.v = y;

         error = rox_ellipse2d_intersection_line_center_point(&point_intersection, ellipse2d, &point_input);
         ROX_ERROR_CHECK_CONTINUE(error);

         Rox_Double distance = 0.0;

         error = rox_distance_point2d_to_point2d(&distance, &point_intersection, &point_input);
         ROX_ERROR_CHECK_CONTINUE(error);

         if (distance <= 1.0)
         {
            data[y][x] = color;
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_ellipse3d ( Rox_Image_RGBA image_rgba, Rox_Array2D_Double calib, Rox_Array2D_Double pose, Rox_Ellipse3D ellipse3d, Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Ellipse2D ellipse2d = NULL;

   error = rox_ellipse2d_new(&ellipse2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ellipse2d_transform_project_ellipse3d(ellipse2d, calib, pose, ellipse3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_rgba_draw_ellipse2d(image_rgba,  ellipse2d, color);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_ellipse2d_del(&ellipse2d);
   return error;
}


