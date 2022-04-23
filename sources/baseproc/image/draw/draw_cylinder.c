//============================================================================
//
//    OPENROX   : File draw_cylinder.c
//
//    Contents  : Implementation of draw_cylinder module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "draw_cylinder.h"
#include "draw_line.h"

#include "baseproc/geometry/cylinder/cylinder2d_struct.h"
#include "baseproc/geometry/cylinder/cylinder3d_struct.h"
#include "baseproc/geometry/cylinder/cylinder_project.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_rgba_draw_cylinder2d(Rox_Image_RGBA image_rgba, Rox_Cylinder2D cylinder2d, Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if (!image_rgba || !cylinder2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Draw visible segments
   error = rox_image_rgba_draw_segment2d(image_rgba, cylinder2d->s1, color);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_rgba_draw_segment2d(image_rgba, cylinder2d->s2, color);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Draw visible ellipses
   // error = rox_image_rgba_draw_ellipse2d(image_rgba, cylinder2d->e1, color);
   // ROX_ERROR_CHECK_TERMINATE ( error );
   
   // error = rox_image_rgba_draw_ellipse2d(image_rgba, cylinder2d->e2, color);
   // ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_cylinder3d(Rox_Image_RGBA image_rgba, Rox_Array2D_Double calib, Rox_Array2D_Double pose, Rox_Cylinder3D cylinder3d, Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Cylinder2D cylinder2d = NULL;
   
   error = rox_cylinder2d_new(&cylinder2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_cylinder2d_transform_project_cylinder3d(cylinder2d, calib, pose, cylinder3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_cylinder2d_print(cylinder2d);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_image_rgba_draw_cylinder2d(image_rgba, cylinder2d, color);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_cylinder2d_del(&cylinder2d);
   return error;
}


