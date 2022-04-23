//==============================================================================
//
//    OPENROX   : File draw_polygon.c
//
//    Contents  : Implementation of draw_polygon module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "draw_polygon.h"
#include "draw_line.h"

#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>

#include <inout/geometry/point/point2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_rgba_draw_polygon(
   Rox_Image_RGBA image_rgba,
   Rox_Point2D_Double pts,
   Rox_Uint nbpts,
   Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!image_rgba || !pts) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint i = 1; i < nbpts; i++)
   {
      error = rox_image_rgba_draw_line(image_rgba, &pts[i-1], &pts[i], color);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_image_rgba_draw_line(image_rgba, &pts[nbpts-1], &pts[0], color);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_projection_3d_polygon (
   Rox_Image_RGBA image_rgba,
   Rox_MatUT3 calibration,
   Rox_MatSE3 pose,
   Rox_Point3D_Double pts,
   Rox_Uint nbpts,
   Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double  pts_2d = NULL;

   if(!image_rgba || !calibration || !pose || !pts) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pts_2d = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), nbpts);
   if (pts_2d == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Projection of the 3D points
   error = rox_point3d_double_transform_project(pts_2d, pose, calibration, pts, nbpts);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Draw the reprojected polygon
   error = rox_image_rgba_draw_polygon(image_rgba, pts_2d, nbpts, color);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_memory_delete(pts_2d);
   return error;
}
