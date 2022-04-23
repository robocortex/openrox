//==============================================================================
//
//    OPENROX   : File draw_warp_polygon.c
//
//    Contents  : Implementation of draw_warp_polygon module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "draw_warp_polygon.h"
#include "draw_polygon.h"

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_rgba_draw_warp_polygon (
   Rox_Image_RGBA obj, 
   Rox_MatSL3 H, 
   Rox_Point2D_Double pts, 
   Rox_Uint nbpts, 
   Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double  pts_2d = NULL;

   if (!obj || !H || !pts)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pts_2d = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), nbpts);
   if (pts_2d == NULL) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_point2d_double_homography(pts_2d, pts, H, nbpts); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_rgba_draw_polygon(obj, pts_2d, nbpts, color); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_memory_delete(pts_2d);
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_warp_projection_polygon_3d(
   Rox_Image_RGBA obj, 
   Rox_MatSL3 H, 
   Rox_Point3D_Double pts_3d, 
   Rox_Uint nbpts, 
   Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double  pts_2d = NULL;

   if (!obj || !H || !pts_3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pts_2d = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), nbpts);
   if (pts_2d == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_point3d_double_transform_project_homography(pts_2d, H, pts_3d, nbpts); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_image_rgba_draw_polygon(obj, pts_2d, nbpts, color); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_memory_delete(pts_2d);
   return error;
}
