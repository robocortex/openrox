//==============================================================================
//
//    OPENROX   : File draw_points.c
//
//    Contents  : Implementation of draw_points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "draw_points.h"
#include "draw_line.h"

#include <generated/dynvec_point2d_double_struct.h>
#include <generated/array2d_uint.h>
#include <core/features/detectors/segment/segmentpoint_struct.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

// TODO: rox_image_rgba_draw_2d_points should have as input Rox_Point2D_Sint *pts
// write function rox_image_rgba_draw_2d_points_float  which convert Rox_Point2D_Float  points to Rox_Point2D_Sint points and call rox_image_rgba_draw_2d_points
// write function rox_image_rgba_draw_2d_points_double which convert Rox_Point2D_Double points to Rox_Point2D_Sint points and call rox_image_rgba_draw_2d_points

Rox_ErrorCode rox_image_rgba_draw_dynvec_point2d_double (
   Rox_Image_RGBA output, 
   Rox_DynVec_Point2D_Double pts, 
   Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!pts)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_image_rgba_draw_2d_points(output, pts->data, pts->used, color);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_2d_points (
   Rox_Image_RGBA output,
   Rox_Point2D_Double pts,
   Rox_Uint nbpts,
   Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!pts)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      Rox_Point2D_Double_Struct pt1, pt2;

      pt1.u = pts[i].u - 2; pt1.v = pts[i].v - 2;
      pt2.u = pts[i].u + 2; pt2.v = pts[i].v + 2;
      error = rox_image_rgba_draw_line(output, &pt1, &pt2, color);
      ROX_ERROR_CHECK_TERMINATE ( error );

      pt1.u = pts[i].u - 2; pt1.v = pts[i].v + 2;
      pt2.u = pts[i].u + 2; pt2.v = pts[i].v - 2;
      error = rox_image_rgba_draw_line(output, &pt1, &pt2, color);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_2d_points_sint (
   Rox_Image_RGBA output,
   Rox_Point2D_Sint  pts,
   Rox_Uint nbpts,
   Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!pts)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      Rox_Point2D_Sint_Struct pt1, pt2;

      pt1.u = pts[i].u - 2; pt1.v = pts[i].v - 2;
      pt2.u = pts[i].u + 2; pt2.v = pts[i].v + 2;
      error = rox_image_rgba_draw_line_sint(output, &pt1, &pt2, color);
      ROX_ERROR_CHECK_TERMINATE ( error );

      pt1.u = pts[i].u - 2; pt1.v = pts[i].v + 2;
      pt2.u = pts[i].u + 2; pt2.v = pts[i].v - 2;
      error = rox_image_rgba_draw_line_sint(output, &pt1, &pt2, color);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_2d_points_float (
   Rox_Image_RGBA output,
   Rox_Point2D_Float  pts,
   Rox_Uint nbpts,
   Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!pts)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      Rox_Point2D_Float_Struct pt1, pt2;
      pt1.u = pts[i].u - 2; pt1.v = pts[i].v - 2;
      pt2.u = pts[i].u + 2; pt2.v = pts[i].v + 2;
      error = rox_image_rgba_draw_line_float(output, &pt1, &pt2, color);
      ROX_ERROR_CHECK_TERMINATE ( error );

      pt1.u = pts[i].u - 2; pt1.v = pts[i].v + 2;
      pt2.u = pts[i].u + 2; pt2.v = pts[i].v - 2;
      error = rox_image_rgba_draw_line_float(output, &pt1, &pt2, color);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_2d_segment_points (
   Rox_Image_RGBA output,
   Rox_Segment_Point pts,
   Rox_Uint nbpts,
   Rox_Uint color)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!pts)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      Rox_Point2D_Sint_Struct pt1, pt2;
      pt1.u = pts[i].i - 2; pt1.v = pts[i].j - 2;
      pt2.u = pts[i].i + 2; pt2.v = pts[i].j + 2;
      error = rox_image_rgba_draw_line_sint(output, &pt1, &pt2, color);
      ROX_ERROR_CHECK_TERMINATE ( error );

      pt1.u = pts[i].i - 2; pt1.v = pts[i].j + 2;
      pt2.u = pts[i].i + 2; pt2.v = pts[i].j - 2;
      error = rox_image_rgba_draw_line_sint(output, &pt1, &pt2, color);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_image_rgba_draw_2d_points_array_float (
   Rox_Image_RGBA image_rgba,
   Rox_Float * pts,
   Rox_Uint nbpts,
   Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!image_rgba)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!pts)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** image_rgba_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer( &image_rgba_data, image_rgba);
   
   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      int u = (int) pts[2*i+0];
      int v = (int) pts[2*i+1];

      image_rgba_data[v][u] = color;
   }

function_terminate:
   return error;
}
