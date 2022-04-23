//==============================================================================
//
//    OPENROX   : File rox_image_rgba_draw_projection_model_single_plane.c
//
//    Contents  : Implementation of image_rgba_draw module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/image/draw/image_rgba_draw_projection_model_single_plane.h>

// #include <core/model/model_single_plane_struct.h>
#include <core/model/model_single_plane_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_image_rgba_draw_projection_model_single_plane (
   Rox_Image_RGBA image, 
   const Rox_Matrix calib, 
   const Rox_MatSE3 pose, 
   const Rox_Model_Single_Plane model, 
   const Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct pts[4];

   if (!model || !image || !calib || !pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double model_sizex = model->sizex;
   Rox_Double model_sizey = model->sizey;

   // Define 3D pts 
   pts[0].X = -model_sizex / 2.0;   pts[0].Y = -model_sizey / 2.0; pts[0].Z = 0;
   pts[1].X =  model_sizex / 2.0;   pts[1].Y = -model_sizey / 2.0; pts[1].Z = 0;
   pts[2].X =  model_sizex / 2.0;   pts[2].Y =  model_sizey / 2.0; pts[2].Z = 0;
   pts[3].X = -model_sizex / 2.0;   pts[3].Y =  model_sizey / 2.0; pts[3].Z = 0;

   // should be pts = model->vertices_ref

   error = rox_image_rgba_draw_projection_3d_polygon ( image, calib, pose, pts, 4, color );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

  return error;
}
