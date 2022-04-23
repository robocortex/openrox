//==============================================================================
//
//    OPENROX   : File rox_image_rgba_draw_projection_model_multi_plane.c
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

#include <baseproc/image/draw/image_rgba_draw_projection_model_multi_plane.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <core/model/model_multi_plane.h>

#include <inout/system/errors_print.h>

#define nb_points 4

Rox_ErrorCode rox_image_rgba_draw_projection_model_multi_plane (
   Rox_Image_RGBA image,
   const Rox_Matrix calib,
   const Rox_MatSE3 pose,
   const Rox_Model_Multi_Plane model_multi_plane,
   const Rox_Uint color
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point3D_Double_Struct trvert[nb_points];
   Rox_Point2D_Double_Struct curpts[nb_points];

   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint count_templates = 0;

   error = rox_model_multi_plane_get_number ( &count_templates, model_multi_plane);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_model_multi_plane_set_currentpose ( model_multi_plane, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_model_multi_plane_transform ( model_multi_plane );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint idtemplate = 0; idtemplate < count_templates; idtemplate++)
   {
      error = rox_model_multi_plane_get_vertices_cur ( trvert, model_multi_plane, idtemplate );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point2d_double_project ( curpts, trvert, calib, nb_points);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_image_rgba_draw_2d_polygon ( image, curpts, nb_points, color );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
  return error;
}
