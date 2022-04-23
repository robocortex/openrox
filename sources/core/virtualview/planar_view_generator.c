//==============================================================================
//
//    OPENROX   : File planar_view_generator.c
//
//    Contents  : Implementation of planar_view_generator module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "planar_view_generator.h"

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/point/point3d_sphere.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/remap/remap_ewa_omo/remap_ewa_omo.h>
#include <inout/system/errors_print.h>

#include "stdio.h"

Rox_ErrorCode rox_viewgenerator_planar_new(Rox_ViewGenerator_Planar * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_ViewGenerator_Planar ret = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret  = (Rox_ViewGenerator_Planar) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->homography = NULL;
   ret->invhomography = NULL;
   ret->pose = NULL;
   ret->calib_input = NULL;
   ret->calib_output = NULL;
   ret->generated_grid = NULL;
   ret->generated = NULL;
   ret->generated_mask = NULL;

   error = rox_array2d_double_new(&ret->homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->invhomography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->calib_input, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->calib_output, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_array2d_double_fillunit(ret->calib_input);

   ret->image_height = 1;
   ret->image_width = 1;

   *obj = ret;

function_terminate:
   if (error) rox_viewgenerator_planar_del(&ret);
   return error;
}

Rox_ErrorCode rox_viewgenerator_planar_del(Rox_ViewGenerator_Planar * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_ViewGenerator_Planar todel = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del(&todel->homography);
   rox_array2d_double_del(&todel->invhomography);
   rox_array2d_double_del(&todel->pose);
   rox_array2d_double_del(&todel->calib_input);
   rox_array2d_double_del(&todel->calib_output);
   rox_array2d_uchar_del(&todel->generated);
   rox_array2d_uint_del(&todel->generated_mask);
   rox_meshgrid2d_float_del(&todel->generated_grid);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_viewgenerator_planar_set_source_size(Rox_ViewGenerator_Planar obj, Rox_Sint width, Rox_Sint height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_fillunit(obj->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_array2d_double_set_value(obj->pose, 2, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_build_calibration_matrix_for_template(obj->calib_input, width, height, 1.0, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   obj->image_width = width;
   obj->image_height = height;

function_terminate:
   return error;
}

Rox_ErrorCode rox_viewgenerator_planar_generate (
   Rox_ViewGenerator_Planar obj, 
   Rox_Image source, 
   Rox_Double scale, 
   Rox_Double inplane_rot, 
   Rox_Point3D_Double origin
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint outheight = 0, outwidth = 0;

   // Generate homography for this view
   rox_transformtools_build_calibration_matrix_for_template(obj->calib_output, ((Rox_Double)obj->image_width) / scale, ((Rox_Double)obj->image_height) / scale, 1.0, 1.0);
   rox_transformtools_pose_from_sphere(obj->pose, origin, inplane_rot);
   rox_transformtools_build_homography_intermodel(obj->homography, obj->pose, obj->calib_output, obj->calib_input);
   rox_transformtools_homography_optimalforward(&outwidth, &outheight, obj->homography, obj->image_width, obj->image_height);
   rox_array2d_double_mat3x3_inverse(obj->invhomography, obj->homography);

   // Delete previous generation
   rox_array2d_uchar_del(&obj->generated);
   rox_array2d_uint_del(&obj->generated_mask);
   rox_meshgrid2d_float_del(&obj->generated_grid);

   // Create image buffers
   error = rox_meshgrid2d_float_new(&obj->generated_grid, outheight, outwidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_new(&obj->generated, outheight, outwidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_new(&obj->generated_mask, outheight, outwidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Warp image to synthetic view
   error = rox_warp_grid_sl3_float(obj->generated_grid, obj->invhomography); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_remap_ewa_omo_uchar(obj->generated, obj->generated_mask, source, obj->generated_grid); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
