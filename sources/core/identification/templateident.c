//==============================================================================
//
//    OPENROX   : File templateident.c
//
//    Contents  : Implementation of templateident module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "templateident.h"
#include "templateident_struct.h"

#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/transforms/matsl3/sl3virtualview.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/image/remap/remap_bilinear_float_to_float/remap_bilinear_float_to_float.h>
#include <baseproc/image/remap/remap_bilinear_nomask_float_to_float/remap_bilinear_nomask_float_to_float_doubled.h>

#include <core/features/descriptors/sraid/sraid.h>
#include <core/features/descriptors/sraid/sraid_matchset.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_template_ident_new(Rox_Template_Ident *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Template_Ident ret = NULL;
   Rox_DynVec_SRAID_Feature localfeats = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret = (Rox_Template_Ident) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->reference_features_subsets = NULL;
   ret->current_points_matched = NULL;
   ret->reference_points_matched = NULL;
   ret->calib_input = NULL;

   // Create various point lists
   error = rox_objset_dynvec_sraid_feature_new(&ret->reference_features_subsets, 14);
   ROX_ERROR_CHECK_TERMINATE(error)

   for (Rox_Sint idset = 0; idset < 12 + 1; idset++)
   {
      error = rox_dynvec_sraiddesc_new(&localfeats, 100);
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_objset_dynvec_sraid_feature_append(ret->reference_features_subsets, localfeats);
      if (error)
      {
         rox_dynvec_sraiddesc_del(&localfeats);
         ROX_ERROR_CHECK_TERMINATE(error)
      }
   }

   error = rox_array2d_double_new(&ret->calib_input, 3, 3);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_float_new(&ret->current_points_matched, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_float_new(&ret->reference_points_matched, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_array2d_double_fillunit(ret->calib_input);

   // Store parameters
   ret->model_height_pixels = 1;
   ret->model_width_pixels = 1;

   *obj = ret;

function_terminate:
   if (error) rox_template_ident_del(&ret);
   return error;
}

Rox_ErrorCode rox_template_ident_del(Rox_Template_Ident *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Template_Ident todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_array2d_double_del(&todel->calib_input);
   rox_dynvec_point2d_float_del(&todel->current_points_matched);
   rox_dynvec_point2d_float_del(&todel->reference_points_matched);
   rox_objset_dynvec_sraid_feature_del(&todel->reference_features_subsets);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_template_ident_set_model(Rox_Template_Ident obj, Rox_Array2D_Float model_image, Rox_Array2D_Double calib_template, Rox_Uint dbl_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float source = NULL;


   if (!obj || !model_image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Non mandatory copy (calib_template may be null)
   rox_array2d_double_copy(obj->calib_input, calib_template);

   // Assign model sizes

   error = rox_array2d_float_get_size(&obj->model_height_pixels, &obj->model_width_pixels, model_image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (dbl_image)
   {
      error = rox_array2d_float_new(&source, obj->model_height_pixels * 2, obj->model_width_pixels * 2); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = remap_bilinear_nomask_float_to_float_doubled(source, model_image); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      source = model_image;
   }

   // Detect in main view
   error = rox_sraidpipeline_process(obj->reference_features_subsets->data[0], source, -1, 1.6f, 3.0f, 0.5f, 3, 0.04f, 10, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rescale
   if (dbl_image)
   {
      for (Rox_Uint idpt = 0; idpt < obj->reference_features_subsets->data[0]->used; idpt++)
      {
         obj->reference_features_subsets->data[0]->data[idpt].x /= 2.0;
         obj->reference_features_subsets->data[0]->data[idpt].y /= 2.0;
      }
   }

   // No affine features
   for (Rox_Uint idaffine = 0; idaffine < 12; idaffine++)
   {
      rox_dynvec_sraiddesc_reset(obj->reference_features_subsets->data[idaffine + 1]);
   }

function_terminate:
   if (dbl_image) rox_array2d_float_del(&source);

   return error;
}

Rox_ErrorCode rox_template_ident_set_model_affine(Rox_Template_Ident obj, Rox_Array2D_Float model_image, Rox_Array2D_Double calib_template, Rox_Uint dbl_image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Float source = NULL;
   Rox_Array2D_Double homographies[12];
   Rox_Sint new_widths[12];
   Rox_Sint new_heights[12];
   Rox_Double maxskew;
   Rox_Double **dh = NULL;
   Rox_Array2D_Uint omi = NULL, omo = NULL, im = NULL;
   Rox_MeshGrid2D_Float grid = NULL;
   Rox_Array2D_Float target = NULL;


   if (!obj || !model_image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Non mandatory copy (calib_template may be null)
   if(calib_template) rox_array2d_double_copy(obj->calib_input, calib_template);

   // Assign model sizes

   error = rox_array2d_float_get_size(&obj->model_height_pixels, &obj->model_width_pixels, model_image); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   maxskew  = (Rox_Double) obj->model_height_pixels / (Rox_Double) obj->model_width_pixels * 0.7;

   for (Rox_Sint idaffine = 0; idaffine < 12; idaffine++)
   {

      error = rox_array2d_double_new(&homographies[idaffine], 3, 3); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      new_widths[idaffine]  = 1;
      new_heights[idaffine] = 1;
   }

   if (dbl_image)
   {
      obj->model_height_pixels *= 2;
      obj->model_width_pixels  *= 2;

      
      error = rox_array2d_float_new(&source, obj->model_height_pixels, obj->model_width_pixels); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = remap_bilinear_nomask_float_to_float_doubled(source, model_image); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      source = model_image;
   }


   error = rox_virtualview_anisotropic_scaling(&new_widths[0], &new_heights[0], homographies[0], 1.59, 1.0, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_anisotropic_scaling(&new_widths[1], &new_heights[1], homographies[1], 2.52, 1.0, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_anisotropic_scaling(&new_widths[2], &new_heights[2], homographies[2], 4.0, 1.0, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_anisotropic_scaling(&new_widths[3], &new_heights[3], homographies[3], 1.0, 1.59, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_anisotropic_scaling(&new_widths[4], &new_heights[4], homographies[4], 1.0, 2.52, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_anisotropic_scaling(&new_widths[5], &new_heights[5], homographies[5], 1.0, 4.0, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_virtualview_skew(&new_widths[6], &new_heights[6], homographies[6], 0.48 * maxskew, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_skew(&new_widths[7], &new_heights[7], homographies[7], 0.69 * maxskew, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_skew(&new_widths[8], &new_heights[8], homographies[8], 1.0 * maxskew, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_skew(&new_widths[9], &new_heights[9], homographies[9], -0.48 * maxskew, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_skew(&new_widths[10], &new_heights[10], homographies[10], -0.69 * maxskew, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_virtualview_skew(&new_widths[11], &new_heights[11], homographies[11], -1.0 * maxskew, obj->model_width_pixels, obj->model_height_pixels); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_sraidpipeline_process(obj->reference_features_subsets->data[0], source, -1, 1.6f, 3.0f, 0.5f, 3, 0.04f, 10, 0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint idaffine = 0; idaffine < 12; idaffine++)
   {
      error = rox_array2d_uint_new(&omi, new_heights[idaffine], new_widths[idaffine]); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_uint_new(&omo, new_heights[idaffine], new_widths[idaffine]); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_uint_new(&im, obj->model_height_pixels, obj->model_width_pixels); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_meshgrid2d_float_new(&grid, new_heights[idaffine], new_widths[idaffine]); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_float_new(&target, new_heights[idaffine], new_widths[idaffine]); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer( &dh, homographies[idaffine]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_uint_fillval(omi, ~0);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_uint_fillval(omo, ~0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_uint_fillval(im, ~0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_warp_grid_sl3_float ( grid, homographies[idaffine]); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_remap_bilinear_float_to_float(target, omo, omi, source, im, grid); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_sraidpipeline_process(obj->reference_features_subsets->data[idaffine + 1], target, -1, 1.6f, 3.0f, 0.5f, 3, 0.04f, 10, 0); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      for (Rox_Uint idpt = 0; idpt < obj->reference_features_subsets->data[idaffine + 1]->used; idpt++)
      {
         Rox_Double u, v, x, y, w;
         u = obj->reference_features_subsets->data[idaffine + 1]->data[idpt].x;
         v = obj->reference_features_subsets->data[idaffine + 1]->data[idpt].y;

         x = dh[0][0] * u + dh[0][1] * v + dh[0][2];
         y = dh[1][0] * u + dh[1][1] * v + dh[1][2];
         w = dh[2][0] * u + dh[2][1] * v + dh[2][2];

         if (dbl_image)
         {
            w *= 2;
         }
         obj->reference_features_subsets->data[idaffine + 1]->data[idpt].x = (Rox_Float) (x / w);
         obj->reference_features_subsets->data[idaffine + 1]->data[idpt].y = (Rox_Float) (y / w);
      }

      rox_meshgrid2d_float_del(&grid);
      rox_array2d_float_del(&target);
      rox_array2d_uint_del(&omi);
      rox_array2d_uint_del(&omo);
      rox_array2d_uint_del(&im);
   }

   if (dbl_image)
   {
      obj->model_height_pixels /= 2;
      obj->model_width_pixels /= 2;
   }

function_terminate:

   if(grid)     rox_meshgrid2d_float_del(&grid);
   if(target)  rox_array2d_float_del(&target);
   if(omi)     rox_array2d_uint_del(&omi);
   if(omo)     rox_array2d_uint_del(&omo);
   if(im)      rox_array2d_uint_del(&im);

   if (dbl_image) rox_array2d_float_del(&source);

   for (Rox_Sint idaffine = 0; idaffine < 12; idaffine++)
   {
      rox_array2d_double_del(&homographies[idaffine]);
   }

   return error;
}

