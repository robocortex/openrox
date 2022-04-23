//==============================================================================
//
//    OPENROX   : File odometry_dense_depthmap.c
//
//    Contents  : Implementation of odometry_dense_depthmap module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_dense_depthmap.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillzero.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/mean/mean.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/error/centered_error.h>
#include <baseproc/array/band/band.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/scale/scaleshift.h>
#include <baseproc/array/robust/huber.h>
#include <baseproc/array/crosscor/zncrosscor.h>
#include <baseproc/array/conversion/array2d_uchar_from_float.h>
#include <baseproc/array/conversion/array2d_float_from_uchar.h>

#include <baseproc/calculus/linsys/linsys_texture_matse3_light_affine_model3d_zi.h>
#include <baseproc/calculus/linsys/linsys_texture_matse3_model3d_zi.h>

#include <baseproc/geometry/pixelgrid/warp_grid_matse3_zi.h>

#include <baseproc/image/remap/remap_bilinear_float_to_float/remap_bilinear_float_to_float.h>
#include <baseproc/image/gradient/basegradient.h>
#include <baseproc/maths/maths_macros.h>

#include <inout/image/pgm/pgmfile.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/numeric/array2d_save.h>

// #define DEPTH
// #define NORMALIZE

#ifdef ANDROID
   #define RESULT_PATH "/storage/emulated/0/Documents/Robocortex/Tests/Results/"
#else
   #define RESULT_PATH "./"
#endif

// #define SAVE_DEBUG

#ifdef SAVE_DEBUG
   static int count = 0;
   static char filename[FILENAME_MAX];
#endif

Rox_ErrorCode rox_odometry_dense_depthmap_new (
   Rox_Odometry_Dense_DepthMap * obj,
   const Rox_Sint width,
   const Rox_Sint height
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Dense_DepthMap ret = NULL;

   if ( !obj )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Odometry_Dense_DepthMap) rox_memory_allocate(sizeof(*ret), 1);

   if ( !ret )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->pose = NULL;
   ret->calibration = NULL;

   ret->Zir = NULL;
   ret->Ziur = NULL;
   ret->Zivr = NULL;
   ret->depthmap = NULL;
   ret->depthmap_validity = NULL;

   ret->image_ref = NULL;

   ret->warped_grid = NULL;
   ret->warped_grid_mask = NULL;

   ret->warped_image = NULL;
   ret->warped_image_validity = NULL;

   ret->image_cur_mask = NULL;

   ret->difference = NULL;
   ret->mean = NULL;
   ret->mean_lum = NULL;
   ret->weights = NULL;
   ret->Iu = NULL;
   ret->Iv = NULL;
   ret->gradient_mask = NULL;
   ret->JtJ = NULL;
   ret->Jtf = NULL;
   ret->iJtJ = NULL;
   ret->solution = NULL;
   ret->solution_pose = NULL;

   ret->max_iters = 100;

   error = rox_array2d_float_new ( &ret->Zir, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &ret->Ziur, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &ret->Zivr, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &ret->depthmap, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new ( &ret->depthmap_validity, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &ret->image_ref, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_meshgrid2d_float_new ( &ret->warped_grid, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new ( &ret->warped_grid_mask, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new ( &ret->warped_image, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new ( &ret->warped_image_validity, height, width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->image_cur_mask, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->difference, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->weights, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->mean, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->mean_lum, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->Iu, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_new(&ret->Iv, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->gradient_mask, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->JtJ, 8, 8);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &ret->Jtf, 8, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &ret->iJtJ, 8, 8 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &ret->solution, 8, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d ( &ret->solution_pose, ret->solution, 0, 0, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &ret->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_new ( &ret->calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_fillval(ret->image_cur_mask, ~0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->calibration);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->alpha = 1.0;
   ret->beta = 0.0;

   *obj = ret;

function_terminate:
   if (error) rox_odometry_dense_depthmap_del(&ret);
   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_del (
   Rox_Odometry_Dense_DepthMap * obj
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Dense_DepthMap todel = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matut3_del(&todel->calibration);
   rox_matse3_del(&todel->pose);
   rox_array2d_float_del(&todel->Zir);
   rox_array2d_float_del(&todel->Ziur);
   rox_array2d_float_del(&todel->Zivr);

   rox_array2d_float_del(&todel->depthmap);
   rox_array2d_float_del(&todel->warped_image);
   rox_array2d_float_del(&todel->image_ref);
   rox_array2d_float_del(&todel->difference);
   rox_array2d_float_del(&todel->mean);
   rox_array2d_float_del(&todel->mean_lum);
   rox_array2d_float_del(&todel->weights);
   rox_array2d_float_del(&todel->Iu);
   rox_array2d_float_del(&todel->Iv);

   rox_array2d_uint_del(&todel->depthmap_validity);
   rox_array2d_uint_del(&todel->warped_grid_mask);
   rox_array2d_uint_del(&todel->warped_image_validity);
   rox_array2d_uint_del(&todel->image_cur_mask);
   rox_array2d_uint_del(&todel->gradient_mask);

   rox_meshgrid2d_float_del(&todel->warped_grid);

   rox_array2d_double_del(&todel->JtJ);
   rox_array2d_double_del(&todel->Jtf);
   rox_array2d_double_del(&todel->iJtJ);
   rox_array2d_double_del(&todel->solution_pose);
   rox_array2d_double_del(&todel->solution);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_set_calibration (
   Rox_Odometry_Dense_DepthMap obj,
   const Rox_MatUT3 calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matut3_copy(obj->calibration, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_set_pose (
   Rox_Odometry_Dense_DepthMap obj,
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy(obj->pose, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_set_predicition (
   Rox_Odometry_Dense_DepthMap obj,
   const Rox_MatSE3 cTc
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_MatSE3 pose = NULL;
   error = rox_matse3_new ( &pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_copy ( pose, obj->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_mulmatmat ( obj->pose, cTc, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matse3_del ( &pose );
   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_get_score (
   Rox_Double * score ,
   const Rox_Odometry_Dense_DepthMap obj
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!score)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *score = obj->score;

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_get_pose (
   Rox_MatSE3 pose,
   const Rox_Odometry_Dense_DepthMap obj
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy(pose, obj->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

ROX_API Rox_ErrorCode rox_odometry_dense_depthmap_get_results (
   Rox_Sint * success,
   Rox_Double * score,
   Rox_MatSE3 pose,
   const Rox_Odometry_Dense_DepthMap obj
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   *success = 0;

   error = rox_odometry_dense_depthmap_get_pose ( pose, obj );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_odometry_dense_depthmap_get_score ( score, obj );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *success = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_set_reference_depth (
   Rox_Odometry_Dense_DepthMap obj,
#ifdef IMAGE_FLOAT
   const Rox_Array2D_Float Ir,
#else
   const Rox_Image Ir,
#endif
   const Rox_Array2D_Float depth,
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

#ifdef IMAGE_FLOAT
   #ifdef NORMALIZE
      error = rox_array2d_float_scale ( obj->image_ref, Ir, 1.0f/255.0f );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #else
      error = rox_array2d_float_copy ( obj->image_ref, Ir );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #endif
#else
   // Convert image to float
   #ifdef NORMALIZE
      error = rox_array2d_float_from_uchar_normalize ( obj->image_ref, Ir );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #else
      error = rox_array2d_float_from_uchar ( obj->image_ref, Ir );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #endif
#endif

   error = rox_array2d_float_copy(obj->depthmap, depth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_copy(obj->depthmap_validity, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->alpha = 1.0;
   obj->beta = 0.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_set_reference (
   Rox_Odometry_Dense_DepthMap obj,
#ifdef IMAGE_FLOAT
   const Rox_Array2D_Float Ir,
#else
   const Rox_Image Ir,
#endif
   const Rox_Array2D_Float Zir,
   const Rox_Array2D_Float Ziur,
   const Rox_Array2D_Float Zivr,
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

#ifdef IMAGE_FLOAT
   #ifdef NORMALIZE
      error = rox_array2d_float_scale ( obj->image_ref, Ir, (Rox_Float) 1.0/255.0 );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #else
      error = rox_array2d_float_copy ( obj->image_ref, Ir );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #endif
#else
   // Convert image to float
   #ifdef NORMALIZE
      error = rox_array2d_float_from_uchar_normalize ( obj->image_ref, Ir );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #else
      error = rox_array2d_float_from_uchar ( obj->image_ref, Ir );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #endif
#endif

   error = rox_array2d_float_copy(obj->Zir, Zir);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_copy(obj->Ziur, Ziur);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_copy(obj->Zivr, Zivr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_copy(obj->depthmap_validity, mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->alpha = 1.0;
   obj->beta = 0.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_dense_compute_weights (
   Rox_Odometry_Dense_DepthMap obj,
   Rox_Uint count_valid
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double vecw = NULL, vecd = NULL, vecwb1 = NULL, vecwb2 = NULL;


   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new ( &vecw, count_valid, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &vecd, count_valid, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &vecwb1, count_valid, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &vecwb2, count_valid, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dm   = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dm, obj->warped_image_validity );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dd  = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dd, obj->difference );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dw  = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dw, obj->weights );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dvw = NULL;
   error = rox_array2d_double_get_data_pointer ( &dvw, vecw );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dvd = NULL;
   error = rox_array2d_double_get_data_pointer ( &dvd, vecd );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width = 0, height = 0;

   error = rox_array2d_float_get_size ( &height, &width, obj->difference );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint pos = 0;
   for ( Rox_Sint i = 0; i < height; i++)
   {
      for ( Rox_Sint j = 0; j < width; j++)
      {
         if (dm[i][j] == 0) continue;
         dvd[pos] = dd[i][j];
         pos++;
      }
   }

   error = rox_array2d_double_huber ( vecw, vecwb1, vecwb2, vecd );
   ROX_ERROR_CHECK_TERMINATE ( error );

   pos = 0;
   for ( Rox_Sint i = 0; i < height; i++)
   {
      for ( Rox_Sint j = 0; j < width; j++)
      {
         if (dm[i][j] == 0) continue;
         dw[i][j] = (Rox_Float) dvw[pos];
         pos++;
      }
   }

function_terminate:

   rox_array2d_double_del(&vecw);
   rox_array2d_double_del(&vecd);
   rox_array2d_double_del(&vecwb1);
   rox_array2d_double_del(&vecwb2);

   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_prepare (
   Rox_Odometry_Dense_DepthMap obj,
#ifdef IMAGE_FLOAT
   const Rox_Array2D_Float image
#else
   #ifdef NORMALIZE
      const Rox_Array2D_Float image
   #else
      const Rox_Image image
   #endif
#endif
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !obj ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !image )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //rox_matse3_print(obj->pose);
   //rox_matut3_print(obj->calibration);

   // Create the map to warp the current image
#ifdef DEPTH
   error = rox_warp_grid_float_matse3_z_float ( obj->warped_grid, obj->warped_grid_mask, obj->depthmap, obj->pose, obj->calibration, obj->calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );
#else
   error = rox_warp_grid_float_matse3_zi_float ( obj->warped_grid, obj->warped_grid_mask, obj->Zir, obj->pose, obj->calibration, obj->calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );
#endif

   // Set the mask
   error = rox_array2d_uint_band ( obj->warped_grid_mask, obj->warped_grid_mask, obj->depthmap_validity );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Warp the current image
#ifdef IMAGE_FLOAT
   // input image is float, output warped image is float
   //error = rox_remap_bilinear_float_to_float ( obj->warped_image, obj->warped_image_validity, obj->warped_grid_mask, image, obj->image_cur_mask, obj->warped_grid );
   //ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_remap_bilinear_float_to_float ( obj->warped_image, obj->warped_image_validity, obj->warped_grid_mask, image, obj->image_cur_mask, obj->warped_grid );
   ROX_ERROR_CHECK_TERMINATE ( error );

#else
   #ifdef NORMALIZE
      // input image is float, output warped image is float
      //error = rox_remap_bilinear_float_to_float ( obj->warped_image, obj->warped_image_validity, obj->warped_grid_mask, image, obj->image_cur_mask, obj->warped_grid );
      //ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_remap_bilinear_float_to_float ( obj->warped_image, obj->warped_image_validity, obj->warped_grid_mask, image, obj->image_cur_mask, obj->warped_grid );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #else
      // input image is uchar, output warped image is float
      //error = rox_remap_bilinear_uchar_to_uchar_to_float ( obj->warped_image, obj->warped_image_validity, obj->warped_grid_mask, image, obj->image_cur_mask, obj->warped_grid );
      //ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_remap_bilinear_uchar_to_uchar_to_float ( obj->warped_image, obj->warped_image_validity, obj->warped_grid_mask, image, obj->image_cur_mask, obj->warped_grid );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #endif
#endif


   // compute the mean
   // error = rox_array2d_float_mean ( obj->mean, obj->warped_image, obj->image_ref );
   // ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_scaleshift ( obj->warped_image, obj->warped_image, (Rox_Float) obj->alpha, (Rox_Float) obj->beta );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_mean ( obj->mean_lum, obj->warped_image, obj->image_ref );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_substract ( obj->difference, obj->image_ref, obj->warped_image );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_array2d_float_centered_error ( &obj->median, &obj->sum_square, &obj->count_pixels, obj->difference, obj->warped_image_validity, obj->image_ref, obj->warped_image );
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // Robust estimation with Huber
   // error = rox_odometry_dense_compute_weights( obj, obj->count_pixels );
   // ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_fillzero ( obj->Iu );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_fillzero ( obj->Iv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_float_basegradient ( obj->Iu, obj->Iv, obj->gradient_mask, obj->mean_lum, obj->warped_image_validity );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set weights to 1,0 to have faster convergence but less robustnees
   //error = rox_array2d_float_fillval ( obj->weights, 1.0f );
   //ROX_ERROR_CHECK_TERMINATE ( error );

   // The definition of this function is in interaction_matse3_light_affine_texture_model3d.h

#ifdef DEPTH
   error = rox_linsys_weighted_texture_matse3_light_affine_model3d ( obj->JtJ, obj->Jtf, obj->calibration, obj->pose, obj->Iu, obj->Iv, obj->depthmap, obj->mean_lum, obj->difference, obj->weights, obj->gradient_mask );
   ROX_ERROR_CHECK_TERMINATE ( error );
#else
   //error = rox_linsys_weighted_texture_matse3_light_affine_model3d_zi ( obj->JtJ, obj->Jtf, obj->calibration, obj->pose, obj->Zir, obj->Ziur, obj->Zivr, obj->Iu, obj->Iv, obj->difference, obj->mean_lum, obj->weights, obj->gradient_mask );
   //ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_linsys_texture_matse3_light_affine_model3d_zi ( obj->JtJ, obj->Jtf, obj->calibration, obj->pose, obj->Zir, obj->Ziur, obj->Zivr, obj->Iu, obj->Iv, obj->difference, obj->mean_lum, obj->gradient_mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The input order is not consistent with the previous function
   //error = rox_linsys_texture_matse3_model3d_zi ( obj->JtJ, obj->Jtf, obj->Iu, obj->Iv, obj->difference, obj->Zir, obj->Ziur, obj->Zivr, obj->calibration, obj->pose, obj->gradient_mask );
   //ROX_ERROR_CHECK_TERMINATE ( error );

#endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_dense_depthmap_make (
   Rox_Odometry_Dense_DepthMap obj,
#ifdef IMAGE_FLOAT
   const Rox_Array2D_Float image
#else
   const Rox_Image image
#endif
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double ** sol_data = NULL;
   Rox_Double norm_tra = 0.0, norm_rot = 0.0;
   // Rox_Double ratio_pixels = 0.0;
   Rox_Double zncc_prev = 0, zncc_cur = 0;

   if ( !obj )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !image )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_float_get_size ( &rows, &cols, obj->depthmap );
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifdef IMAGE_FLOAT
   Rox_Array2D_Float Ic = image;
   #ifdef NORMALIZE
      error = rox_array2d_float_scale_inplace ( Ic, 1.0f/255.0f );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #endif
#else
   #ifdef NORMALIZE
      Rox_Array2D_Float Ic = NULL;
      error = rox_array2d_float_new_from_uchar_normalize ( &Ic, image );
      ROX_ERROR_CHECK_TERMINATE ( error );
   #else
      Rox_Image Ic = image;
   #endif
#endif


   error = rox_array2d_double_get_data_pointer_to_pointer(&sol_data, obj->solution);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->score = 0.0;

   for (Rox_Uint iter = 0; iter < obj->max_iters; iter++)
   {
      error = rox_odometry_dense_depthmap_prepare ( obj, Ic );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (iter == 0) 
      {

         #ifdef SAVE_DEBUG
            rox_log("saving debug images \n");

            sprintf(filename, "%s/debug_image_ref_pred_matse3_%03d.txt", RESULT_PATH, count);
            rox_array2d_float_save(filename, obj->image_ref);

            sprintf(filename, "%s/debug_image_war_pred_matse3_%03d.txt", RESULT_PATH, count);
            rox_array2d_float_save(filename, obj->warped_image);

            sprintf(filename, "%s/debug_imask_war_pred_matse3_%03d.txt", RESULT_PATH, count);
            rox_array2d_uint_save(filename, obj->warped_image_validity);

            count++;
         #endif

         error = rox_array2d_float_zncc ( &zncc_prev, obj->image_ref, obj->warped_image, obj->warped_image_validity);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      error = rox_array2d_double_svdinverse ( obj->iJtJ, obj->JtJ );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat ( obj->solution, obj->iJtJ, obj->Jtf );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // error = rox_array2d_double_scale_inplace ( obj->solution, -0.8 );
      // ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_update_right ( obj->pose, obj->solution_pose );
      ROX_ERROR_CHECK_TERMINATE ( error );

      obj->alpha += sol_data[6][0];
      obj->beta += sol_data[7][0];

      //ratio_pixels = ((double)obj->count_pixels)/(double)(rows * cols);

      norm_tra = sqrt(sol_data[0][0]*sol_data[0][0] + sol_data[1][0]*sol_data[1][0] + sol_data[2][0]*sol_data[2][0]);
      norm_rot = sqrt(sol_data[3][0]*sol_data[3][0] + sol_data[4][0]*sol_data[4][0] + sol_data[5][0]*sol_data[5][0]);

      if ( (norm_rot < 1e-6) && (norm_tra < 1e-6) )
      {
         break;
      }
   }


   //if (ratio_pixels < 0.15)
   //{ error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_zncc ( &zncc_cur, obj->image_ref, obj->warped_image, obj->warped_image_validity );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( (zncc_prev > zncc_cur) && (zncc_cur < 0.85) )
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->score = (zncc_cur+1.0)/2.0;

function_terminate:
#ifdef NORMALIZE
   #ifdef IMAGE_FLOAT
   #else
      rox_array2d_float_del ( &Ic );
   #endif
#endif

   return error;
}

#ifdef BOTH
Rox_ErrorCode rox_odometry_dense_depthmap_make_both (
   Rox_Odometry_Dense_DepthMap obj_left,
   Rox_Odometry_Dense_DepthMap obj_right,
#ifdef IMAGE_FLOAT
   const Rox_Array2D_Float image_left,
   const Rox_Array2D_Float image_right
#else
   const Rox_Image image_left,
   const Rox_Image image_right
#endif
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if ( !obj_left || !image_left )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !obj_right || !image_right )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** sol_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &sol_data, obj_left->solution);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rox_Double ratio_pixels = 0.0;
   Rox_Uint found = 0;

   for (Rox_Uint iter = 0; iter < obj_left->max_iters; iter++)
   {
      error = rox_odometry_dense_depthmap_prepare ( obj_left, image_left );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_odometry_dense_depthmap_prepare ( obj_right, image_right );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add ( obj_left->JtJ, obj_left->JtJ, obj_right->JtJ );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add ( obj_left->Jtf, obj_left->Jtf, obj_right->Jtf );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_svdinverse ( obj_left->iJtJ, obj_left->JtJ );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat ( obj_left->solution, obj_left->iJtJ, obj_left->Jtf );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_update_right ( obj_left->pose, obj_left->solution );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_copy ( obj_right->pose, obj_left->pose );
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Double norm_tra = sqrt(sol_data[0][0]*sol_data[0][0] + sol_data[1][0]*sol_data[1][0] + sol_data[2][0]*sol_data[2][0]);
      Rox_Double norm_rot = sqrt(sol_data[3][0]*sol_data[3][0] + sol_data[4][0]*sol_data[4][0] + sol_data[5][0]*sol_data[5][0]);

      if ( (norm_rot < 1e-5) && (norm_tra < 1e-5) )
      {
         found = 1;
         break;
      }

      Rox_Sint rows = 0, cols = 0;

      error = rox_array2d_float_get_size(&rows, &cols, obj_left->depthmap);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //ratio_pixels = ((double)obj_left->count_pixels + obj_right->count_pixels)/(double)(rows * cols);
   }

   //if (ratio_pixels < 0.3)
   //{ error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (found == 0)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

#endif