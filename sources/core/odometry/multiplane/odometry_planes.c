//============================================================================
//
//    OPENROX   : File odometry_planes.c
//
//    Contents  : Implementation of odometry_planes module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "odometry_planes.h"

#include <generated/objset_patchplane_pyramid_struct.h>

#include <baseproc/array/conversion/array2d_float_from_uchar.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/geometry/transforms/matsl3/sl3normalize.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/pixelgrid/warp_grid_matsl3.h>
#include <baseproc/image/imask/imask.h>

#include <core/patch/patchplane.h>
#include <baseproc/calculus/linsys/linsys_se3_light_affine_premul_left.h>
#include <baseproc/calculus/linsys/linsys_se3_z1_light_affine_premul_left.h>
#include <core/templatesearch/region_zncc_search_mask_template_mask.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/numeric/array2d_print.h>
#include <inout/numeric/array2d_save.h>

#define CONV_THRESH 1e-4
#define CONV_THRESH_VT 1e-6 // Default threshold for convergence of translation 
#define CONV_THRESH_VR 1e-5 // Default threshold for convergence of rotation

//#define POSE_SHIFT_TZ1 0 // NEW STYLE
#define POSE_SHIFT_TZ1 1 // OLD STYLE

// static int count_iter = 0;

Rox_ErrorCode rox_odometry_planes_new (
   Rox_Odometry_Planes * odometry_planes,
   const Rox_Model_Multi_Plane model_multi_plane
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Planes ret = NULL;
   Rox_PatchPlane_Pyramid pyramid = NULL;
   Rox_Array2D_Float ref_buffer = NULL;

   if ( !odometry_planes || !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (model_multi_plane->planes->used == 0)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *odometry_planes = NULL;

   ret = ( Rox_Odometry_Planes ) rox_memory_allocate( sizeof( struct Rox_Odometry_Planes_Struct ), 1 );

   if ( !ret )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pyramid        = NULL;
   ref_buffer     = NULL;
   ret->min_level = 1000;
   ret->predicter = NULL;

   // Reset current score to 0
   ret->score = 0.0;

   ret->pose         = NULL;
   error = rox_matse3_new ( &ret->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->calib_camera = NULL;
   error = rox_array2d_double_new ( &ret->calib_camera, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit ( ret->calib_camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->calib_zoom = NULL;
   error = rox_array2d_double_new ( &ret->calib_zoom, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->homography   = NULL;
   error = rox_array2d_double_new( &ret->homography, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->localJtJ     = NULL;
   error = rox_array2d_double_new ( &ret->localJtJ, 8, 8 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->localJtf     = NULL;
   error = rox_array2d_double_new ( &ret->localJtf, 8, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->patches      = NULL;
   error = rox_objset_patchplane_pyramid_new ( &ret->patches, 5 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Add reference images
   for ( Rox_Uint id = 0; id < model_multi_plane->planes->used; id++ )
   {
      Rox_Sint width = 0, height = 0;
      error = rox_array2d_uchar_get_size ( &height, &width, model_multi_plane->planes->data[id]->image_template );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Convert image
      error = rox_array2d_float_new ( &ref_buffer, height, width );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_float_from_uchar_normalize ( ref_buffer, model_multi_plane->planes->data[id]->image_template );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Create patch
      error = rox_patchplane_pyramid_new ( &pyramid, height, width );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Min level
      if ( pyramid->count < ret->min_level ) ret->min_level = pyramid->count;

      // Set patch reference information
      error = rox_patchplane_pyramid_apply ( pyramid, ref_buffer, model_multi_plane->planes->data[id]->mask );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Add patch to set
      error = rox_objset_patchplane_pyramid_append ( ret->patches, pyramid );
      ROX_ERROR_CHECK_TERMINATE ( error );

      rox_array2d_float_del( &ref_buffer );
      pyramid = NULL;
   }

   ret->prediction_radius = 16;
   ret->max_iterations = 10;
   ret->score_threshold = 0.89;

   // Init prediction
   // Allocate one predicter for each template model
   ret->predicter = (Rox_Plane_Search *) rox_memory_allocate ( sizeof(struct Rox_Plane_Search_Struct), model_multi_plane->planes->used);

   for ( Rox_Uint id = 0; id < model_multi_plane->planes->used; id++ )
   {
      Rox_PatchPlane lastlevel = ret->patches->data[id]->levels[ret->min_level-1];

      Rox_Sint sheight = 0, swidth = 0;
      error = rox_array2d_float_get_size ( &sheight, &swidth, lastlevel->reference);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // prediction_radius should be a parameter
      error = rox_plane_search_new ( &ret->predicter[id], sheight, swidth, ret->prediction_radius);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_plane_search_set_model ( ret->predicter[id], lastlevel->reference, lastlevel->reference_mask);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *odometry_planes = ret;
   error = ROX_ERROR_NONE;

function_terminate:
   if ( error )
   {
      rox_array2d_float_del ( &ref_buffer );
      rox_patchplane_pyramid_del ( &pyramid );
      rox_odometry_planes_del ( &ret );
   }

   return error;
}


Rox_ErrorCode rox_odometry_planes_del (
   Rox_Odometry_Planes * odometry_planes
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Planes todel = NULL;

   if ( !odometry_planes )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *odometry_planes;
   *odometry_planes = NULL;

   if ( !todel )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matse3_del ( &todel->pose );
   rox_matut3_del ( &todel->calib_camera );
   rox_matut3_del ( &todel->calib_zoom );
   rox_matsl3_del ( &todel->homography );
   rox_matrix_del ( &todel->localJtJ );
   rox_matrix_del ( &todel->localJtf );

   for ( Rox_Uint id = 0; id < todel->patches->used; id++ )
   {
      rox_plane_search_del ( &todel->predicter[id] );
   }
   rox_memory_delete ( todel->predicter );

   rox_objset_patchplane_pyramid_del ( &todel->patches );

   rox_memory_delete( todel );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_planes_optim_esm (
   Rox_Odometry_Planes odometry_planes,
   const Rox_Model_Multi_Plane model_multi_plane,
   const Rox_Array2D_Float source,
   const Rox_Sint level
) 
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // ------------------------------------------------------------------------------------
   // Optimization
   Rox_Sint count_visibles = 0;

   Rox_Double norm_vt = 0.0;
   Rox_Double norm_vr = 0.0;

   Rox_Array2D_Double iLtL = NULL;
   Rox_Array2D_Double LtL = NULL;
   Rox_Array2D_Double Lte = NULL;

   Rox_Array2D_Double sol = NULL;
   Rox_Array2D_Double subsol = NULL;
   Rox_Array2D_Double subsol_vt = NULL;
   Rox_Array2D_Double subsol_vr = NULL;

   // Compute number of visible patches, this can be done before optimisation because
   // we suppose the minimization will not severely impact visibility of planes
   Rox_Sint * visibility_list = (Rox_Sint *) rox_memory_allocate ( sizeof(Rox_Sint), model_multi_plane->planes->used );

   count_visibles = 0;
   for ( Rox_Uint idpatch = 0; idpatch < model_multi_plane->planes->used; idpatch++ )
   {
      visibility_list[idpatch] = 0;
      if ( model_multi_plane->planes->data[idpatch]->is_potentially_visible == 0 ) continue;
      visibility_list[idpatch] = 1;
      count_visibles++;
   }

   if (count_visibles == 0)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Create buffer for global jacobian for all visible planes

   error = rox_array2d_double_new ( &LtL, 7 + count_visibles, 7 + count_visibles );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &iLtL, 7 + count_visibles, 7 + count_visibles );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &Lte, 7 + count_visibles, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &sol, 7 + count_visibles, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** local_LtL_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &local_LtL_data, odometry_planes->localJtJ );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** local_Lte_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &local_Lte_data,  odometry_planes->localJtf );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** LtL_data = NULL;
   error  = rox_array2d_double_get_data_pointer_to_pointer ( &LtL_data,  LtL );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Lte_data = NULL;
   error  = rox_array2d_double_get_data_pointer_to_pointer ( &Lte_data,  Lte );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Descent
   for ( Rox_Uint iter = 0; iter < odometry_planes->max_iterations; iter++ )
   {
      Rox_Sint current_pos = 0;

      // Clean up jacobian buffers for accumulation (use set zeros for perfomance issues ???)
      error  = rox_array2d_double_fillval ( LtL, 0 );
      ROX_ERROR_CHECK_TERMINATE ( error );
         
      error  = rox_array2d_double_fillval ( Lte, 0 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Loop over patches to fill jacobian
      for ( Rox_Uint idpatch = 0; idpatch < model_multi_plane->planes->used; idpatch++ )
      {
         // Avoid hidden surfaces
         if ( model_multi_plane->planes->data[idpatch]->is_potentially_visible == 0 ) continue;

         Rox_PatchPlane_Pyramid pyramid = odometry_planes->patches->data[idpatch];
         Rox_PatchPlane patch = pyramid->levels[level];
         Rox_Model_Single_Plane curmodel = model_multi_plane->planes->data[idpatch];

         // Set the zoom calibration matrix for the given level
         error = rox_transformtools_matrix33_left_pyramidzoom ( odometry_planes->calib_zoom, curmodel->calibration_template, level );
         ROX_ERROR_CHECK_TERMINATE ( error );

         if ( POSE_SHIFT_TZ1 == 1 )
         {
            // Build homography for current view/patch
            error = rox_transformtools_build_homography ( odometry_planes->homography, curmodel->c_T_z1, odometry_planes->calib_camera, odometry_planes->calib_zoom, 0, 0, 1, -1 );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
         else
         {
            // New code to replace rox_transformtools_build_homography function

            Rox_MatUT3 Kc    = odometry_planes->calib_camera;
            Rox_MatSE3 c_T_o = curmodel->c_T_z0;
            Rox_MatSL3 t_G_o = odometry_planes->calib_zoom;
            Rox_MatSL3 c_G_t = odometry_planes->homography;
            Rox_MatSL3 c_G_o = NULL; // should be permanently stored in the odometry object ?

            error = rox_matsl3_new ( &c_G_o ) ;
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_transformtools_build_model_to_image_homography ( c_G_o, Kc, c_T_o );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
            ROX_ERROR_CHECK_TERMINATE ( error );

            rox_matsl3_del ( &c_G_o );
         }

         // Compute jacobian for this patch only
         error = rox_patchplane_prepare_sl3 ( patch, odometry_planes->homography, source );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_patchplane_prepare_finish ( patch );
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Sint valid_pixels = 0;
         error = rox_imask_count_valid ( &valid_pixels, patch->current_mask );
         ROX_ERROR_CHECK_TERMINATE ( error );

         if (valid_pixels == 0)
         {
            visibility_list[idpatch] = 0;
            continue;
         }

         // localJtJ: from 0 to 5 is the pose, 6 is beta, from 7 to 7 + nbpatch is alpha
         if ( POSE_SHIFT_TZ1 == 1)
         {
            error = rox_jacobian_se3_z1_light_affine_premul_left ( odometry_planes->localJtJ,
                                                                      odometry_planes->localJtf,
                                                                      patch->gx,
                                                                      patch->gy,
                                                                      patch->mean,
                                                                      patch->difference,
                                                                      patch->gradient_mask,
                                                                      curmodel->c_T_z1,
                                                                      odometry_planes->calib_zoom );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
         else
         {
            error = rox_jacobian_se3_light_affine_premul_left ( odometry_planes->localJtJ,
                                                                  odometry_planes->localJtf,
                                                                   patch->gx,
                                                                   patch->gy,
                                                                   patch->mean,
                                                                   patch->difference,
                                                                   patch->gradient_mask,
                                                                   curmodel->c_T_z0,
                                                                   odometry_planes->calib_zoom );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }

         // Append per patch jacobian to global jacobian ( same pose, same beta but different alpha )
         for ( Rox_Sint k = 0; k < 6; k++ )
         {
            for ( Rox_Sint l = 0; l < 6; l++ )
            {
               LtL_data[k][l] += local_LtL_data[k][l];
            }

            Lte_data[k][0] += local_Lte_data[k][0];
         }

           for ( Rox_Sint k = 0; k < 6; k++ )
            {
               LtL_data[ 6               ][ k               ] += local_LtL_data[ 7 ][ k ];
               LtL_data[ k               ][ 6               ] += local_LtL_data[ 7 ][ k ];
               LtL_data[ 7 + current_pos ][ k               ] += local_LtL_data[ 6 ][ k ];
               LtL_data[ k               ][ 7 + current_pos ] += local_LtL_data[ 6 ][ k ];
            }

            // ( We inverse alpha and beta order so that the common beta come first, the the alpha of each plane
            LtL_data[ 7 + current_pos ][ 7 + current_pos ] += local_LtL_data[ 6 ][ 6 ];
            LtL_data[ 7 + current_pos ][ 6               ] += local_LtL_data[ 6 ][ 7 ];
            LtL_data[ 6               ][ 7 + current_pos ] += local_LtL_data[ 7 ][ 6 ];
            LtL_data[ 6               ][ 6               ] += local_LtL_data[ 7 ][ 7 ];
            Lte_data[ 7 + current_pos ][ 0               ] += local_Lte_data[ 6 ][ 0 ];
            Lte_data[ 6               ][ 0               ] += local_Lte_data[ 7 ][ 0 ];

            current_pos++;
         }

         // create submatrix of appropriate size depending on visible patches

         Rox_Array2D_Double iLtL_sub = NULL;
         error = rox_array2d_double_new_subarray2d (&iLtL_sub, iLtL, 0, 0, 7+current_pos, 7+current_pos);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Array2D_Double LtL_sub = NULL;
         error = rox_array2d_double_new_subarray2d (&LtL_sub, LtL, 0, 0, 7+current_pos, 7+current_pos);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Array2D_Double Lte_sub = NULL;
         error = rox_array2d_double_new_subarray2d (&Lte_sub, Lte, 0, 0, 7+current_pos, 1);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Array2D_Double sol_sub = NULL;
         error = rox_array2d_double_new_subarray2d (&sol_sub, sol, 0, 0, 7+current_pos, 1);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Double ** dsol = NULL;
         error = rox_array2d_double_get_data_pointer_to_pointer( &dsol, sol_sub );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_new_subarray2d( &subsol, sol_sub, 0, 0, 6, 1 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_new_subarray2d( &subsol_vt, sol_sub, 0, 0, 3, 1 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_new_subarray2d( &subsol_vr, sol_sub, 3, 0, 3, 1 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Update solution to the left ( reference frame is different for each patch, impossible to update to the right )
         error = rox_array2d_double_svdinverse( iLtL_sub, LtL_sub );
         if (error == ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE)
         {
            //rox_log("ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE \n");
         }
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_mulmatmat( sol_sub, iLtL_sub, Lte_sub );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // error = rox_array2d_double_scale_inplace ( subsol, -1.0 );
         // ROX_ERROR_CHECK_TERMINATE( error );

         error = rox_matse3_update_left ( odometry_planes->pose, subsol );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Update model pose
         error = rox_model_multi_plane_set_currentpose ( model_multi_plane, odometry_planes->pose );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Update patch lighting parameters
         current_pos = 0;
         for ( Rox_Uint idpatch = 0; idpatch < model_multi_plane->planes->used; idpatch++ )
         {
            if ( model_multi_plane->planes->data[idpatch]->is_potentially_visible == 0 ) continue;

            if ( visibility_list[idpatch] == 0) { continue; }
            Rox_PatchPlane_Pyramid pyramid  = odometry_planes->patches->data[idpatch];
            pyramid->levels[level]->beta  += (Rox_Float) dsol[ 6               ][ 0 ];
            pyramid->levels[level]->alpha += (Rox_Float) dsol[ 7 + current_pos ][ 0 ];
            current_pos++;
         }

         // Convergence test
         //error = rox_array2d_double_norm2sq ( &norm, subsol );
         // ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_norm2sq ( &norm_vt, subsol_vt );
         ROX_ERROR_CHECK_TERMINATE ( error );
         
         error = rox_array2d_double_norm2sq ( &norm_vr, subsol_vr );
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_matrix_del ( &subsol    );
         rox_matrix_del ( &subsol_vt );
         rox_matrix_del ( &subsol_vr );

         rox_matrix_del ( &sol_sub  );
         rox_matrix_del ( &iLtL_sub );
         rox_matrix_del ( &Lte_sub  );
         rox_matrix_del ( &LtL_sub  );

      // count_iter++;

      if (( norm_vt < CONV_THRESH_VT ) && ( norm_vr < CONV_THRESH_VR ))
      {
         break;
      }
   }


function_terminate:

   rox_matrix_del( &iLtL );
   rox_matrix_del( &LtL );
   rox_matrix_del( &Lte );
   rox_matrix_del( &sol );
   rox_matrix_del( &subsol );
   rox_matrix_del( &subsol_vt );
   rox_matrix_del( &subsol_vr );

   rox_memory_delete(visibility_list);

   return error;

}

Rox_ErrorCode rox_odometry_planes_prediction ( 
   Rox_MatSE3 best_pose, 
   Rox_Double * best_score,
   const Rox_Odometry_Planes odometry_planes,
   const Rox_Model_Multi_Plane model_multi_plane, 
   const Rox_Array2D_Float source,
   const Rox_MatSE3 init_pose,
   const Rox_Sint level
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 temp_pose = NULL;

   error = rox_matse3_new ( &temp_pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint patchid = 0; patchid < odometry_planes->patches->used; patchid++ )
   {
      Rox_Model_Single_Plane curmodel = model_multi_plane->planes->data[patchid];

      // Skip not visible planes
      if ( model_multi_plane->planes->data[patchid]->is_potentially_visible == 0 ) continue;

      error = rox_matse3_copy ( temp_pose, init_pose );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Set the zoom calibration matrix for the given level
      error = rox_transformtools_matrix33_left_pyramidzoom( odometry_planes->calib_zoom, curmodel->calibration_template, level );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (POSE_SHIFT_TZ1 == 1)
      {
         error = rox_transformtools_build_homography(odometry_planes->homography, temp_pose, odometry_planes->calib_camera, odometry_planes->calib_zoom, 0, 0, -1, 1);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         // New code to replace rox_transformtools_build_homography function

         Rox_MatUT3 Kc    = odometry_planes->calib_camera;
         Rox_MatSE3 c_T_o = temp_pose;
         Rox_MatSL3 t_G_o = odometry_planes->calib_zoom;
         Rox_MatSL3 c_G_t = odometry_planes->homography;
         Rox_MatSL3 c_G_o = NULL; // should be permanently stored in the odometry object ?

         error = rox_matsl3_new ( &c_G_o ) ;
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_transformtools_build_model_to_image_homography ( c_G_o, Kc, c_T_o );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_matsl3_del ( &c_G_o );
      }

      error = rox_plane_search_make ( odometry_planes->predicter[patchid], source, odometry_planes->homography );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Display search results
      Rox_Double score = 0.0, tu = 0.0, tv = 0.0;

      error = rox_plane_search_get_results ( &score, &tu, &tv, odometry_planes->predicter[patchid] );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ( score > *best_score )
      {
         *best_score = score;

         error = rox_plane_search_update_pose_translation ( temp_pose, odometry_planes->calib_zoom, odometry_planes->predicter[patchid]);
         ROX_ERROR_CHECK_TERMINATE ( error );

         if (POSE_SHIFT_TZ1 == 1)
         {
            // Update pose to change Z of reference plane for prediction
            error = rox_transformtools_updateZref ( best_pose, temp_pose, 1 );
            ROX_ERROR_CHECK_TERMINATE ( error );

         }
         else
         {
            error = rox_matse3_copy ( best_pose, temp_pose );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
      }
   }

function_terminate:
   rox_matse3_del ( &temp_pose );
   return error;
}


Rox_ErrorCode rox_odometry_planes_make (
   Rox_Odometry_Planes odometry_planes,
   const Rox_Model_Multi_Plane model_multi_plane,
   const Rox_Array2D_Float source
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double score = 0.0;
   Rox_Sint countvalid = 0;

   Rox_MatSE3 pred_pose = NULL;
   Rox_MatSE3 init_pose = NULL;
   Rox_MatSE3 best_pose = NULL;

   // Test Inputs
   if ( !model_multi_plane || !source )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Test Outputs
   if ( !odometry_planes )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Reset current score to 0
   odometry_planes->score = 0.0;

   error = rox_matse3_new ( &pred_pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &init_pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &best_pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_matse3_print(odometry_planes->pose);

   // 
   Rox_Sint image_rows = 0, image_cols = 0;
   error = rox_array2d_float_get_size ( &image_rows, &image_cols, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Transform model given predicted pose ( to get visibility constraints )
   error = rox_model_multi_plane_set_currentpose ( model_multi_plane, odometry_planes->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Transform the model according the current pose setted with function rox_model_multi_plane_set_currentpose
   error = rox_model_multi_plane_transform ( model_multi_plane );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check visible planes  
   error = rox_model_multi_plane_check_visibility ( model_multi_plane, image_rows, image_cols, odometry_planes->calib_camera, odometry_planes->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_copy ( init_pose, odometry_planes->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Save a copy of the pose
   if ( POSE_SHIFT_TZ1 == 1 )
   {

      // Update pose to change Z of reference plane for prediction
      error = rox_transformtools_updateZref ( pred_pose, init_pose, -1 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_copy ( best_pose, init_pose );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {

      error = rox_matse3_copy ( pred_pose, init_pose );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_copy ( best_pose, init_pose );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Begin tracking here
   // From the coarsest to the fined level, repeat the same operations to achieve better convergence
   for ( Rox_Sint level = odometry_planes->min_level - 1; level >= 0; level-- )
   {
      Rox_Double best_score = 0;

      // If we are ate the initial level in the pyramid make a prediction
      if (level == odometry_planes->min_level - 1)
      {
         error = rox_odometry_planes_prediction ( best_pose, &best_score, odometry_planes, model_multi_plane, source, pred_pose, level );
         ROX_ERROR_CHECK_TERMINATE ( error );

         if ( best_score > odometry_planes->score_threshold )
         {
            error = rox_matse3_copy ( odometry_planes->pose, best_pose );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
         else
         {
            //rox_log("NOT setting the predicted pose since the best score is too low\n");
         }

      }

      // Update model pose
      error = rox_model_multi_plane_set_currentpose ( model_multi_plane, odometry_planes->pose );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_odometry_planes_optim_esm ( odometry_planes, model_multi_plane, source, level ); //, max_iters );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_model_multi_plane_set_currentpose ( model_multi_plane, odometry_planes->pose );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compare reference and current for each patch
      countvalid = 0;
      odometry_planes->score = 0.0;
      for ( Rox_Uint idpatch = 0; idpatch < model_multi_plane->planes->used; idpatch++ )
      {
         if ( model_multi_plane->planes->data[idpatch]->is_potentially_visible == 0 ) continue;

         Rox_PatchPlane_Pyramid pyramid = odometry_planes->patches->data[idpatch];
         Rox_PatchPlane patch = pyramid->levels[level];

         // Reproject current image on reference space
         error = rox_transformtools_matrix33_left_pyramidzoom ( odometry_planes->calib_zoom, model_multi_plane->planes->data[idpatch]->calibration_template, level );
         ROX_ERROR_CHECK_TERMINATE ( error );


         if(POSE_SHIFT_TZ1 == 1)
         {
            error = rox_transformtools_build_homography ( odometry_planes->homography,
                                                      model_multi_plane->planes->data[idpatch]->c_T_z1,
                                                      odometry_planes->calib_camera,
                                                      odometry_planes->calib_zoom,
                                                      0, 0, 1, -1 );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
         else
         {
            // New code to replace rox_transformtools_build_homography function

            Rox_MatUT3 Kc    = odometry_planes->calib_camera;
            Rox_MatSE3 c_T_o = model_multi_plane->planes->data[idpatch]->c_T_z0;
            Rox_MatSL3 t_G_o = odometry_planes->calib_zoom;
            Rox_MatSL3 c_G_t = odometry_planes->homography;
            Rox_MatSL3 c_G_o = NULL; // should be permanently stored in the odometry object ?

            error = rox_matsl3_new ( &c_G_o ) ;
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_transformtools_build_model_to_image_homography ( c_G_o, Kc, c_T_o );
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_matsl3_mulmatinv ( c_G_t, c_G_o, t_G_o );
            ROX_ERROR_CHECK_TERMINATE ( error );

            rox_matsl3_del ( &c_G_o );
         }

         error = rox_patchplane_prepare_sl3 ( patch, odometry_planes->homography, source );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_patchplane_prepare_finish ( patch );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Compute normalized ZNCC score between 0 and 1
         error = rox_patchplane_compute_score ( &score, patch );
         ROX_ERROR_CHECK_TERMINATE ( error );


         // If current is similar to reference, consider it valid and increment valid counter
         if ( score > odometry_planes->score_threshold )
         {
            countvalid++;
            odometry_planes->score += score;
         }
         else
         {
            // Do not consider the plane if the score is too low
            model_multi_plane->planes->data[idpatch]->is_potentially_visible = 0;
         }
      }

      // If no patch is good, consider the odometry to have failed
      if ( level > 1 && countvalid == 0 )
      {
         odometry_planes->score = 0.0;

         // We're in a high level keep the best pose
         error = rox_matse3_copy ( odometry_planes->pose, init_pose );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Check visible planes again  
         error = rox_model_multi_plane_check_visibility ( model_multi_plane, image_rows, image_cols, odometry_planes->calib_camera, odometry_planes->pose );
         ROX_ERROR_CHECK_TERMINATE ( error );

      }
      else
      {
         if (countvalid == 0)
         {
            odometry_planes->score = 0.0;
            // We're at level 0, the algorithm failed
            error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
         else
         {
            odometry_planes->score /= (double) countvalid;
         }
         // The score id already normalized between 0 and 1
         // odometry_planes->score = ( odometry_planes->score + 1.0 ) * 0.5;
      }

   }

function_terminate:

   rox_matse3_del ( &pred_pose );
   rox_matse3_del ( &init_pose );
   rox_matse3_del ( &best_pose );

   return error;
}


Rox_ErrorCode rox_odometry_planes_get_pose (
   Rox_MatSE3 pose,
   const Rox_Odometry_Planes odometry_planes
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !odometry_planes || !pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy( pose, odometry_planes->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_planes_get_score (
   Rox_Double * score,
   const Rox_Odometry_Planes odometry_planes
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( NULL == odometry_planes || NULL == score )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *score = odometry_planes->score;

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_planes_get_result (
   Rox_Sint   * is_tracked,
   Rox_Double * score,
   Rox_MatSE3 pose,
   const Rox_Odometry_Planes odometry_planes
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !odometry_planes || !pose || !score || !is_tracked)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_odometry_planes_get_pose ( pose, odometry_planes );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_odometry_planes_get_score ( score, odometry_planes );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if( *score > odometry_planes->score_threshold )
   {
      *is_tracked = 1;
   }
   else
   {
      *is_tracked = 0;
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_planes_set_pose (
   Rox_Odometry_Planes odometry_planes,
   const Rox_Array2D_Double pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !odometry_planes || !pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy( odometry_planes->pose, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_odometry_planes_set_camera_calibration (
   Rox_Odometry_Planes odometry_planes,
   const Rox_Array2D_Double calibration_camera )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if( !odometry_planes || !calibration_camera )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy( odometry_planes->calib_camera, calibration_camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
