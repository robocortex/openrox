//==============================================================================
//
//    OPENROX   : File ident_photoframe_se3.c
//
//    Contents  : Implementation of ident_photoframe_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ident_photoframe_se3.h"
#include "ident_photoframe_struct.h"

#include <generated/objset_photoframe_struct.h>
#include <generated/objset_dynvec_point2d_double.h>

#include <system/memory/memory.h>
#include <system/time/timer.h>

#include <baseproc/array/fill/fillval.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>

#include <core/identification/codedframe.h>
#include <core/identification/photoframe.h>
#include <core/identification/photoframe_struct.h>
#include <core/model/model_single_plane_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#include <user/sensor/camera/camera_struct.h>

Rox_ErrorCode rox_ident_photoframe_se3_new (
   Rox_Ident_PhotoFrame_SE3 * ident_photoframe_se3,
   const Rox_Sint image_width,
   const Rox_Sint image_height
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_PhotoFrame_SE3 ret = NULL;

   if (!ident_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *ident_photoframe_se3 = NULL;

   ret = (Rox_Ident_PhotoFrame_SE3) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init internal variables
   ret->qhom               = NULL;
   ret->photoframes        = NULL;
   ret->detector           = NULL;
   ret->mask               = NULL;
   ret->orientation_method = 0; // 0 slowest but more robust
   ret->detection_method   = 0; // 0 fastest but less robust
   ret->score_threshold    = 0.9;

   // Set the quad color to 0 in order to detect white inside black quads by default
   ret->quad_color = 0;

   // Set the minimum length of a segment extracted from the image for quad detection
   ret->quad_segment_length_min = 10;

   error = rox_matsl3_new ( &ret->qhom );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_photoframe_new(&ret->photoframes, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_new(&ret->detector, image_width, image_height);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new(&ret->mask, image_height, image_width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *ident_photoframe_se3 = ret;

function_terminate:
   // Delet only if an error occurs.
   if (error) rox_ident_photoframe_se3_del(&ret);

   return error;
}

Rox_ErrorCode rox_ident_photoframe_se3_del (
   Rox_Ident_PhotoFrame_SE3 * ident_photoframe_se3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_PhotoFrame_SE3 todel = NULL;

   if (!ident_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ident_photoframe_se3;
   *ident_photoframe_se3 = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matsl3_del(&todel->qhom);
   rox_imask_del(&todel->mask);
   rox_quaddetector_del(&todel->detector);
   rox_objset_photoframe_del(&todel->photoframes);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_se3_addframe_model (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   const Rox_Model_Single_Plane model_single_plane,
   const Rox_Sint border_width
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame toadd = NULL;

   if ( !model_single_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !ident_photoframe_se3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (border_width < 1)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // error = rox_photoframe_new ( &toadd, image_model, border_width );
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_photoframe_set_size ( toadd, width_meters, height_meters );
   // ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_photoframe_new_model ( &toadd, model_single_plane, border_width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_photoframe_append ( ident_photoframe_se3->photoframes, toadd );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   // Delete only if an error occurs
   if(error) ROX_ERROR_CHECK(rox_photoframe_del(&toadd))

   return error;
}

#if 0
Rox_ErrorCode rox_ident_photoframe_se3_addframe (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   const Rox_Image image_model,
   const Rox_Double width_meters,
   const Rox_Double height_meters,
   const Rox_Sint border_width
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame toadd = NULL;

   if (!ident_photoframe_se3 || !image_model)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (border_width < 1)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_photoframe_new ( &toadd, image_model, border_width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_photoframe_set_size ( toadd, width_meters, height_meters );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_photoframe_new_model ( &toadd, model_single_plane, border_width );
   // ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_photoframe_append ( ident_photoframe_se3->photoframes, toadd );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   // Delete only if an error occurs
   if(error) ROX_ERROR_CHECK(rox_photoframe_del(&toadd))

   return error;
}
#endif


Rox_ErrorCode rox_ident_photoframe_se3_addframe_masked (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   const Rox_Model_Single_Plane model_single_plane,
   const Rox_Sint border_width,
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame toadd = NULL;

   if (!ident_photoframe_se3 || !model_single_plane)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( border_width < 1 )
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_photoframe_new_model ( &toadd, model_single_plane, border_width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_photoframe_set_mask( toadd, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_photoframe_append(ident_photoframe_se3->photoframes, toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   // Delete only if an error occurs
   if(error) ROX_ERROR_CHECK(rox_photoframe_del(&toadd))

   return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_getcountframes (
   Rox_Sint * count,
   const Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_se3 || !count)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *count = ident_photoframe_se3->photoframes->used;

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_make (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   const Rox_Camera camera
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint quad_count = 0;

   if (!ident_photoframe_se3 || !camera)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Reset frames to mark them as not detected
   for (Rox_Uint idframe = 0; idframe < ident_photoframe_se3->photoframes->used; idframe++)
   {
      error = rox_photoframe_reset ( ident_photoframe_se3->photoframes->data[idframe] );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_array2d_uint_fillval(ident_photoframe_se3->mask, ~0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_set_quad_color(ident_photoframe_se3->detector, ident_photoframe_se3->quad_color);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_set_segment_length_min(ident_photoframe_se3->detector, ident_photoframe_se3->quad_segment_length_min);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rox_Timer timer = 0;
   // Rox_Double time = 0.0;

   // error = rox_timer_new(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_start(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // Quad detection depends on selected processing method
   if ( ident_photoframe_se3->detection_method == 0 )
   {
      error = rox_quaddetector_process_image ( ident_photoframe_se3->detector, camera->image, ident_photoframe_se3->mask );
   }
   else
   {
      error = rox_quaddetector_process_image_ac ( ident_photoframe_se3->detector, camera->image, ident_photoframe_se3->mask);
   }
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_get_quad_count ( &quad_count, ident_photoframe_se3->detector );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_stop(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_get_elapsed_ms(&time, timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_log("time to detect %d quads = %f\n", quad_count, time);

   // error = rox_timer_start(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint idquad = 0; idquad < quad_count; idquad++)
   {
      // qhom contains the homography that transforms the quad basic coordinates into the quad image coordinates:
      // p_quad_image = qhom * p_quad_basic
      // p_quad_basic = [  0, cols+2*border, cols+2*border_u,               0;
      //                   0,             0, rows+2*border_v, rows+2*border_v;
      //                   1,             1,               1,               1];

      error = rox_quaddetector_get_quad_SL3 ( ident_photoframe_se3->qhom, ident_photoframe_se3->detector, idquad);
      ROX_ERROR_CHECK_CONTINUE(error);

      for (Rox_Uint idframe = 0; idframe < ident_photoframe_se3->photoframes->used; idframe++)
      {
         // rox_log("score threshold for photoframe %d = %f \n", idframe, ident_photoframe_se3->score_threshold);
         error = rox_photoframe_set_score_threshold ( ident_photoframe_se3->photoframes->data[idframe], ident_photoframe_se3->score_threshold );
         ROX_ERROR_CHECK_CONTINUE(error);

         error = rox_photoframe_make_se3 ( ident_photoframe_se3->photoframes->data[idframe], camera->image, ident_photoframe_se3->qhom, camera->calib_camera, ident_photoframe_se3->orientation_method );
         ROX_ERROR_CHECK_CONTINUE(error);
      }
   }

   // error = rox_timer_stop(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_get_elapsed_ms(&time, timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_del(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );
function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_get_result (
   Rox_Sint * is_identified,
   Rox_Double * score,
   Rox_MatSE3 pose,
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_ident_photoframe_se3_get_pose ( is_identified, pose, ident_photoframe_se3, id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_photoframe_se3_get_score ( score, ident_photoframe_se3, id );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_get_pose (
   Rox_Sint * is_identified,
   Rox_MatSE3 pose,
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_se3 || !is_identified || !pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (id > (Rox_Sint) ident_photoframe_se3->photoframes->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *is_identified = ident_photoframe_se3->photoframes->data[id]->is_detected;
   if (*is_identified)
   {
      error = rox_matse3_copy(pose, ident_photoframe_se3->photoframes->data[id]->pose);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_get_score (
   Rox_Double *score,
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_se3 || !score)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (id > (Rox_Sint) ident_photoframe_se3->photoframes->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *score = ident_photoframe_se3->photoframes->data[id]->score;

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_set_orientation_method (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   Rox_Sint orientation_method
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!ident_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (orientation_method > 1)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ident_photoframe_se3->orientation_method = orientation_method;

function_terminate:
  return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_set_detection_method (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe,
   Rox_Sint detection_method
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (detection_method > 1)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ident_photoframe->detection_method = detection_method;

function_terminate:
  return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_set_type (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   Rox_Sint black_to_white
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!ident_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (black_to_white > 1)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ident_photoframe_se3->quad_color = black_to_white;

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_set_segment_length_min (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   const Rox_Double segment_length_min
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!ident_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // if (segment_length_min < 0.0)
   // { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ident_photoframe_se3->quad_segment_length_min = segment_length_min;

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_set_score_threshold (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   Rox_Double score_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_se3)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if ( 0.0 > score_threshold || 1.0 < score_threshold )
   {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   ident_photoframe_se3->score_threshold = score_threshold;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_se3_set_side_bounds (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   Rox_Double side_min,
   Rox_Double side_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_quaddetector_set_side_bounds ( ident_photoframe_se3->detector, side_min, side_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_photoframe_se3_set_area_bounds (
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   Rox_Double area_min,
   Rox_Double area_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_quaddetector_set_area_bounds ( ident_photoframe_se3->detector, area_min, area_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_photoframe_se3_get_code (
   Rox_Sint * is_identified,
   Rox_Sint * code_number,
   Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   Rox_Camera camera,
   Rox_Sint code_bits,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CodedFrame codedframe = NULL;
   Rox_MatSE3 T = NULL;

   if (!ident_photoframe_se3 || !camera || !is_identified || !code_number)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *code_number = 0;

   error = rox_matse3_new(&T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_photoframe_se3_get_pose(is_identified, T, ident_photoframe_se3, id);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_codedframe_new(&codedframe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (is_identified)
   {
      Rox_MatSL3 H = ident_photoframe_se3->photoframes->data[id]->resH;

      if (code_bits == 16)
      {
         error = rox_codedframe_make16(codedframe, camera->image, H);
         ROX_ERROR_CHECK_TERMINATE ( error );

         *code_number = codedframe->value;
      }

      if (code_bits == 64)
      {
         error = rox_codedframe_make64(codedframe, camera->image, H);
         ROX_ERROR_CHECK_TERMINATE ( error );

         *code_number = codedframe->value;
      }
   }

function_terminate:
   rox_matse3_del(&T);
   rox_codedframe_del(&codedframe);
   return error;
}

// Get the best score
Rox_ErrorCode rox_ident_photoframe_se3_get_best_result (
   Rox_Sint * identified,
   Rox_Sint * best_photoframe_id,
   Rox_Double * best_score,
   Rox_MatSE3 best_pose,
   const Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 pose = NULL;

   if ( !ident_photoframe_se3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !identified || !best_photoframe_id || !best_score || !best_pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_new(&pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint nb_frames = 0;
   error = rox_ident_photoframe_se3_getcountframes ( &nb_frames, ident_photoframe_se3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_log("number of frames = %d\n",nb_frames);

   *identified = 0;
   *best_score = -1.0;
   *best_photoframe_id = -1;
   for (Rox_Sint k=0; k<nb_frames; k++)
   {
      Rox_Double score = 0.0;
      Rox_Sint identified_k = 0;

      error = rox_ident_photoframe_se3_get_result ( &identified_k, &score, pose, ident_photoframe_se3, k );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ( identified_k )
      {
         *identified = 1;
         if (score > *best_score)
         {
            *best_photoframe_id = k;
            *best_score = score;
            error = rox_matse3_copy ( best_pose, pose );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
      }

   }

function_terminate:
   rox_matse3_del(&pose);
   return error;
}

#ifdef get_results_multi_ident
// Get the score after vvs
Rox_ErrorCode rox_ident_photoframe_se3_get_vvs_result (
   Rox_Sint * identified,
   Rox_Sint * vvs_photoframe_id,
   Rox_Double * vvs_score,
   Rox_MatSE3 vvs_pose,
   const Rox_Ident_PhotoFrame_SE3 ident_photoframe_se3,
   const Rox_MatUT3 Kc
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 cTo = NULL;

   if ( !ident_photoframe_se3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !identified || !vvs_photoframe_id || !vvs_score || !vvs_pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_new ( &cTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint nb_frames = 0;
   error = rox_ident_photoframe_se3_getcountframes ( &nb_frames, ident_photoframe_se3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_log("number of frames = %d\n",nb_frames);

   Rox_ObjSet_DynVec_Point2D_Double pr = NULL;  // Should be a dynvec since there are many possible planes

   error = rox_objset_dynvec_point2d_double_new( &pr, nb_frames );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_ObjSet_DynVec_Point3D_Double mb = NULL;  // Should be a dynvec since there are many possible planes

   // Define the models of the points on the planes
   error = rox_objset_dynvec_point3d_double_new ( &mb, nb_frames );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_ObjSet_MatSE3 oTb = NULL;                // Should be an objset since there are many possible planes

   error = rox_objset_matse3_new( &oTb, nb_frames );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *identified = 0;
   *vvs_score = -1.0;
   *vvs_photoframe_id = -1;
   for (Rox_Sint k=0; k < nb_frames; k++)
   {
      Rox_Double score = 0.0;
      Rox_Sint identified_k = 0;

      error = rox_ident_photoframe_se3_get_result ( &identified_k, &score, cTo, ident_photoframe_se3, k );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ( identified_k )
      {
         Rox_Double depths[4];

         Rox_Double model_sizex = ident_photoframe_se3->photoframes->data[k]->width_meters;
         Rox_Double model_sizey = ident_photoframe_se3->photoframes->data[k]->height_meters;

         Rox_Point3D_Double_Struct mo[4];

         mo[0].X = -model_sizex / 2.0;
         mo[0].Y = -model_sizey / 2.0;
         mo[0].Z = 0.0;

         mo[1].X = +model_sizex / 2.0;
         mo[1].Y = -model_sizey / 2.0;
         mo[1].Z = 0.0;

         mo[2].X = +model_sizex / 2.0;
         mo[2].Y = +model_sizey / 2.0;
         mo[2].Z = 0.0;

         mo[3].X = -model_sizex / 2.0;
         mo[3].Y = +model_sizey / 2.0;
         mo[3].Z = 0.0;


         Rox_DynVec_Point3D_Double mb_k = NULL;

         error = rox_dynvec_point3d_double_new ( &mb_k, 4);
         ROX_ERROR_CHECK_TERMINATE ( error );
      
         error = rox_dynvec_point3d_double_append_rectangle ( mb_k, model_sizex, model_sizey );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Append set of points mb_0 to objset mb
         error = rox_objset_dynvec_point3d_double_append( mb, mb_k ); 
         ROX_ERROR_CHECK_TERMINATE ( error );


         Rox_DynVec_Point2D_Double pr_k = NULL;

         error = rox_dynvec_point2d_double_new(&pr_k, 4);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_point2d_double_transform_project ( pr_k->data, depths, Kc, cTo, mo, 4 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_dynvec_point2d_double_append( pr, pr_k ); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_MatSE3 oTb_k = NULL;

         error = rox_matse3_new ( &oTb_k );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_matse3_append( oTb, oTb_k );
         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_log("Adding photoframe id %d, with score = %f \n", k, score); 

         if (score > *best_score)
         {
            *best_photoframe_id = k;
            *best_score = score;
            error = rox_matse3_copy ( best_pose, pose );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
      }

   }

   error = rox_vvs_points_pix_se3_so3z_r2 ( vvs_pose, oTb, K, pr, mb );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matse3_del ( &cTo );
   return error;
}

#endif