//==============================================================================
//
//    OPENROX   : File ident_multi_photoframe_se3.c
//
//    Contents  : Implementation of ident_multi_photoframe_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ident_multi_photoframe_se3.h"
#include "ident_multi_photoframe_struct.h"

#include <generated/objset_photoframe_struct.h>
#include <generated/objset_matse3_struct.h>
#include <generated/objset_dynvec_point2d_double.h>
#include <generated/objset_dynvec_point3d_double.h>
#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>

#include <system/memory/memory.h>
#include <system/time/timer.h>

#include <baseproc/array/fill/fillval.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/dynvec_point2d_tools.h>
#include <baseproc/geometry/point/dynvec_point3d_tools.h>

#include <core/identification/codedframe.h>
#include <core/identification/photoframe.h>
#include <core/identification/photoframe_struct.h>
#include <core/model/model_single_plane_struct.h>
#include <core/indirect/euclidean/vvspointsse3.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#include <user/sensor/camera/camera_struct.h>

Rox_ErrorCode rox_ident_multi_photoframe_se3_new (
   Rox_Ident_Multi_PhotoFrame_SE3 * ident_multi_photoframe_se3,
   const Rox_Sint image_width,
   const Rox_Sint image_height
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Multi_PhotoFrame_SE3 ret = NULL;
   
   if (!ident_multi_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *ident_multi_photoframe_se3 = NULL;

   ret = (Rox_Ident_Multi_PhotoFrame_SE3) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init internal variables
   ret->qhom               = NULL;
   ret->photoframes        = NULL;
   ret->detector           = NULL;
   ret->mask               = NULL;
   ret->pTo                = NULL;
   ret->cTo                = NULL;
   ret->orientation_method =    0; // 0 slowest but more robust
   ret->detection_method   =    0; // 0 fastest but less robust
   ret->score_threshold    =  0.9;
   ret->score              =  0.0;

   // Set the quad color to 0 in order to detect white inside black quads by default
   ret->quad_color = 0;

   // Set the minimum length of a segment extracted from the image for quad detection
   ret->quad_segment_length_min = 10;

   error = rox_matsl3_new ( &ret->qhom );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_photoframe_new ( &ret->photoframes, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_matse3_new ( &ret->pTo, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &ret->cTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_new ( &ret->detector, image_width, image_height );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_new ( &ret->mask, image_height, image_width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *ident_multi_photoframe_se3 = ret;

function_terminate:
   // Delet only if an error occurs.
   if (error) rox_ident_multi_photoframe_se3_del ( &ret );

   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_del (
   Rox_Ident_Multi_PhotoFrame_SE3 * ident_multi_photoframe_se3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ident_Multi_PhotoFrame_SE3 todel = NULL;

   if (!ident_multi_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ident_multi_photoframe_se3;
   *ident_multi_photoframe_se3 = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matsl3_del(&todel->qhom);
   rox_imask_del(&todel->mask);
   rox_quaddetector_del(&todel->detector);
   rox_objset_photoframe_del(&todel->photoframes);
   rox_objset_matse3_del(&todel->pTo);
   rox_matse3_del(&todel->cTo);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_add_model (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   const Rox_Model_Multi_Plane model_multi_plane,
   const Rox_Sint border_width
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame toadd = NULL;

   if ( !model_multi_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !ident_multi_photoframe_se3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (border_width < 1)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint nb_planes = model_multi_plane->planes->used; 

   for ( Rox_Sint k = 0; k < nb_planes; k++)
   {
      Rox_Model_Single_Plane model_single_plane = NULL;
      Rox_MatSE3 pTo = NULL;

      error = rox_model_single_plane_new ( & model_single_plane );
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Image model_imgs = model_multi_plane->planes->data[k]->image_template;
      Rox_Double sizex = model_multi_plane->planes->data[k]->sizex;
      Rox_Double sizey = model_multi_plane->planes->data[k]->sizey;

      error = rox_model_single_plane_set_template_xright_ydown ( model_single_plane, model_imgs, sizex, sizey );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_photoframe_new_model ( &toadd, model_single_plane, border_width );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_objset_photoframe_append ( ident_multi_photoframe_se3->photoframes, toadd );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_new ( &pTo );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Check if model_multi_plane->pTo->data[k] exists ?
      error = rox_matse3_copy ( pTo, model_multi_plane->pTo->data[k] );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_objset_matse3_append ( ident_multi_photoframe_se3->pTo, pTo );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_model_single_plane_del ( &model_single_plane );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   // Delete only if an error occurs
   if ( error ) ROX_ERROR_CHECK( rox_photoframe_del( &toadd ) )

   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_addframe_model (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   const Rox_Model_Single_Plane model_single_plane,
   const Rox_Sint border_width
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame toadd = NULL;
   
   if ( !model_single_plane )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !ident_multi_photoframe_se3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (border_width < 1)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_photoframe_new_model ( &toadd, model_single_plane, border_width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_photoframe_append ( ident_multi_photoframe_se3->photoframes, toadd );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_MatSE3 pTo = NULL;

   error = rox_matse3_new ( &pTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_matse3_append ( ident_multi_photoframe_se3->pTo, pTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   // Delete only if an error occurs
   if ( error ) rox_photoframe_del ( &toadd );

   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_addframe_masked (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   const Rox_Model_Single_Plane model_single_plane,
   const Rox_Sint border_width,
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_PhotoFrame toadd = NULL;

   if (!ident_multi_photoframe_se3 || !model_single_plane)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( border_width < 1 )
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_photoframe_new_model ( &toadd, model_single_plane, border_width );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_photoframe_set_mask( toadd, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_photoframe_append(ident_multi_photoframe_se3->photoframes, toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_MatSE3 pTo = NULL;

   error = rox_matse3_new ( &pTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_matse3_append ( ident_multi_photoframe_se3->pTo, pTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   // Delete only if an error occurs
   if(error) ROX_ERROR_CHECK(rox_photoframe_del(&toadd))

   return error;
}


Rox_ErrorCode rox_ident_multi_photoframe_se3_getcountframes (
   Rox_Sint * count,
   const Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_multi_photoframe_se3 || !count)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *count = ident_multi_photoframe_se3->photoframes->used;

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_multi_photoframe_se3_make (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   const Rox_Camera camera
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint quad_count = 0;

   if ( !ident_multi_photoframe_se3 || !camera )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Reset frames to mark them as not detected
   for (Rox_Uint idframe = 0; idframe < ident_multi_photoframe_se3->photoframes->used; idframe++)
   {
      error = rox_photoframe_reset ( ident_multi_photoframe_se3->photoframes->data[idframe] );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_array2d_uint_fillval(ident_multi_photoframe_se3->mask, ~0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_set_quad_color(ident_multi_photoframe_se3->detector, ident_multi_photoframe_se3->quad_color);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_set_segment_length_min(ident_multi_photoframe_se3->detector, ident_multi_photoframe_se3->quad_segment_length_min);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rox_Timer timer = 0;
   // Rox_Double time = 0.0;

   // error = rox_timer_new(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_start(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // Quad detection depends on selected processing method
   if ( ident_multi_photoframe_se3->detection_method == 0 )
   {
      error = rox_quaddetector_process_image ( ident_multi_photoframe_se3->detector, camera->image, ident_multi_photoframe_se3->mask );
   }
   else
   {
      error = rox_quaddetector_process_image_ac ( ident_multi_photoframe_se3->detector, camera->image, ident_multi_photoframe_se3->mask);
   }
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quaddetector_get_quad_count ( &quad_count, ident_multi_photoframe_se3->detector );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_stop(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_get_elapsed_ms(&time, timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_start(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint idquad = 0; idquad < quad_count; idquad++)
   {
      // qhom contains the homography that transforms the quad basic coordinates into the quad image coordinates:
      // p_quad_image = qhom * p_quad_basic
      // p_quad_basic = [  0, cols+2*border, cols+2*border_u,               0;
      //                   0,             0, rows+2*border_v, rows+2*border_v;
      //                   1,             1,               1,               1];

      error = rox_quaddetector_get_quad_SL3 ( ident_multi_photoframe_se3->qhom, ident_multi_photoframe_se3->detector, idquad );
      ROX_ERROR_CHECK_CONTINUE(error);

      for (Rox_Uint idframe = 0; idframe < ident_multi_photoframe_se3->photoframes->used; idframe++)
      {
         error = rox_photoframe_set_score_threshold ( ident_multi_photoframe_se3->photoframes->data[idframe], ident_multi_photoframe_se3->score_threshold );
         ROX_ERROR_CHECK_CONTINUE(error);

         error = rox_photoframe_make_se3 ( ident_multi_photoframe_se3->photoframes->data[idframe], camera->image, ident_multi_photoframe_se3->qhom, camera->calib_camera, ident_multi_photoframe_se3->orientation_method );
         ROX_ERROR_CHECK_CONTINUE(error);
      }
   }

   // Set the optimal score to 0, set correctly in make_optimal
   ident_multi_photoframe_se3->score = 0;

   // error = rox_timer_stop(timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_get_elapsed_ms(&time, timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_timer_del(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_multi_photoframe_se3_make_optimal (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   const Rox_Camera camera
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_ident_multi_photoframe_se3_make ( ident_multi_photoframe_se3, camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_multi_photoframe_se3_compute_optimal ( ident_multi_photoframe_se3, camera->calib_camera );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_get_result (
   Rox_Sint * is_identified,
   Rox_Double * score,
   Rox_MatSE3 cTp,
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_ident_multi_photoframe_se3_get_pose ( is_identified, cTp, ident_multi_photoframe_se3, id );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_multi_photoframe_se3_get_score ( score, ident_multi_photoframe_se3, id );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_get_optimal_result (
   Rox_Sint * identified,
   Rox_Double * score,
   Rox_MatSE3 cTo,
   const Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_ident_multi_photoframe_se3_get_optimal_pose ( identified, cTo, ident_multi_photoframe_se3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_multi_photoframe_se3_get_optimal_score ( score, ident_multi_photoframe_se3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_multi_photoframe_se3_get_pose (
   Rox_Sint * is_identified,
   Rox_MatSE3 cTp,
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !ident_multi_photoframe_se3 || !is_identified || !cTp )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (id > (Rox_Sint) ident_multi_photoframe_se3->photoframes->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *is_identified = ident_multi_photoframe_se3->photoframes->data[id]->is_detected;
   if (*is_identified)
   {
      error = rox_matse3_copy ( cTp, ident_multi_photoframe_se3->photoframes->data[id]->pose );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_get_optimal_pose (
   Rox_Sint * is_identified,
   Rox_MatSE3 cTo,
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !ident_multi_photoframe_se3 || !is_identified || !cTo )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *is_identified = 0;
   if ( ident_multi_photoframe_se3->score > ident_multi_photoframe_se3->score_threshold )
   {
      *is_identified = 1;
      error = rox_matse3_copy ( cTo, ident_multi_photoframe_se3->cTo );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_get_score (
   Rox_Double *score,
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_multi_photoframe_se3 || !score)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (id > (Rox_Sint) ident_multi_photoframe_se3->photoframes->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *score = ident_multi_photoframe_se3->photoframes->data[id]->score;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_get_optimal_score (
   Rox_Double *score,
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !ident_multi_photoframe_se3 || !score )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *score = ident_multi_photoframe_se3->score;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_get_pose_object (
   Rox_Sint * is_identified,
   Rox_MatSE3 cTo,
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !ident_multi_photoframe_se3 || !is_identified || !cTo )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (id > (Rox_Sint) ident_multi_photoframe_se3->photoframes->used)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *is_identified = ident_multi_photoframe_se3->photoframes->data[id]->is_detected;
   if (*is_identified)
   {
      error = rox_matse3_mulmatmat ( cTo, ident_multi_photoframe_se3->photoframes->data[id]->pose, ident_multi_photoframe_se3->pTo->data[id] );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_multi_photoframe_se3_set_orientation_method (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Sint orientation_method
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!ident_multi_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (orientation_method > 1)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ident_multi_photoframe_se3->orientation_method = orientation_method;

function_terminate:
  return error;
}


Rox_ErrorCode rox_ident_multi_photoframe_se3_set_detection_method (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_photoframe,
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


Rox_ErrorCode rox_ident_multi_photoframe_se3_set_type (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Sint black_to_white
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!ident_multi_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (black_to_white > 1)
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ident_multi_photoframe_se3->quad_color = black_to_white;

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_multi_photoframe_se3_set_segment_length_min (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   const Rox_Double segment_length_min
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!ident_multi_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // if (segment_length_min < 0.0)
   // { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ident_multi_photoframe_se3->quad_segment_length_min = segment_length_min;

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_multi_photoframe_se3_set_score_threshold (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Double score_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_multi_photoframe_se3)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if ( 0.0 > score_threshold || 1.0 < score_threshold )
   {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   ident_multi_photoframe_se3->score_threshold = score_threshold;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_set_side_bounds (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Double side_min,
   Rox_Double side_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_multi_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_quaddetector_set_side_bounds ( ident_multi_photoframe_se3->detector, side_min, side_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_set_area_bounds (
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Double area_min,
   Rox_Double area_max
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ident_multi_photoframe_se3)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_quaddetector_set_area_bounds ( ident_multi_photoframe_se3->detector, area_min, area_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}


Rox_ErrorCode rox_ident_multi_photoframe_se3_get_code (
   Rox_Sint * is_identified,
   Rox_Sint * code_number,
   Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3,
   Rox_Camera camera,
   Rox_Sint code_bits,
   Rox_Sint id
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_CodedFrame codedframe = NULL;
   Rox_MatSE3 T = NULL;

   if (!ident_multi_photoframe_se3 || !camera || !is_identified || !code_number)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *code_number = 0;

   error = rox_matse3_new(&T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ident_multi_photoframe_se3_get_pose(is_identified, T, ident_multi_photoframe_se3, id);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_codedframe_new(&codedframe);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (is_identified)
   {
      Rox_MatSL3 H = ident_multi_photoframe_se3->photoframes->data[id]->resH;

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
Rox_ErrorCode rox_ident_multi_photoframe_se3_get_best_result (
   Rox_Sint * identified,
   Rox_Sint * best_photoframe_id,
   Rox_Double * best_score,
   Rox_MatSE3 best_pose, // cTo
   const Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 cTp = NULL;

   if ( !ident_multi_photoframe_se3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !identified || !best_photoframe_id || !best_score || !best_pose )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_new ( &cTp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint nb_frames = 0;
   error = rox_ident_multi_photoframe_se3_getcountframes ( &nb_frames, ident_multi_photoframe_se3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *identified = 0;
   *best_score = -1.0;
   *best_photoframe_id = -1;

   for (Rox_Sint k=0; k<nb_frames; k++)
   {
      Rox_Double score = 0.0;
      Rox_Sint identified_k = 0;

      error = rox_ident_multi_photoframe_se3_get_result ( &identified_k, &score, cTp, ident_multi_photoframe_se3, k );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ( identified_k )
      {
         *identified = 1;
         if (score > *best_score)
         {
            *best_photoframe_id = k;
            *best_score = score;
            // Get the best pose cTo  
            error = rox_matse3_mulmatmat ( best_pose, cTp, ident_multi_photoframe_se3->pTo->data[k] );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
      }

   }

function_terminate:
   rox_matse3_del(&cTp);
   return error;
}

Rox_ErrorCode rox_ident_multi_photoframe_se3_compute_optimal ( Rox_Ident_Multi_PhotoFrame_SE3 ident_multi_photoframe_se3, const Rox_MatUT3 Kc )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_MatSE3 oTp = NULL;
   Rox_MatSE3 cTp = NULL;
   Rox_MatSE3 cTo = NULL;
   Rox_MatSE3 best_pose = NULL;
   Rox_Double best_score = 0.0;
   Rox_Double score = 0.0;

   Rox_Sint identified = 0;
   Rox_Double vvs_score = -1.0;
 
   if ( !ident_multi_photoframe_se3 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !Kc )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get pointer
   cTo = ident_multi_photoframe_se3->cTo;

   error = rox_matse3_new ( &cTp );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_new ( &oTp );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new ( &best_pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint nb_frames = 0;
   error = rox_ident_multi_photoframe_se3_getcountframes ( &nb_frames, ident_multi_photoframe_se3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_DynVec_Point3D_Double mo = NULL;  // Should be a dynvec since there are many possible planes

   error = rox_dynvec_point3d_double_new ( &mo, 4*nb_frames );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_DynVec_Point2D_Double pc = NULL;  // Should be a dynvec since there are many possible planes

   error = rox_dynvec_point2d_double_new ( &pc, 4*nb_frames );
   ROX_ERROR_CHECK_TERMINATE ( error );

   identified = 0;
   vvs_score = -1.0;
   // Prepare 
   for (Rox_Sint k=0; k < nb_frames; k++)
   {
      Rox_Sint identified_k = 0;

      // get the pose cTb
      error = rox_ident_multi_photoframe_se3_get_result ( &identified_k, &score, cTp, ident_multi_photoframe_se3, k );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ( identified_k )
      {

         Rox_MatSE3 pTo = ident_multi_photoframe_se3->pTo->data[k];

         error = rox_matse3_inv ( oTp, pTo );
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Double depths[4];

         Rox_Double model_sizex = ident_multi_photoframe_se3->photoframes->data[k]->width_meters;
         Rox_Double model_sizey = ident_multi_photoframe_se3->photoframes->data[k]->height_meters;

         Rox_Point3D_Double_Struct mp[4];

         mp[0].X = -model_sizex / 2.0;
         mp[0].Y = -model_sizey / 2.0;
         mp[0].Z = 0.0;

         mp[1].X = +model_sizex / 2.0;
         mp[1].Y = -model_sizey / 2.0;
         mp[1].Z = 0.0;

         mp[2].X = +model_sizex / 2.0;
         mp[2].Y = +model_sizey / 2.0;
         mp[2].Z = 0.0;

         mp[3].X = -model_sizex / 2.0;
         mp[3].Y = +model_sizey / 2.0;
         mp[3].Z = 0.0;

         Rox_Point3D_Double_Struct mo_k[4];
      
         error = rox_point3d_double_transform ( mo_k, oTp, mp, 4 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Append vector of points 3D
         error = rox_dynvec_point3d_double_append_vector_point3d_double ( mo, mo_k, 4 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Point2D_Double_Struct pc_k[4];

         error = rox_point2d_double_transform_project ( pc_k, depths, Kc, cTp, mp, 4 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Append vector of points 3D
         error = rox_dynvec_point2d_double_append_vector_point2d_double ( pc, pc_k, 4 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         identified = 1;

         if (score > best_score)
         {
            best_score = score;
            // Get the best pose cTo  
            error = rox_matse3_mulmatmat ( best_pose, cTp, pTo );
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
      }

   }

   if ( identified == 1 )
   {
      // Copy the best pose
      error = rox_matse3_copy ( cTo, best_pose );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // pc is a dynvec point 2d
      // mo is a dynvec point 3d
      error = rox_points_double_refine_pose_vvs ( cTo, Kc, pc, mo, 100.0 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      vvs_score = best_score;
   }
   else
   {
      vvs_score = -1;
   }

   ident_multi_photoframe_se3->score = vvs_score;

function_terminate:
   rox_matse3_del ( &best_pose );
   rox_matse3_del ( &cTp );
   rox_matse3_del ( &oTp );
   return error;
}
