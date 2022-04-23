//==============================================================================
//
//    OPENROX   : File odometry_segments.c
//
//    Contents  : Implementation of module odometry segments
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_segments.h"

#include <float.h>

#include <generated/dynvec_edge_segment_site_struct.h>

#include <system/time/timer.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/array/robust/huber.h>
#include <baseproc/geometry/line/line2d.h>
#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/maths_macros.h>

#include <core/odometry/edge/objset_edge_segment_tools.h>

#include <inout/numeric/array2d_save.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#define CONV_THRESH_VT 1e-7 // Default threshold for convergence of translation 
#define CONV_THRESH_VR 1e-6 // Default threshold for convergence of rotation

// #define TEST_INTERACTION_MATRIX

Rox_ErrorCode rox_odometry_segments_new (
   Rox_Odometry_Segments * odometry_segments,
   const Rox_Double fu,
   const Rox_Double fv,
   const Rox_Double cu,
   const Rox_Double cv,
   const Rox_Sint search_range,
   const Rox_Double contrast_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Segments ret = NULL;

   if (!odometry_segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *odometry_segments = NULL;

   ret = (Rox_Odometry_Segments) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->pose = NULL;

   error = rox_matse3_new ( &ret->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->calibration = NULL;
   error = rox_matut3_new ( &ret->calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Set the camera calibration matrix
   error = rox_transformtools_build_calibration_matrix ( ret->calibration, fu, fv, cu, cv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->objset_edge_segment = NULL;
   error = rox_objset_edge_segment_new ( &ret->objset_edge_segment, 100 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->tracker = NULL;
   error = rox_tracking_segment_new ( &ret->tracker, RoxTrackingSegmentMethod_Search_Edge, search_range, contrast_threshold );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *odometry_segments = ret;

function_terminate:
   if (error) rox_odometry_segments_del(&ret);
   return error;
}

Rox_ErrorCode rox_odometry_segments_del (
   Rox_Odometry_Segments * odometry_segments
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Segments todel = NULL;

   if (!odometry_segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *odometry_segments;
   *odometry_segments = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matse3_del(&todel->pose);
   rox_matut3_del(&todel->calibration);
   rox_objset_edge_segment_del(&todel->objset_edge_segment);
   rox_tracking_segment_del(&todel->tracker);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_set_pose (
   Rox_Odometry_Segments odometry_segments,
   const Rox_MatSE3 cTo
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!cTo) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy ( odometry_segments->pose, cTo );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_get_pose (
   Rox_MatSE3 cTo,
   const Rox_Odometry_Segments odometry_segments
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!cTo) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_copy ( cTo, odometry_segments->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_set_segments (
   Rox_Odometry_Segments odometry_segments,
   const Rox_Segment3D segment3d,
   const Rox_Sint nbs,
   const Rox_Double sampling_step
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!segment3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!(sampling_step > 0.0))
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // reset segments
   rox_objset_edge_segment_reset ( odometry_segments->objset_edge_segment );
   
   for ( Rox_Sint id = 0; id < nbs; id++ )
   {
      error = rox_objset_edge_segment_add_segment3d ( odometry_segments->objset_edge_segment, &segment3d[id], sampling_step );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_add_segment (
   Rox_Odometry_Segments odometry_segments,
   Rox_Segment3D segment3d,
   const Rox_Double sampling_step
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!segment3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!(sampling_step > 0.0))
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_objset_edge_segment_add_segment3d ( odometry_segments->objset_edge_segment, segment3d, sampling_step );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_estimate_pose (
   Rox_Odometry_Segments odometry_segments,
   const Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double norm_vt = 0.0;
   Rox_Double norm_vr = 0.0;
   
   Rox_Array2D_Double vt = NULL;
   Rox_Array2D_Double vr = NULL;

   Rox_Array2D_Double verr = NULL, wb1 = NULL, wb2 = NULL, vw = NULL, x = NULL;
   Rox_Matrix LtL = NULL, Lte = NULL, iLtL =  NULL;
   Rox_Double ** dverr = NULL, ** dvw = NULL;
   Rox_Double * ptrErr = NULL;

   if (!odometry_segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Enumerate the valid segment sites
   Rox_Sint nbrows = 0;
   error = rox_objset_edge_segment_get_valid_measures ( &nbrows, odometry_segments->objset_edge_segment );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (nbrows == 0) 
   { error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create buffers
   // Signed errors
   error = rox_array2d_double_new ( &verr, nbrows, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Temporary vectors
   error = rox_array2d_double_new ( &wb1, nbrows, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new ( &wb2, nbrows, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Vector with weights
   error = rox_array2d_double_new ( &vw, nbrows, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new ( &LtL, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new ( &Lte, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matrix_new ( &iLtL, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &x, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Translation velocity
   error = rox_array2d_double_new_subarray2d( &vt, x, 0, 0, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rotation velocity
   error = rox_array2d_double_new_subarray2d( &vr, x, 3, 0, 3, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dverr, verr );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dvw, vw );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve camera intrinsic parameters
   Rox_MatUT3 K = odometry_segments->calibration;

   // Retrieve camera extrinsic parameters
   Rox_MatSE3 cTo = odometry_segments->pose;

   // Estimation loop
   for ( Rox_Sint iter = 0; iter < max_iters; iter++ )
   {
      // Transform the segments in the camera frame
      //error = rox_objset_edge_segment_transform_project ( odometry_segments->objset_edge_segment, K, cTo );
      //ROX_ERROR_CHECK_TERMINATE ( error );

      ptrErr = dverr[0];

      // Compute difference between estimated segments and measured segments
      error = rox_objset_edge_segment_build_error ( &ptrErr, K, odometry_segments->objset_edge_segment );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute M-estimators weights, compute weights and propagate to containers
      // Do not use Tukey for MBO ??? since it completely eliminates lines and may put the visual servoing in singularity
      
      //error = rox_array2d_double_tukey ( vw, wb1, wb2, verr );
      //ROX_ERROR_CHECK_TERMINATE ( error );
      
      //error = rox_array2d_double_huber ( vw, wb1, wb2, verr );
      //ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_fillval ( vw, 1.0 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Initialize jacobians
      error = rox_array2d_double_fillval ( LtL, 0 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_fillval ( Lte, 0 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Build linear system for segments
      error = rox_objset_edge_segment_build_linear_system ( LtL, Lte, K, odometry_segments->objset_edge_segment, dverr, dvw );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Solve system 
      error = rox_array2d_double_svdinverse ( iLtL, LtL );
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_mulmatmat ( x, iLtL, Lte );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Scale factor to tune the exponential decreasing of the error
      Rox_Double lambda = 0.9;

      // Update pose 
      error = rox_array2d_double_scale ( x, x, -lambda);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_update_left ( cTo, x );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // We should add a convergence test
      error = rox_array2d_double_norm2sq ( &norm_vt, vt );
      ROX_ERROR_CHECK_TERMINATE ( error );
         
      error = rox_array2d_double_norm2sq ( &norm_vr, vr );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (( norm_vt < CONV_THRESH_VT ) && ( norm_vr < CONV_THRESH_VR ))
      {
         odometry_segments->convergence = 1;
         break;
      }

      if ( max_iters > 1 )
      {
         // Transform the segments in the camera frame
         error = rox_objset_edge_segment_transform_project ( odometry_segments->objset_edge_segment, K, cTo );
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

function_terminate:
   rox_array2d_double_del ( &verr );
   rox_array2d_double_del ( &wb1 );
   rox_array2d_double_del ( &wb2 );
   rox_array2d_double_del ( &vw );
   rox_array2d_double_del ( &iLtL );
   rox_array2d_double_del ( &LtL );
   rox_array2d_double_del ( &Lte );
   rox_array2d_double_del ( &x );
   rox_matrix_del ( &vt );
   rox_matrix_del ( &vr );

   return error;
}

Rox_ErrorCode rox_odometry_segments_validate_tracking ( Rox_Odometry_Segments odometry_segments )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double expected;
   Rox_Sint nbgood, nbbad, min;
   Rox_Double vest, vmes;
   
   if (!odometry_segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbgood = 0;
   nbbad = 0;
   expected = 0.0;

   for (Rox_Uint id = 0; id < odometry_segments->objset_edge_segment->used; id++)
   {
      Rox_Edge_Segment segment = odometry_segments->objset_edge_segment->data[id];

      expected += segment->expected_density;

      for (Rox_Uint idsite = 0; idsite < segment->sites->used; idsite++)
      {
         Rox_Edge_Segment_Site_Struct site = segment->sites->data[idsite];

         if (site.state == 0)
         {
            nbgood++;
         }
         else
         {
            nbbad++;
         }
      }
   }

   vest = 0.4 * expected;
   vmes = 0.4 * (nbgood + nbbad);
   min = (Rox_Sint) ROX_MIN(vest, vmes);

   if (nbgood < min || expected < 2.0)
   { error = ROX_ERROR_ALGORITHM_FAILURE; }

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_update_edges ( Rox_Odometry_Segments odometry_segments )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_objset_edge_segment_transform_project ( odometry_segments->objset_edge_segment, odometry_segments->calibration, odometry_segments->pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_get_measures (
   Rox_Odometry_Segments odometry_segments,
   const Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_segments || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get pointers
   Rox_ObjSet_Edge_Segment objset_edge_segment  = odometry_segments->objset_edge_segment;
   Rox_Tracking_Segment tracker                 = odometry_segments->tracker;
   Rox_MatSE3 pose                              = odometry_segments->pose;
   Rox_MatUT3 calibration                       = odometry_segments->calibration;

   error = rox_objset_edge_segment_get_measures ( objset_edge_segment, tracker, pose, calibration, image );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_get_measures_gradient (
   Rox_Odometry_Segments odometry_segments,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !odometry_segments || !gradient_scale || !gradient_angle ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get pointers
   Rox_ObjSet_Edge_Segment objset_edge_segment  = odometry_segments->objset_edge_segment;
   Rox_Tracking_Segment tracker                 = odometry_segments->tracker;
   Rox_MatSE3 pose                              = odometry_segments->pose;
   Rox_MatUT3 calibration                       = odometry_segments->calibration;

   error = rox_objset_edge_segment_get_measures_gradient ( objset_edge_segment, tracker, pose, calibration, gradient_scale, gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_get_score ( Rox_Double * score, Rox_Odometry_Segments odometry_segments )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double score_max = odometry_segments->tracker->search_edge->_search_range;

   error = rox_objset_edge_segment_get_score ( score, odometry_segments->objset_edge_segment, score_max );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_make (
   Rox_Odometry_Segments odometry_segments,
   const Rox_Image image,
   const Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Double score = -1.0; // Init score to an impossible value (score must be between 0 and 1)

   // // Define timer to measure performances
   // Rox_Timer timer = NULL;
   // Rox_Double time = 0.0;
   
   // // Init new timer
   // error = rox_timer_new(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   if (!odometry_segments || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
         
   // rox_timer_start(timer);

   // Track edges in image and get new measures
   error = rox_odometry_segments_get_measures ( odometry_segments, image );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to get the measures = %f (ms)\n", time);

   // Should we test convergence ?
   odometry_segments->convergence = 0; // Set to not converged

   //error = rox_odometry_segments_get_score(&score, odometry_segments);
   //ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_timer_start(timer);

   // Estimate pose given new measurements
   error = rox_odometry_segments_estimate_pose ( odometry_segments, max_iters );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to estimate the pose = %f (ms)\n", time);

   // Test tracking result
   error = rox_odometry_segments_validate_tracking ( odometry_segments );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Update edges
   //error = rox_odometry_segments_update_edges ( odometry_segments );
   //ROX_ERROR_CHECK_TERMINATE ( error );

   //error = rox_odometry_segments_get_score(&score, odometry_segments);
   //ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_segments_make_gradient (
   Rox_Odometry_Segments odometry_segments,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle,
   const Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Double score = -1.0; // Init score to an impossible value (score must be between 0 and 1)

   // // Define timer to measure performances
   // Rox_Timer timer = NULL;
   // Rox_Double time = 0.0;
   
   // // Init new timer
   // error = rox_timer_new(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   if ( !odometry_segments || !gradient_scale || !gradient_angle ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
         
   // rox_timer_start(timer);

   // Track edges in image and get new measures
   error = rox_odometry_segments_get_measures_gradient ( odometry_segments, gradient_scale, gradient_angle );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to get the measures = %f (ms)\n", time);

   // Should we test convergence ?
   odometry_segments->convergence = 0; // Set to not converged

   //error = rox_odometry_segments_get_score(&score, odometry_segments);
   //ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_timer_start(timer);

   // Estimate pose given new measurements
   error = rox_odometry_segments_estimate_pose ( odometry_segments, max_iters );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);

   // rox_log("time to estimate the pose = %f (ms)\n", time);

   // Test tracking result
   error = rox_odometry_segments_validate_tracking ( odometry_segments );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Update edges
   //error = rox_odometry_segments_update_edges ( odometry_segments );
   //ROX_ERROR_CHECK_TERMINATE ( error );

   //error = rox_odometry_segments_get_score(&score, odometry_segments);
   //ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
