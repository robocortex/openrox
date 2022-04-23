//==============================================================================
//
//    OPENROX   : File odometry_ellipses.c
//
//    Contents  : Implementation of module odometry ellipses
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_ellipses.h"

#include <float.h>
#include <generated/dynvec_edge_ellipse_site_struct.h>

#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/ellipse/ellipse3d_struct.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/robust/huber.h>
#include <baseproc/array/inverse/svdinverse.h>

#include <core/odometry/edge/objset_edge_ellipse_tools.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <baseproc/geometry/transforms/transform_tools.h>

Rox_ErrorCode rox_odometry_ellipses_new (
   Rox_Odometry_Ellipses * odometry_ellipses,
   const Rox_Double fu,
   const Rox_Double fv,
   const Rox_Double cu,
   const Rox_Double cv,
   const Rox_Sint search_range,
   const Rox_Double contrast_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Ellipses ret = NULL;


   if (!odometry_ellipses) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *odometry_ellipses = NULL;

   ret = (Rox_Odometry_Ellipses) rox_memory_allocate(sizeof(*ret), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->pose = NULL;
   // Create a new matse3 pose matrix
   error = rox_matse3_new ( &ret->pose ) ;
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->calibration = NULL;
   // Create a new matut3 calibration matrix
   error = rox_matut3_new ( &ret->calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set the camera calibration matrix
   error = rox_transformtools_build_calibration_matrix ( ret->calibration, fu, fv, cu, cv );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->objset_edge_ellipse = NULL;
   // Create a new objset of type edge ellipse and init with one edge_ellipse
   error = rox_objset_edge_ellipse_new(&ret->objset_edge_ellipse, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->tracker = NULL;
   // Create a new tracker for edge ellipse tracking
   error = rox_tracking_ellipse_new(&ret->tracker, RoxTrackingEllipseMethod_Search_Edge, search_range, contrast_threshold);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *odometry_ellipses = ret;

function_terminate:
   if (error) rox_odometry_ellipses_del(&ret);
   return error;
}

Rox_ErrorCode rox_odometry_ellipses_del (
   Rox_Odometry_Ellipses * odometry_ellipses
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Ellipses todel = NULL;


   if (!odometry_ellipses) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *odometry_ellipses;
   *odometry_ellipses = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matse3_del(&todel->pose);
   rox_matut3_del(&todel->calibration);
   rox_objset_edge_ellipse_del(&todel->objset_edge_ellipse);
   rox_tracking_ellipse_del(&todel->tracker);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_ellipses_add_ellipse (
   Rox_Odometry_Ellipses odometry_ellipses,
   const Rox_Ellipse3D ellipse3d,
   const Rox_Double sampling_step
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!odometry_ellipses || !ellipse3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_objset_edge_ellipse_add_ellipse3d(odometry_ellipses->objset_edge_ellipse, ellipse3d, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

 function_terminate:
   return error;
}

// #define TEST_INTERACTION_MATRIX

Rox_ErrorCode rox_odometry_ellipses_estimate_pose (
   Rox_Odometry_Ellipses odometry_ellipses,
   const Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double verr = NULL, wb1 = NULL, wb2 = NULL, vw = NULL;
   Rox_Array2D_Double JtJ = NULL, Jte = NULL, iJtJ = NULL, x = NULL;
   Rox_Double * ptrErr = NULL;


   if (!odometry_ellipses) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Enumerate the valid ellipse sites
   Rox_Sint nbrows = 0;
   error = rox_objset_edge_ellipse_get_valid_measures(&nbrows, odometry_ellipses->objset_edge_ellipse);

   ROX_ERROR_CHECK_TERMINATE ( error );

   if (nbrows == 0)
   { error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create buffers
   error = rox_array2d_double_new(&verr, nbrows, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&wb1, nbrows, 1);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&wb2, nbrows, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&vw, nbrows, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&JtJ, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&Jte, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&iJtJ, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&x, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dverr = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dverr, verr );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dvw = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dvw, vw );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve camera intrinsic parameters
   Rox_MatUT3 K = odometry_ellipses->calibration;

   // Retrieve camera extrinsic parameters
   Rox_MatSE3 cTo = odometry_ellipses->pose;

   // Estimation loop
   for (Rox_Sint iter = 0; iter < max_iters; iter++)
   {
      // Transform the ellipses in the camera frame
      error = rox_objset_edge_ellipse_transform_project(odometry_ellipses->objset_edge_ellipse, K, cTo);
      ROX_ERROR_CHECK_TERMINATE ( error );

      ptrErr = dverr[0];
      // Compute difference between estimated ellipses and measured ellipses
      error = rox_objset_edge_ellipse_build_error(&ptrErr, K, odometry_ellipses->objset_edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute M-estimators weights, compute weights and propagate to containers
      // Do not use Tukey for MBO ??? since it completely eliminates lines and may put the visual servoing in singularity
      // error = rox_array2d_double_tukey(vw, wb1, wb2, verr);
      // ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_huber(vw, wb1, wb2, verr);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Initialize linear system
      rox_array2d_double_fillval(JtJ, 0.0);
      rox_array2d_double_fillval(Jte, 0.0);

      // Build linear system for ellipses
      error = rox_objset_edge_ellipse_build_linear_system(JtJ, Jte, K, odometry_ellipses->objset_edge_ellipse, dverr, dvw);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Solve system
      error = rox_array2d_double_svdinverse(iJtJ, JtJ);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(x, iJtJ, Jte);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Scale factor to tune the exponential decreasing of the error
      Rox_Double lambda = 0.9;

      // Update pose
      error = rox_array2d_double_scale(x, x, -lambda);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matse3_update_left(cTo, x);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:

   rox_array2d_double_del(&verr);
   rox_array2d_double_del(&wb1);
   rox_array2d_double_del(&wb2);
   rox_array2d_double_del(&vw);
   rox_array2d_double_del(&iJtJ);
   rox_array2d_double_del(&JtJ);
   rox_array2d_double_del(&Jte);
   rox_array2d_double_del(&x);

   return error;
}

Rox_ErrorCode rox_odometry_ellipses_validate_tracking(Rox_Odometry_Ellipses odometry_ellipses)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double expected = 0.0;
   Rox_Sint nbgood = 0,  nbbad = 0;
   // Rox_Sint min;
   //Rox_Double vest, vmes;

   
   if (!odometry_ellipses) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbgood = 0;
   nbbad = 0;
   expected = 0.0;

   for (Rox_Uint id = 0; id < odometry_ellipses->objset_edge_ellipse->used; id++)
   {
      Rox_Edge_Ellipse edge_ellipse = odometry_ellipses->objset_edge_ellipse->data[id];

      expected += edge_ellipse->expected_density;

      for (Rox_Uint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
      {
         Rox_Edge_Ellipse_Site_Struct site = edge_ellipse->sites->data[idsite];

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
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_ellipses_update_edges(Rox_Odometry_Ellipses odometry_ellipses)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   for (Rox_Uint idseg = 0; idseg < odometry_ellipses->objset_edge_ellipse->used; idseg++)
   {
      Rox_Edge_Ellipse edge_ellipse = odometry_ellipses->objset_edge_ellipse->data[idseg];

      error = rox_edge_ellipse_transform_project(edge_ellipse, odometry_ellipses->pose, odometry_ellipses->calibration);
      ROX_ERROR_CHECK_CONTINUE(error);
   }

   // If we arrived here there are no problems
   error = ROX_ERROR_NONE;

// function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_ellipses_get_valid_sites(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Odometry_Ellipses odometry_ellipses)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_objset_edge_ellipse_get_valid_sites(dynvec_point2d, odometry_ellipses->objset_edge_ellipse);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_ellipses_get_measures (
   Rox_Odometry_Ellipses odometry_ellipses,
   const Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!odometry_ellipses || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse = odometry_ellipses->objset_edge_ellipse;
   Rox_Tracking_Ellipse tracker = odometry_ellipses->tracker;
   Rox_MatSE3 pose = odometry_ellipses->pose;
   Rox_MatUT3 calibration = odometry_ellipses->calibration;

   error = rox_objset_edge_ellipse_get_measures(objset_edge_ellipse, tracker, pose, calibration, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_ellipses_get_score(Rox_Double * score, Rox_Odometry_Ellipses odometry_ellipses)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double score_max = odometry_ellipses->tracker->search_edge->_search_range;

   error = rox_objset_edge_ellipse_get_score(score, odometry_ellipses->objset_edge_ellipse, score_max);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_ellipses_make (
   Rox_Odometry_Ellipses odometry_ellipses,
   Rox_Image image,
   const Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double score = -1.0; // Init score to an impossible value (score must be between 0 and 1)

   if (!odometry_ellipses || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Track edges in image and get new measures
   error = rox_odometry_ellipses_get_measures ( odometry_ellipses, image );
   ROX_ERROR_CHECK_TERMINATE ( error );
         
   error = rox_odometry_ellipses_get_score ( &score, odometry_ellipses );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimate pose given new measurements
   error = rox_odometry_ellipses_estimate_pose ( odometry_ellipses, max_iters );

   ROX_ERROR_CHECK_TERMINATE ( error );

   // Test tracking result
   error = rox_odometry_ellipses_validate_tracking ( odometry_ellipses );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Update edges using the pose
   error = rox_odometry_ellipses_update_edges (odometry_ellipses );

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_odometry_ellipses_get_score ( &score, odometry_ellipses );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
