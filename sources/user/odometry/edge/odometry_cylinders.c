//==============================================================================
//
//    OPENROX   : File odometry_cylinders.c
//
//    Contents  : Implementation of module odometry cylinders
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_cylinders.h"

#include <float.h>
#include <generated/dynvec_edge_cylinder_site_struct.h>

#include <baseproc/geometry/line/line2d.h>
#include <baseproc/geometry/line/line2d_struct.h>
#include <baseproc/geometry/line/line3d_struct.h>
#include <baseproc/geometry/line/line_from_points.h>

#include <baseproc/geometry/segment/segment2d.h>
#include <baseproc/geometry/segment/segment2d_struct.h>

#include <baseproc/geometry/cylinder/cylinder2d_struct.h>
#include <baseproc/geometry/cylinder/cylinder3d_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/robust/huber.h>
#include <baseproc/array/inverse/svdinverse.h>

#include <core/odometry/edge/objset_edge_cylinder_tools.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <baseproc/geometry/transforms/transform_tools.h>

Rox_ErrorCode rox_odometry_cylinders_new (
   Rox_Odometry_Cylinders * odometry_cylinders, 
   const Rox_Double fu,
   const Rox_Double fv,
   const Rox_Double cu,
   const Rox_Double cv,
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Cylinders ret = NULL;


   if (!odometry_cylinders) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *odometry_cylinders = NULL;

   ret = (Rox_Odometry_Cylinders) rox_memory_allocate(sizeof(*ret), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->pose = NULL;
   // Create a new matse3 pose matrix
   error = rox_matse3_new(&ret->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->calibration = NULL;
   // Create a new matct2 calibration matrix
   error = rox_matut3_new ( &ret->calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set the camera calibration matrix
   error = rox_transformtools_build_calibration_matrix ( ret->calibration, fu, fv, cu, cv );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   ret->objset_edge_cylinder = NULL;
   // Create a new objset of type edge cylinder and init with one edge_cylinder
   error = rox_objset_edge_cylinder_new(&ret->objset_edge_cylinder, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->tracker = NULL;
   // Create a new tracker for edge cylinder tracking
   error = rox_tracking_cylinder_new(&ret->tracker, RoxTrackingCylinderMethod_Search_Edge, search_range, contrast_threshold);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *odometry_cylinders = ret;

function_terminate:
   if (error) rox_odometry_cylinders_del(&ret);
   return error;
}

Rox_ErrorCode rox_odometry_cylinders_del(Rox_Odometry_Cylinders * odometry_cylinders)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Cylinders todel = NULL;


   if (!odometry_cylinders) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *odometry_cylinders;
   *odometry_cylinders = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_matse3_del(&todel->pose);
   rox_matut3_del(&todel->calibration);
   rox_objset_edge_cylinder_del(&todel->objset_edge_cylinder);
   rox_tracking_cylinder_del(&todel->tracker);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cylinders_add_cylinder(Rox_Odometry_Cylinders odometry_cylinders, Rox_Cylinder3D cylinder3d, const Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!odometry_cylinders || !cylinder3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_objset_edge_cylinder_add_cylinder3d(odometry_cylinders->objset_edge_cylinder, cylinder3d, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

// #define TEST_INTERACTION_MATRIX

Rox_ErrorCode rox_odometry_cylinders_estimate_pose(Rox_Odometry_Cylinders odometry_cylinders, const Rox_Sint max_iters)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double verr = NULL, wb1 = NULL, wb2 = NULL, vw = NULL;
   Rox_Array2D_Double A = NULL, b = NULL, A_inv = NULL, x = NULL;
   Rox_Double * ptrErr = NULL;


   if (!odometry_cylinders) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Enumerate the valid cylinder sites
   Rox_Sint nbrows = 0;
   error = rox_objset_edge_cylinder_get_valid_measures(&nbrows, odometry_cylinders->objset_edge_cylinder);
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
   
   error = rox_array2d_double_new(&A, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&b, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&A_inv, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(verr, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(vw, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dverr = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dverr, verr );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dvw = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dvw, vw );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve camera intrinsic parameters
   Rox_Array2D_Double K = odometry_cylinders->calibration;

   // Retrieve camera extrinsic parameters
   Rox_MatSE3 cTo = odometry_cylinders->pose;

   // Estimation loop
   for (Rox_Sint iter = 0; iter < max_iters; iter++)
   {
      // Transform the cylinders in the camera frame
      error = rox_objset_edge_cylinder_transform_project(odometry_cylinders->objset_edge_cylinder, K, cTo);
      ROX_ERROR_CHECK_TERMINATE ( error );

      ptrErr = dverr[0];
      // Compute difference between estimated cylinders and measured cylinders
      error = rox_objset_edge_cylinder_build_error(&ptrErr, K, odometry_cylinders->objset_edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute M-estimators weights, compute weights and propagate to containers
      // Do not use Tukey for MBO ??? since it completely eliminates lines and may put the visual servoing in singularity
      // error = rox_array2d_double_tukey(vw, wb1, wb2, verr);
      // ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_huber(vw, wb1, wb2, verr);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Initialize linear system
      rox_array2d_double_fillval(A, 0.0);
      rox_array2d_double_fillval(b, 0.0);

      #ifdef TEST_INTERACTION_MATRIX
         rox_array2d_double_fillval(vw, 1.0);
      #endif

      // Build linear system for ellipses : A * x = b, where A = L'*L and b = L'*e
      error = rox_objset_edge_cylinder_build_linear_system(A, b, K, odometry_cylinders->objset_edge_cylinder, dverr, dvw);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // rox_array2d_double_print(A);

      #ifdef TEST_INTERACTION_MATRIX
         Rox_Array2D_Double L = NULL;
         error = rox_array2d_double_new(&L, nbrows, 6);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_edge_cylinder_build_interaction_matrix(L, K, odometry_cylinders->objset_edge_cylinder);

         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_log("L : \n");
         rox_array2d_double_print(L);

         rox_log("e : \n");
         rox_array2d_double_print(verr);

         rox_log("A = L'*L : \n");
         rox_array2d_double_print(A);

         rox_log("b = L'*e : \n");
         rox_array2d_double_print(b);
      #endif

      // Solve system
      error = rox_array2d_double_svdinverse(A_inv, A);

      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute x = inv(A) * b
      error = rox_array2d_double_mulmatmat(x, A_inv, b);
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
   rox_array2d_double_del(&A_inv);
   rox_array2d_double_del(&A);
   rox_array2d_double_del(&b);
   rox_array2d_double_del(&x);
   return error;
}

Rox_ErrorCode rox_odometry_cylinders_validate_tracking(Rox_Odometry_Cylinders odometry_cylinders)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double expected;
   Rox_Sint nbgood, nbbad;
   // Rox_Sint min;
   //Rox_Double vest, vmes;

   
   if (!odometry_cylinders) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbgood = 0;
   nbbad = 0;
   expected = 0.0;

   for (Rox_Uint idseg = 0; idseg < odometry_cylinders->objset_edge_cylinder->used; idseg++)
   {
      Rox_Edge_Cylinder cylinder = odometry_cylinders->objset_edge_cylinder->data[idseg];

      expected += cylinder->expected_density;

      // Segment 1
      for (Rox_Uint idsite = 0; idsite < cylinder->sites_segment_1->used; idsite++)
      {
         Rox_Edge_Cylinder_Site_Struct site = cylinder->sites_segment_1->data[idsite];

         if (site.state == 0)
         {
            nbgood++;
         }
         else
         {
            nbbad++;
         }
      }

      // Segment 2
      for (Rox_Uint idsite = 0; idsite < cylinder->sites_segment_2->used; idsite++)
      {
         Rox_Edge_Cylinder_Site_Struct site = cylinder->sites_segment_2->data[idsite];

         if (site.state == 0)
         {
            nbgood++;
         }
         else
         {
            nbbad++;
         }
      }
      // TODO : ellipses
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cylinders_update_edges(Rox_Odometry_Cylinders odometry_cylinders)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   for (Rox_Uint idseg = 0; idseg < odometry_cylinders->objset_edge_cylinder->used; idseg++)
   {
      Rox_Edge_Cylinder edge_cylinder = odometry_cylinders->objset_edge_cylinder->data[idseg];

      error = rox_edge_cylinder_transform_project(edge_cylinder, odometry_cylinders->pose, odometry_cylinders->calibration);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cylinders_get_measures (
   Rox_Odometry_Cylinders odometry_cylinders,
   const Rox_Image image
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!odometry_cylinders || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_ObjSet_Edge_Cylinder objset_edge_cylinder = odometry_cylinders->objset_edge_cylinder;
   Rox_Tracking_Cylinder tracker = odometry_cylinders->tracker;
   Rox_MatSE3 pose = odometry_cylinders->pose;
   Rox_Array2D_Double calibration = odometry_cylinders->calibration;

   error = rox_objset_edge_cylinder_get_measures(objset_edge_cylinder, tracker, pose, calibration, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cylinders_get_score(Rox_Double * score, Rox_Odometry_Cylinders odometry_cylinders)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double score_max = odometry_cylinders->tracker->search_edge->_search_range;

   error = rox_objset_edge_cylinder_get_score(score, odometry_cylinders->objset_edge_cylinder, score_max);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cylinders_make (
   Rox_Odometry_Cylinders odometry_cylinders,
   const Rox_Image image,
   const Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Double score = -1.0; // Init score to an impossible value (score must be between 0 and 1)

   if (!odometry_cylinders || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Track edges in image and get new measures
   error = rox_odometry_cylinders_get_measures(odometry_cylinders, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //error = rox_odometry_cylinders_get_score(&score, odometry_cylinders);
   //ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimate pose given new measurements
   error = rox_odometry_cylinders_estimate_pose(odometry_cylinders, max_iters);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Test tracking result, should be test odometry result ?
   error = rox_odometry_cylinders_validate_tracking(odometry_cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Update edges
   error = rox_odometry_cylinders_update_edges(odometry_cylinders);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
