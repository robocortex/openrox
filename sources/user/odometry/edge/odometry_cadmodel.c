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

#include "odometry_cadmodel.h"

#include <baseproc/array/add/add.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/norm/norm2sq.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillzero.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/inverse/pseudoinverse.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/array/robust/huber.h>

//#include <core/odometry/edge/odometry_segments.h>
//#include <core/odometry/edge/odometry_ellipses.h>
//#include <core/odometry/edge/odometry_cylinders.h>

#include <core/odometry/edge/objset_edge_segment_tools.h>
#include <core/odometry/edge/objset_edge_ellipse_tools.h>
#include <core/odometry/edge/objset_edge_cylinder_tools.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_cadmodel_new ( 
   Rox_Odometry_CadModel * odometry_cadmodel, 
   const Rox_Double fu,
   const Rox_Double fv,
   const Rox_Double cu,
   const Rox_Double cv,
   const Rox_Sint search_range, 
   const Rox_Double contrast_threshold 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_CadModel ret = NULL;

   if (!odometry_cadmodel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *odometry_cadmodel = NULL;

   ret = (Rox_Odometry_CadModel) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->pose = NULL;
   // Create a new matse3 pose matrix
   error = rox_matse3_new(&ret->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->calibration = NULL;
   // Create a new matut3 calibration matrix
   error = rox_matut3_new ( &ret->calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set the camera calibration matrix
   error = rox_transformtools_build_calibration_matrix ( ret->calibration, fu, fv, cu, cv );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   ret->objset_edge_segment = NULL;
   // Create a new objset of type edge segment and init with one edge_segment
   error = rox_objset_edge_segment_new(&ret->objset_edge_segment, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->tracking_segment = NULL;
   // Create a new tracker for edge segment tracking
   error = rox_tracking_segment_new ( &ret->tracking_segment, RoxTrackingSegmentMethod_Search_Edge, search_range, contrast_threshold);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->objset_edge_ellipse = NULL;

   // Create a new objset of type edge ellipse and init with one edge_ellipse 
   error = rox_objset_edge_ellipse_new ( &ret->objset_edge_ellipse, 10 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create a new tracker for edge ellipse tracking
   ret->tracking_ellipse = NULL;
   error = rox_tracking_ellipse_new(&ret->tracking_ellipse, RoxTrackingEllipseMethod_Search_Edge, search_range, contrast_threshold);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->objset_edge_cylinder = NULL;
   // Create a new objset of type edge ellipse and init with one edge_cylinder 
   error = rox_objset_edge_cylinder_new ( &ret->objset_edge_cylinder, 10 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->tracking_cylinder = NULL;
   // Create a new tracker for edge ellipse tracking
   error = rox_tracking_cylinder_new(&ret->tracking_cylinder, RoxTrackingCylinderMethod_Search_Edge, search_range, contrast_threshold);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *odometry_cadmodel = ret;

function_terminate:
   if (error) rox_odometry_cadmodel_del(&ret);
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_del(Rox_Odometry_CadModel * odometry_cadmodel)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_CadModel todel = NULL;

   if (!odometry_cadmodel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *odometry_cadmodel;
   *odometry_cadmodel = NULL;

   rox_matse3_del(&todel->pose);
   rox_array2d_double_del(&todel->calibration);

   // Delete edge segments and tracking
   rox_objset_edge_segment_del(&todel->objset_edge_segment);
   rox_tracking_segment_del(&todel->tracking_segment);

   // Delete edge ellipses and tracking
   rox_objset_edge_ellipse_del(&todel->objset_edge_ellipse);
   rox_tracking_ellipse_del(&todel->tracking_ellipse);

   // Delete edge cylinders and tracking
   rox_objset_edge_cylinder_del(&todel->objset_edge_cylinder);
   rox_tracking_cylinder_del(&todel->tracking_cylinder);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_add_segment(Rox_Odometry_CadModel odometry_cadmodel, Rox_Segment3D segment3d, const Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_cadmodel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_objset_edge_segment_add_segment3d(odometry_cadmodel->objset_edge_segment, segment3d, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

 function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_add_ellipse(Rox_Odometry_CadModel odometry_cadmodel, Rox_Ellipse3D ellipse3d, const Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_cadmodel || !ellipse3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Ellipse3D ellipse3d_toadd = NULL;
   error = rox_ellipse3d_new(&ellipse3d_toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ellipse3d_copy(ellipse3d_toadd, ellipse3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_edge_ellipse_add_ellipse3d(odometry_cadmodel->objset_edge_ellipse, ellipse3d_toadd, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

 function_terminate:
   rox_ellipse3d_del(&ellipse3d_toadd);
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_add_cylinder(Rox_Odometry_CadModel odometry_cadmodel, Rox_Cylinder3D cylinder3d, const Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_cadmodel || !cylinder3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Cylinder3D cylinder3d_toadd = NULL;
   error = rox_cylinder3d_new(&cylinder3d_toadd);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_cylinder3d_copy(cylinder3d_toadd, cylinder3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_edge_cylinder_add_cylinder3d(odometry_cadmodel->objset_edge_cylinder, cylinder3d, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_cylinder3d_del(&cylinder3d_toadd);
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_update_edges(Rox_Odometry_CadModel odometry_cadmodel)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_cadmodel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TODO
   for (Rox_Uint idseg = 0; idseg < odometry_cadmodel->objset_edge_segment->used; idseg++)
   {
      Rox_Edge_Segment edge_segment = odometry_cadmodel->objset_edge_segment->data[idseg];

      error = rox_edge_segment_transform_project ( edge_segment, odometry_cadmodel->pose, odometry_cadmodel->calibration);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_validate_tracking(Rox_Odometry_CadModel odometry_cadmodel)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_cadmodel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TODO or NOT TODO ?

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_get_measures(Rox_Odometry_CadModel odometry_cadmodel, Rox_Image image)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!odometry_cadmodel || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_MatSE3 pose = odometry_cadmodel->pose;
   Rox_Array2D_Double calibration = odometry_cadmodel->calibration;

   // Get measures from segments
   Rox_ObjSet_Edge_Segment objset_edge_segment = odometry_cadmodel->objset_edge_segment;
   Rox_Tracking_Segment tracking_segment = odometry_cadmodel->tracking_segment;

   error = rox_objset_edge_segment_get_measures(objset_edge_segment, tracking_segment, pose, calibration, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get measures from ellipses
   Rox_ObjSet_Edge_Ellipse objset_edge_ellipse = odometry_cadmodel->objset_edge_ellipse;
   Rox_Tracking_Ellipse tracking_ellipse = odometry_cadmodel->tracking_ellipse;

   error = rox_objset_edge_ellipse_get_measures(objset_edge_ellipse, tracking_ellipse, pose, calibration, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get measures from cylinders
   Rox_ObjSet_Edge_Cylinder objset_edge_cylinder = odometry_cadmodel->objset_edge_cylinder;
   Rox_Tracking_Cylinder tracking_cylinder = odometry_cadmodel->tracking_cylinder;

   error = rox_objset_edge_cylinder_get_measures(objset_edge_cylinder, tracking_cylinder, pose, calibration, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_build_new_weights (
   Rox_Array2D_Double * res_weights,
   Rox_Array2D_Double errors,
   Rox_Odometry_CadModel odometry_cadmodel
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry_cadmodel || !res_weights)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //check validations
   Rox_Sint valid_measures_segments = 0;
   Rox_Sint valid_measures_ellipses = 0;
   Rox_Sint valid_measures_cylinders = 0;
   Rox_Sint valid_measures_all = 0;

   error = rox_objset_edge_segment_get_valid_measures(&valid_measures_segments, odometry_cadmodel->objset_edge_segment);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_objset_edge_ellipse_get_valid_measures(&valid_measures_ellipses, odometry_cadmodel->objset_edge_ellipse);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_edge_cylinder_get_valid_measures(&valid_measures_cylinders, odometry_cadmodel->objset_edge_cylinder);
   ROX_ERROR_CHECK_TERMINATE ( error );

   valid_measures_all = valid_measures_cylinders + valid_measures_ellipses + valid_measures_segments;
   if (valid_measures_all == 0)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create buffers
   Rox_Array2D_Double wb1_segments = NULL, wb2_segments = NULL;
   Rox_Array2D_Double vw_segments = NULL, verr_segments = NULL;
   Rox_Array2D_Double wb1_ellipses = NULL, wb2_ellipses = NULL;
   Rox_Array2D_Double vw_ellipses = NULL, verr_ellipses = NULL;
   Rox_Array2D_Double wb1_cylinders = NULL, wb2_cylinders = NULL;
   Rox_Array2D_Double vw_cylinders = NULL, verr_cylinders = NULL;

   if (valid_measures_all > 0)
   {
      error = rox_array2d_double_new(res_weights, valid_measures_all, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (valid_measures_segments > 0)
   {
      error = rox_array2d_double_new(&wb1_segments, valid_measures_segments, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&wb2_segments, valid_measures_segments, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&vw_segments, *res_weights, 0, 0, valid_measures_segments, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&verr_segments, errors, 0, 0, valid_measures_segments, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (valid_measures_ellipses > 0)
   {
      error = rox_array2d_double_new(&wb1_ellipses, valid_measures_ellipses, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&wb2_ellipses, valid_measures_ellipses, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&vw_ellipses, *res_weights, valid_measures_segments, 0, valid_measures_ellipses, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&verr_ellipses, errors, valid_measures_segments, 0, valid_measures_ellipses, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (valid_measures_cylinders > 0)
   {
      error = rox_array2d_double_new(&wb1_cylinders, valid_measures_cylinders, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&wb2_cylinders, valid_measures_cylinders, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&vw_cylinders, *res_weights, valid_measures_segments + valid_measures_ellipses, 0, valid_measures_cylinders, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new_subarray2d(&verr_cylinders, errors, valid_measures_segments + valid_measures_ellipses, 0, valid_measures_cylinders, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Compute M-estimators weights, compute weights and propagate to containers
   // Do not use Tukey for MBO ??? since it completely eliminates lines and may put the visual servoing in singularity
   // error = rox_array2d_double_tukey(vw, wb1, wb2, verr);
   // ROX_ERROR_CHECK_TERMINATE ( error );
   if (valid_measures_segments > 0)
   {
      error = rox_array2d_double_huber(vw_segments, wb1_segments, wb2_segments, verr_segments);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (valid_measures_ellipses > 0)
   {
      error = rox_array2d_double_huber(vw_ellipses, wb1_ellipses, wb2_ellipses, verr_ellipses);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (valid_measures_cylinders > 0)
   {
      error = rox_array2d_double_huber(vw_cylinders, wb1_cylinders, wb2_cylinders, verr_cylinders);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&verr_segments);
   rox_array2d_double_del(&wb1_segments);
   rox_array2d_double_del(&wb2_segments);
   rox_array2d_double_del(&vw_segments);

   rox_array2d_double_del(&verr_ellipses);
   rox_array2d_double_del(&wb1_ellipses);
   rox_array2d_double_del(&wb2_ellipses);
   rox_array2d_double_del(&vw_ellipses);

   rox_array2d_double_del(&verr_cylinders);
   rox_array2d_double_del(&wb1_cylinders);
   rox_array2d_double_del(&wb2_cylinders);
   rox_array2d_double_del(&vw_cylinders);
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_build_new_errors (
   Rox_Array2D_Double * res_errors,
   Rox_Odometry_CadModel odometry_cadmodel)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry_cadmodel || !res_errors)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   //check validations
   Rox_Sint valid_measures_segments = 0;
   Rox_Sint valid_measures_ellipses = 0;
   Rox_Sint valid_measures_cylinders = 0;
   Rox_Sint valid_measures_all = 0;

   error = rox_objset_edge_segment_get_valid_measures(&valid_measures_segments, odometry_cadmodel->objset_edge_segment);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_edge_ellipse_get_valid_measures(&valid_measures_ellipses, odometry_cadmodel->objset_edge_ellipse);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_edge_cylinder_get_valid_measures(&valid_measures_cylinders, odometry_cadmodel->objset_edge_cylinder);
   ROX_ERROR_CHECK_TERMINATE ( error );

   valid_measures_all = valid_measures_cylinders + valid_measures_ellipses + valid_measures_segments;
   if (valid_measures_all == 0)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create buffers
   Rox_DynVec_Double err_all = NULL;
   Rox_DynVec_Double err_segments = NULL;
   Rox_DynVec_Double err_ellipses = NULL;
   Rox_DynVec_Double err_cylinders = NULL;

   if (valid_measures_segments > 0)
   {
      error = rox_dynvec_double_new(&err_segments, valid_measures_segments+1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (valid_measures_ellipses > 0)
   {
      error = rox_dynvec_double_new(&err_ellipses, valid_measures_ellipses+1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (valid_measures_cylinders > 0)
   {
      error = rox_dynvec_double_new(&err_cylinders, valid_measures_cylinders+1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (valid_measures_all > 0)
   {
      error = rox_dynvec_double_new(&err_all, valid_measures_all + 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Retrieve camera intrinsic parameters
   Rox_MatUT3 K = odometry_cadmodel->calibration;

   // Retrieve camera extrinsic parameters
   Rox_MatSE3 cTo = odometry_cadmodel->pose;
   if (valid_measures_segments > 0)
   {
      // Transform the segments in the camera frame
      error = rox_objset_edge_segment_transform_project(odometry_cadmodel->objset_edge_segment, K, cTo);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute difference between estimated segments and measured segments
      Rox_Double * p = NULL;
      error = rox_dynvec_double_get_data_pointer( &p, err_segments );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_objset_edge_segment_build_error(&p, K, odometry_cadmodel->objset_edge_segment);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_dynvec_double_usecells(err_segments, valid_measures_segments);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //append to our result vector
      error = rox_dynvec_double_stack(err_all, err_segments);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (valid_measures_ellipses > 0)
   {
      // Transform the ellipses in the camera frame
      error = rox_objset_edge_ellipse_transform_project(odometry_cadmodel->objset_edge_ellipse, K, cTo);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Double * p = NULL;
      error = rox_dynvec_double_get_data_pointer ( &p, err_ellipses );

      // Compute difference between estimated ellipses and measured ellipses
      error = rox_objset_edge_ellipse_build_error(&p, K, odometry_cadmodel->objset_edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_dynvec_double_usecells(err_ellipses, valid_measures_ellipses);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //append to our result vector
      error = rox_dynvec_double_stack(err_all, err_ellipses);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (valid_measures_cylinders > 0)
   {
      // Transform the cylinders in the camera frame
      error = rox_objset_edge_cylinder_transform_project(odometry_cadmodel->objset_edge_cylinder, K, cTo);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute difference between estimated ellipses and measured cylinders
      Rox_Double * p = NULL;
      error = rox_dynvec_double_get_data_pointer ( &p, err_cylinders );

      error = rox_objset_edge_cylinder_build_error(&p, K, odometry_cadmodel->objset_edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_dynvec_double_usecells(err_cylinders, valid_measures_cylinders);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //append to our result vector

      error = rox_dynvec_double_stack(err_all, err_cylinders);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (valid_measures_all > 0)
   {
      Rox_Double * p = NULL;
      error = rox_dynvec_double_get_data_pointer ( &p, err_all );

      error = rox_array2d_double_new(res_errors, valid_measures_all, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_set_buffer_no_stride(*res_errors, p);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_dynvec_double_del(&err_all);
   rox_dynvec_double_del(&err_segments);
   rox_dynvec_double_del(&err_ellipses);
   rox_dynvec_double_del(&err_cylinders);
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_build_new_interaction_matrix (
   Rox_Array2D_Double * res_interaction,
   Rox_Odometry_CadModel odometry_cadmodel)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!odometry_cadmodel || !res_interaction)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //check validations
   Rox_Sint valid_measures_segments = 0;
   Rox_Sint valid_measures_ellipses = 0;
   Rox_Sint valid_measures_cylinders = 0;
   Rox_Sint valid_measures_all = 0;

   error = rox_objset_edge_segment_get_valid_measures(&valid_measures_segments, odometry_cadmodel->objset_edge_segment);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_objset_edge_ellipse_get_valid_measures(&valid_measures_ellipses, odometry_cadmodel->objset_edge_ellipse);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_edge_cylinder_get_valid_measures(&valid_measures_cylinders, odometry_cadmodel->objset_edge_cylinder);
   ROX_ERROR_CHECK_TERMINATE ( error );

   valid_measures_all = valid_measures_cylinders + valid_measures_ellipses + valid_measures_segments;
   if (valid_measures_all == 0)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create buffers
   Rox_Array2D_Double res_segments = NULL;
   Rox_Array2D_Double res_ellipses = NULL;
   Rox_Array2D_Double res_cylinders = NULL;

   if (valid_measures_all > 0)
   {
      error = rox_array2d_double_new(res_interaction, valid_measures_all, 6); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (valid_measures_segments > 0)
   {
      error = rox_array2d_double_new_subarray2d(&res_segments, *res_interaction, 0, 0, valid_measures_segments, 6);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (valid_measures_ellipses > 0)
   {
      error = rox_array2d_double_new_subarray2d(&res_ellipses, *res_interaction, valid_measures_segments, 0, valid_measures_ellipses, 6);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (valid_measures_cylinders > 0)
   {
      error = rox_array2d_double_new_subarray2d(&res_cylinders, *res_interaction, valid_measures_segments+valid_measures_ellipses, 0, valid_measures_cylinders, 6);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Retrieve camera intrinsic parameters
   Rox_MatUT3 K = odometry_cadmodel->calibration;
   if (valid_measures_segments > 0)
   {
      error = rox_objset_edge_segment_build_interaction_matrix(res_segments, K, odometry_cadmodel->objset_edge_segment);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (valid_measures_ellipses > 0)
   {
      error = rox_objset_edge_ellipse_build_interaction_matrix(res_ellipses, K, odometry_cadmodel->objset_edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (valid_measures_cylinders > 0)
   {
      error = rox_objset_edge_cylinder_build_interaction_matrix(res_cylinders, K, odometry_cadmodel->objset_edge_cylinder);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&res_cylinders);
   rox_array2d_double_del(&res_ellipses);
   rox_array2d_double_del(&res_segments);
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_build_new_covariance (
   Rox_Array2D_Double * cov,
   Rox_MatSE3 pose,
   Rox_Array2D_Double errors,
   Rox_Array2D_Double interaction,
   Rox_Array2D_Double weights,
   Rox_Odometry_CadModel odometry_cadmodel
   )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double logse3 = NULL;
   Rox_Array2D_Double wa = NULL;
   Rox_Array2D_Double thetau = NULL, position = NULL;
   Rox_Array2D_Double LthetauInvAnalytic = NULL, I = NULL;
   Rox_Array2D_Double theta2u = NULL;
   Rox_Array2D_Double u = NULL;
   Rox_Array2D_Double theta2u_skew = NULL;
   Rox_Array2D_Double u_skew = NULL;
   Rox_Array2D_Double u_skew_squared = NULL;
   Rox_Array2D_Double theta2u_skew_scaled = NULL;
   Rox_Array2D_Double LthetauInvAnalytic_old = NULL;
   Rox_Array2D_Double sum = NULL;
   Rox_Array2D_Double position_skew = NULL;
   Rox_Array2D_Double ctoInitSkew = NULL;
   Rox_Matrix LpInv = NULL;
   Rox_Matrix Js = NULL;
   Rox_Matrix Js_pseudo_inv = NULL;
   Rox_Array2D_Double deltaP = NULL;
   Rox_Array2D_Double W = NULL;
   Rox_Array2D_Double wax = NULL;
   Rox_Array2D_Double wb = NULL;
   Rox_Array2D_Double wbmwax = NULL;
   Rox_Array2D_Double sigma = NULL;
   Rox_Array2D_Double AtW2 = NULL;
   Rox_Array2D_Double AtW2A = NULL;
   Rox_Array2D_Double cov_scaled = NULL;

   if (!pose || !errors || !interaction || !weights || !odometry_cadmodel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&logse3, 6, 1);                                                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   //vpThetaUVector thetau;
   error = rox_transformtools_logse3_from_pose(logse3, pose);                                            
   ROX_ERROR_CHECK_TERMINATE ( error );
   //cMo.extract(thetau);

   error = rox_array2d_double_new_subarray2d(&thetau, logse3, 3, 0, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&position, logse3, 0, 0, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //vpColVector tu(3);
   //for (Rox_Sint i = 0; i < 3; i++)
   //   tu[i] = thetau[i];

   Rox_Double theta = 0;

   error = rox_array2d_double_norm2(&theta, thetau);                                                     
   ROX_ERROR_CHECK_TERMINATE ( error );
   //double theta = sqrt(tu.sumSquare());

   error = rox_array2d_double_new(&LthetauInvAnalytic, 3, 3);                                            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&I, 3, 3);                                                             
   ROX_ERROR_CHECK_TERMINATE ( error );
   //vpMatrix LthetauInvAnalytic(3, 3);
   //vpMatrix I3(3, 3);
   error = rox_array2d_double_fillunit(I);                                                               
   ROX_ERROR_CHECK_TERMINATE ( error );
   //I3.eye();
   error = rox_array2d_double_scale(LthetauInvAnalytic, I, -1.0);                                        
   ROX_ERROR_CHECK_TERMINATE ( error );
   //ROX_ARRAY2D_DOUBLE_PRINT(LthetauInvAnalytic, "LthetauInvAnalytic ");
   //LthetauInvAnalytic = -I3;

   error = rox_array2d_double_new(&theta2u, 3, 1);                                                       
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&u, 3, 1);                                                             
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&theta2u_skew, 3, 3);                                                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&u_skew, 3, 3);                                                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&u_skew_squared, 3, 3);                                                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&theta2u_skew_scaled, 3, 3);                                           
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&LthetauInvAnalytic_old, 3, 3);                                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&sum, 3, 3);                                                           
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (theta / (2.0 * ROX_PI) > DBL_EPSILON)
   {
      //if (theta / (2.0 * M_PI) > std::numeric_limits<double>::epsilon())
      for (Rox_Sint i = 0; i < 3; i++)
      {
         Rox_Double val_tu = 0;

         error = rox_array2d_double_get_value(&val_tu, thetau, i, 0);                                    
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_set_value(theta2u, i, 0, val_tu/2.0);                                
         ROX_ERROR_CHECK_TERMINATE ( error );
         //      theta2u[i] = tu[i] / 2.0;
         error = rox_array2d_double_set_value(u, i, 0, val_tu /theta);                                   
         ROX_ERROR_CHECK_TERMINATE ( error );
         //      u[i] = tu[i] / theta;
      }
      //ROX_ARRAY2D_DOUBLE_PRINT(u, "u ");
      //ROX_ARRAY2D_DOUBLE_PRINT(theta2u, "theta2u ");
      
      error = rox_transformtools_skew_from_vector(theta2u_skew, theta2u);                                
      ROX_ERROR_CHECK_TERMINATE ( error );
      //ROX_ARRAY2D_DOUBLE_PRINT(theta2u_skew, "theta2u_skew ");
      //   vpMatrix theta2u_skew = vpColVector::skew(theta2u);
      error = rox_transformtools_skew_from_vector(u_skew, u);                                            
      ROX_ERROR_CHECK_TERMINATE ( error );

      //ROX_ARRAY2D_DOUBLE_PRINT(u_skew, "u_skew ");
      //   vpMatrix u_skew = vpColVector::skew(u);

      //u_skew_squared = u_skew*u_skew
      error = rox_array2d_double_mulmatmat(u_skew_squared, u_skew, u_skew);                              
      ROX_ERROR_CHECK_TERMINATE ( error );
      //ROX_ARRAY2D_DOUBLE_PRINT(u_skew_squared, "u_skew_squared ");
      //u_skew = - (1.0 - vpMath::sinc(theta))*u_skew_squared)
      error = rox_array2d_double_scale(u_skew, u_skew_squared, -1.0 + sinc(theta));                      
      ROX_ERROR_CHECK_TERMINATE ( error );

      //ROX_ARRAY2D_DOUBLE_PRINT(u_skew, "u_skew ");
      //theta2u_skew_scaled = -(vpMath::sqr(vpMath::sinc(theta / 2.0)) * theta2u_skew
      Rox_Double sc = sinc(theta / 2.0);
      sc = -sc*sc;

      error = rox_array2d_double_scale(theta2u_skew_scaled, theta2u_skew, sc);       
      ROX_ERROR_CHECK_TERMINATE ( error );
      //ROX_ARRAY2D_DOUBLE_PRINT(theta2u_skew_scaled, "theta2u_skew_scaled ");

      error = rox_array2d_double_scale(LthetauInvAnalytic_old, LthetauInvAnalytic, 1.0);                 
      ROX_ERROR_CHECK_TERMINATE ( error );
      //ROX_ARRAY2D_DOUBLE_PRINT(LthetauInvAnalytic_old, "LthetauInvAnalytic_old ");
      error = rox_array2d_double_add(sum, theta2u_skew_scaled, u_skew);                                 
       ROX_ERROR_CHECK_TERMINATE ( error );
      //ROX_ARRAY2D_DOUBLE_PRINT(sum, "sum ");
      error = rox_array2d_double_add(LthetauInvAnalytic, LthetauInvAnalytic_old, sum);                   
      ROX_ERROR_CHECK_TERMINATE ( error );

      //   LthetauInvAnalytic += -(vpMath::sqr(vpMath::sinc(theta / 2.0)) * theta2u_skew - (1.0 - vpMath::sinc(theta))*u_skew*u_skew);

      //ROX_ARRAY2D_DOUBLE_PRINT(LthetauInvAnalytic, "LthetauInvAnalytic ");
   }

   error = rox_array2d_double_new(&position_skew, 3, 3);                                                 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_skew_from_vector(position_skew, position);                                 
   ROX_ERROR_CHECK_TERMINATE ( error );

   //vpTranslationVector ctoInit;
   //cMo.extract(ctoInit);
   //vpMatrix ctoInitSkew = ctoInit.skew();
   //ROX_ARRAY2D_DOUBLE_PRINT(position_skew, "position_skew ");

   error = rox_array2d_double_new(&ctoInitSkew, 3, 3);                                                   
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(ctoInitSkew, position_skew, LthetauInvAnalytic);                 
   ROX_ERROR_CHECK_TERMINATE ( error );
   //ctoInitSkew = ctoInitSkew * LthetauInvAnalytic;
   //ROX_ARRAY2D_DOUBLE_PRINT(ctoInitSkew, "ctoInitSkew ");

   error = rox_matrix_new(&LpInv, 6, 6);                                                         
   ROX_ERROR_CHECK_TERMINATE ( error );
   //vpMatrix LpInv(6, 6);
   error = rox_array2d_double_fillzero(LpInv);                                                           
   ROX_ERROR_CHECK_TERMINATE ( error );
   //LpInv = 0;
   error = rox_array2d_double_set_value(LpInv, 0, 0, -1.0);                                              
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(LpInv, 1, 1, -1.0);                                              
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(LpInv, 2, 2, -1.0);                                              
   ROX_ERROR_CHECK_TERMINATE ( error );

   //LpInv[0][0] = -1.0;
   //LpInv[1][1] = -1.0;
   //LpInv[2][2] = -1.0;
   //ROX_ARRAY2D_DOUBLE_PRINT(LpInv, "LpInv ");

   Rox_Double ** ctoInitSkew_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &ctoInitSkew_data, ctoInitSkew );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint a = 0; a < 3; a++)
   {
      for (Rox_Sint b = 0; b < 3; b++)
      {
         error = rox_array2d_double_set_value(LpInv, a, b+3, ctoInitSkew_data[a][b]);                    
         ROX_ERROR_CHECK_TERMINATE ( error );
         //LpInv[a][b + 3] = ctoInitSkew[a][b];
      }
   }

   //ROX_ARRAY2D_DOUBLE_PRINT(LpInv, "LpInv ");
   Rox_Double ** LthetauInvAnalytic_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &LthetauInvAnalytic_data, LthetauInvAnalytic);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint a = 0; a < 3; a++)
   {
      for (Rox_Sint b = 0; b < 3; b++)
      {
         error = rox_array2d_double_set_value ( LpInv, a+3, b + 3, LthetauInvAnalytic_data[a][b] );         
         ROX_ERROR_CHECK_TERMINATE ( error );
         //LpInv[a + 3][b + 3] = LthetauInvAnalytic[a][b];
      }
   }
   //ROX_ARRAY2D_DOUBLE_PRINT(LpInv, "LpInv ");
   //// Building Js
   Rox_Sint rows = 0; Rox_Sint cols = 0;
   error = rox_array2d_double_get_rows(&rows, interaction);                                              
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_rows(&cols, LpInv);                                                    
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Js, rows, cols);                                                      
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Js = Ls * LpInv;
   error = rox_array2d_double_mulmatmat(Js, interaction, LpInv);                                         
   ROX_ERROR_CHECK_TERMINATE ( error );

   //ROX_ARRAY2D_DOUBLE_PRINT(interaction, "interaction ");
   //ROX_ARRAY2D_DOUBLE_PRINT(LpInv, "LpInv ");
   //ROX_ARRAY2D_DOUBLE_PRINT(Js, "Js ");

   // building deltaP

   error = rox_array2d_double_get_rows(&rows, Js);                                                       
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_cols(&cols, Js);                                                       
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Js_pseudo_inv, cols, rows);                                           
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_pseudoinverse(Js_pseudo_inv, Js);                                          
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_rows(&rows, Js_pseudo_inv);                                            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_cols(&cols, errors);                                                   
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&deltaP, rows, cols);                                                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(deltaP, Js_pseudo_inv, errors);                                  
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   //ROX_ARRAY2D_DOUBLE_PRINT(deltaP, "deltaP ");
   //deltaP = (Js).pseudoInverse(Js.getRows()*std::numeric_limits<double>::epsilon()) * deltaS;

   //prepare diagonal matrix from weigths vector
   error = rox_array2d_double_get_rows(&rows, weights);                                                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   // To use matrix multiplication directly, we should build a rows x rows matrix and fill its diagonal.
   // So, in order to save some memory space and troubles we'll use a one column matrix and adapt the code
   error = rox_array2d_double_new(&W, rows, 1);                                                          
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** w_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &w_data, W );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** weights_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &weights_data, weights);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!weights_data || !w_data)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      w_data[i][0] = weights_data[i][0];
   }
   //vpMatrix D;
   //D.diag(w_true);
   //ROX_ARRAY2D_DOUBLE_PRINT(errors, "errors ");
   //ROX_ARRAY2D_DOUBLE_PRINT(W, "W ");

   Rox_Double denom = 0.0;
   //double denom = 0.0;
   error = rox_array2d_double_get_cols(&rows, W);                                                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i <rows; i++)
   {
      denom+= w_data[i][i];
   //   denom += W[i][i];
   }

   if (denom <= DBL_EPSILON)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
      //if (denom <= std::numeric_limits<double>::epsilon())
      //   throw vpMatrixException(vpMatrixException::divideByZeroError, "Impossible to compute covariance matrix: not enough data");
   }

   //ROX_ARRAY2D_DOUBLE_PRINT(W, "W ");

   //wax = W * Js * deltaP = (W * A * x)
   error = rox_array2d_double_get_rows(&rows, W);                                                        
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_cols(&cols, Js);                                                       
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&wa, rows, cols);                                                      
   ROX_ERROR_CHECK_TERMINATE ( error );

   //We used a one column matrix for W instead of a square one with only its diagonal, so we adapt the code to provide a matrix multiplication
   //error = rox_array2d_double_mulmatmat(wa, W, Js);                                                      ROX_ERROR_CHECK_TERMINATE ( error );
   Rox_Double ** js_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&js_data, Js);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         error = rox_array2d_double_set_value(wa, i, j, w_data[i][0] * js_data[i][j]);                       
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

   error = rox_array2d_double_get_rows(&rows, wa);                                                       
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_cols(&cols, deltaP);                                                   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&wax, rows, cols);                                                     
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(wax, wa, deltaP);                                                
   ROX_ERROR_CHECK_TERMINATE ( error );
   //wb = w*b = weights * errors
   error = rox_array2d_double_get_rows(&rows, W);                                                        
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_cols(&cols, errors);                                                   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&wb, rows, cols);                                                      
   ROX_ERROR_CHECK_TERMINATE ( error );
   //error = rox_array2d_double_mulmatmat(wb, W, errors);                                                  ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** err_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &err_data, errors );
   ROX_ERROR_CHECK_TERMINATE ( error );


   for ( Rox_Sint i = 0; i < rows; i++)
   {
      error = rox_array2d_double_set_value(wb, i, 0, w_data[i][0] * err_data[i][0]);                       
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   //wbmwax = wb-wax = W * b - (W * A * x)

   error = rox_array2d_double_get_rows(&rows, wb);                                                       
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_cols(&cols, wb);                                                       
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&wbmwax, rows, cols);                                                  
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_substract(wbmwax, wb, wax);                                                
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_cols(&cols, wb);                                                       
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&sigma, cols, cols);                                                   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmattransmat(sigma, wbmwax, wbmwax);                                     
   ROX_ERROR_CHECK_TERMINATE ( error );

   //double sigma2 = (W * b - (W * A * x)).t() * (W*b - (W * A * x));

   //sigma should be a 1x1 matrix
   if(cols != 1)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }
   Rox_Double ** sigma_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&sigma_data, weights);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double sigma2 = sigma_data[0][0];
   sigma2 /= denom;

   //vpMatrix W2(W.getCols(), W.getCols());
   error = rox_array2d_double_get_rows(&rows, W);                                                        
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i <rows; i++)
   {
      w_data[i][0] = w_data[i][0] * w_data[i][0];
      //   W2[i][i] = W[i][i] * W[i][i];
   }

   //cov
   Rox_Array2D_Double At = NULL;
   error = rox_array2d_double_get_rows(&rows, Js);                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_cols(&cols, Js);                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&At, cols, rows);               
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_transpose(At, Js);                  
   ROX_ERROR_CHECK_TERMINATE ( error );


   Rox_Double ** at_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &at_data, At);

   error = rox_array2d_double_get_rows(&rows, At);                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_cols(&cols, At);                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&AtW2, rows, cols);             
   ROX_ERROR_CHECK_TERMINATE ( error );

   ////We used a one column matrix for W2 instead of a square one with only its diagonal, so we adapt the code to provide a matrix multiplication
   for ( Rox_Sint i = 0; i < rows; i++)
   {
      for ( Rox_Sint j = 0; j < cols; j++)
      {
         error = rox_array2d_double_set_value(AtW2, i, j, w_data[i][0] * at_data[i][j]);                       
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

   error = rox_array2d_double_get_rows(&rows, AtW2);                 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_cols(&cols, Js);                   
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&AtW2A, rows, cols);               
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(AtW2A, AtW2, Js);            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_rows(&rows, AtW2A);                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_cols(&cols, AtW2A);                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&cov_scaled, cols, rows);          
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_pseudoinverse(cov_scaled, AtW2A);      
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(cov, cols, rows);                  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale(*cov, cov_scaled, sigma2);       
   ROX_ERROR_CHECK_TERMINATE ( error );

   //return (A.t()*(W2)*A).pseudoInverse(A.getCols()*std::numeric_limits<double>::epsilon())*sigma2;
   //ROX_ARRAY2D_DOUBLE_PRINT(cov_scaled, "cov_scaled ");
   //ROX_ARRAY2D_DOUBLE_PRINT(*cov, "cov ");

   //ROX_ARRAY2D_DOUBLE_PRINT(*cov, "cov ");
function_terminate:
   rox_array2d_double_del(&logse3);
   rox_array2d_double_del(&thetau);
   rox_array2d_double_del(&position);
   rox_array2d_double_del(&LthetauInvAnalytic);
   rox_array2d_double_del(&I);
   rox_array2d_double_del(&theta2u);
   rox_array2d_double_del(&u);
   rox_array2d_double_del(&u_skew);
   rox_array2d_double_del(&u_skew_squared);
   rox_array2d_double_del(&theta2u_skew);
   rox_array2d_double_del(&theta2u_skew_scaled);
   rox_array2d_double_del(&LthetauInvAnalytic_old);
   rox_array2d_double_del(&sum);
   rox_array2d_double_del(&position_skew);
   rox_array2d_double_del(&ctoInitSkew);
   rox_array2d_double_del(&LpInv);
   rox_array2d_double_del(&Js);
   rox_array2d_double_del(&Js_pseudo_inv);
   rox_array2d_double_del(&deltaP);
   rox_array2d_double_del(&W);
   rox_array2d_double_del(&wa);
   rox_array2d_double_del(&wax);
   rox_array2d_double_del(&wb);
   rox_array2d_double_del(&wbmwax);
   rox_array2d_double_del(&sigma);
   rox_array2d_double_del(&At);
   rox_array2d_double_del(&AtW2);
   rox_array2d_double_del(&AtW2A);
   rox_array2d_double_del(&cov_scaled);
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_estimate_pose(Rox_Odometry_CadModel odometry_cadmodel, const Rox_Sint max_iters)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Array2D_Double verr= NULL, wb1= NULL, wb2= NULL, vw= NULL;

   // Variables for segments subsytem
   Rox_Array2D_Double JtJ_segments = NULL, Jte_segments = NULL;
   Rox_Array2D_Double wb1_segments = NULL, wb2_segments = NULL;
   Rox_Array2D_Double vw_segments = NULL, verr_segments=  NULL;
   Rox_Double * ptrErr_segments = NULL;
   Rox_Double ** dverr_segments = NULL;
   Rox_Double ** dvw_segments = NULL;

   // Variables for ellipses subsytem
   Rox_Array2D_Double JtJ_ellipses = NULL, Jte_ellipses = NULL;
   Rox_Array2D_Double wb1_ellipses = NULL, wb2_ellipses = NULL;
   Rox_Array2D_Double vw_ellipses = NULL, verr_ellipses=  NULL;
   Rox_Double * ptrErr_ellipses = NULL;
   Rox_Double ** dverr_ellipses = NULL;
   Rox_Double ** dvw_ellipses = NULL;

   // Variables for cylinders subsytem
   Rox_Array2D_Double JtJ_cylinders = NULL, Jte_cylinders = NULL;
   Rox_Array2D_Double wb1_cylinders = NULL, wb2_cylinders = NULL;
   Rox_Array2D_Double vw_cylinders = NULL, verr_cylinders=  NULL;
   Rox_Double * ptrErr_cylinders = NULL;
   Rox_Double ** dverr_cylinders = NULL;
   Rox_Double ** dvw_cylinders = NULL;

   // variables for cadmodel system
   Rox_Array2D_Double JtJ = NULL, Jte = NULL;
   Rox_Array2D_Double iJtJ = NULL, x = NULL;
   // Rox_Double * ptrErr = NULL;
   
   if (!odometry_cadmodel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Sint valid_measures_segments = 0;
   error = rox_objset_edge_segment_get_valid_measures(&valid_measures_segments, odometry_cadmodel->objset_edge_segment);
   ROX_ERROR_CHECK_TERMINATE ( error );   

   Rox_Sint valid_measures_ellipses = 0;
   error = rox_objset_edge_ellipse_get_valid_measures(&valid_measures_ellipses, odometry_cadmodel->objset_edge_ellipse);
   ROX_ERROR_CHECK_TERMINATE ( error );   

   Rox_Sint valid_measures_cylinders = 0;
   error = rox_objset_edge_cylinder_get_valid_measures(&valid_measures_cylinders, odometry_cadmodel->objset_edge_cylinder);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create buffers
   if (valid_measures_segments > 0)
   {
      error = rox_array2d_double_new(&verr_segments, valid_measures_segments, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_new(&wb1_segments, valid_measures_segments, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&wb2_segments, valid_measures_segments, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&vw_segments, valid_measures_segments, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer(&dverr_segments, verr_segments);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer(&dvw_segments, vw_segments);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (valid_measures_ellipses > 0)
   {
      error = rox_array2d_double_new(&verr_ellipses, valid_measures_ellipses, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&wb1_ellipses, valid_measures_ellipses, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&wb2_ellipses, valid_measures_ellipses, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&vw_ellipses, valid_measures_ellipses, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer(&dverr_ellipses, verr_ellipses);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer(&dvw_ellipses, vw_ellipses);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (valid_measures_cylinders > 0)
   {
      error = rox_array2d_double_new(&verr_cylinders, valid_measures_cylinders, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&wb1_cylinders, valid_measures_cylinders, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&wb2_cylinders, valid_measures_cylinders, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&vw_cylinders, valid_measures_cylinders, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer(&dverr_cylinders, verr_cylinders);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_data_pointer_to_pointer(&dvw_cylinders, vw_cylinders);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_array2d_double_new(&JtJ_segments, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Jte_segments, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&JtJ_ellipses, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Jte_ellipses, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&JtJ_cylinders, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Jte_cylinders, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&JtJ, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Jte, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&iJtJ, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Retrieve camera intrinsic parameters
   Rox_Array2D_Double K = odometry_cadmodel->calibration;

   // Retrieve camera extrinsic parameters
   Rox_MatSE3 cTo = odometry_cadmodel->pose;

   // Estimation loop
   for ( Rox_Sint iter = 0; iter < max_iters; iter++)
   {
      if (valid_measures_segments > 0)
      {
         // Transform the segments in the camera frame
         error = rox_objset_edge_segment_transform_project(odometry_cadmodel->objset_edge_segment, K, cTo);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // ptrErr = dverr[0];
         ptrErr_segments = dverr_segments[0];
         // Compute difference between estimated segments and measured segments
         error = rox_objset_edge_segment_build_error(&ptrErr_segments, K, odometry_cadmodel->objset_edge_segment);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      if (valid_measures_ellipses > 0)
      {
         // Transform the ellipses in the camera frame
         error = rox_objset_edge_ellipse_transform_project(odometry_cadmodel->objset_edge_ellipse, K, cTo);
         ROX_ERROR_CHECK_TERMINATE ( error );

         ptrErr_ellipses = dverr_ellipses[0];
         // Compute difference between estimated ellipses and measured ellipses
         error = rox_objset_edge_ellipse_build_error(&ptrErr_ellipses, K, odometry_cadmodel->objset_edge_ellipse);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      if (valid_measures_cylinders > 0)
      {
         // Transform the cylinders in the camera frame
         error = rox_objset_edge_cylinder_transform_project(odometry_cadmodel->objset_edge_cylinder, K, cTo);
         ROX_ERROR_CHECK_TERMINATE ( error );

         ptrErr_cylinders = dverr_cylinders[0];
         // Compute difference between estimated ellipses and measured cylinders
         error = rox_objset_edge_cylinder_build_error(&ptrErr_cylinders, K, odometry_cadmodel->objset_edge_cylinder);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      // Compute M-estimators weights, compute weights and propagate to containers
      // Do not use Tukey for MBO ??? since it completely eliminates lines and may put the visual servoing in singularity
      // error = rox_array2d_double_tukey(vw, wb1, wb2, verr);
      // ROX_ERROR_CHECK_TERMINATE ( error );
      if (valid_measures_segments > 0)
      {
         error = rox_array2d_double_huber(vw_segments, wb1_segments, wb2_segments, verr_segments);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      if (valid_measures_ellipses > 0)
      {
         error = rox_array2d_double_huber(vw_ellipses, wb1_ellipses, wb2_ellipses, verr_ellipses);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      if (valid_measures_cylinders > 0)
      {
         error = rox_array2d_double_huber(vw_cylinders, wb1_cylinders, wb2_cylinders, verr_cylinders);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      // Initialize linear system
      rox_array2d_double_fillval(JtJ, 0);
      rox_array2d_double_fillval(Jte, 0);

      rox_array2d_double_fillval(JtJ_segments, 0);
      rox_array2d_double_fillval(Jte_segments, 0);

      rox_array2d_double_fillval(JtJ_ellipses, 0);
      rox_array2d_double_fillval(Jte_ellipses, 0);

      rox_array2d_double_fillval(JtJ_cylinders, 0);
      rox_array2d_double_fillval(Jte_cylinders, 0);

      if (valid_measures_segments > 0)
      {
         // Build linear system for segments
         error = rox_objset_edge_segment_build_linear_system(JtJ_segments, Jte_segments, K, odometry_cadmodel->objset_edge_segment, dverr_segments, dvw_segments);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      if (valid_measures_ellipses > 0)
      {
         // Build linear system for ellipses
         error = rox_objset_edge_ellipse_build_linear_system(JtJ_ellipses, Jte_ellipses, K, odometry_cadmodel->objset_edge_ellipse, dverr_ellipses, dvw_ellipses);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      if (valid_measures_cylinders > 0)
      {
         // Build linear system for cylinders
         error = rox_objset_edge_cylinder_build_linear_system(JtJ_cylinders, Jte_cylinders, K, odometry_cadmodel->objset_edge_cylinder, dverr_cylinders, dvw_cylinders);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      // Build total system JtJ = JtJ_segments + JtJ_ellipses + JtJ_cylinders
      error = rox_array2d_double_add(JtJ, JtJ_segments, JtJ_ellipses);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add(JtJ, JtJ, JtJ_cylinders);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Build total system Jte = Jte_segments + Jte_ellipses + Jte_cylinders
      error = rox_array2d_double_add(Jte, Jte_segments, Jte_ellipses);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add(Jte, Jte, Jte_cylinders);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Solve linear system JtJ * x = Jte
      error = rox_array2d_double_svdinverse(iJtJ, JtJ);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // x = inv(JtJ) * Jte
      error = rox_array2d_double_mulmatmat(x, iJtJ, Jte);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Scale factor to tune the exponential decreasing of the error
      Rox_Double lambda = 0.9;

      // Compute increment x = lambda * inv(JtJ) * Jte
      error = rox_array2d_double_scale(x, x, -lambda);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update pose on the left : cTo = expmSO3(x) * cTo
      error = rox_matse3_update_left(cTo, x);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   //rox_array2d_double_del(&verr);
   //rox_array2d_double_del(&wb1);
   //rox_array2d_double_del(&wb2);
   //rox_array2d_double_del(&vw);

   rox_array2d_double_del(&verr_segments);
   rox_array2d_double_del(&wb1_segments);
   rox_array2d_double_del(&wb2_segments);
   rox_array2d_double_del(&vw_segments);

   rox_array2d_double_del(&verr_ellipses);
   rox_array2d_double_del(&wb1_ellipses);
   rox_array2d_double_del(&wb2_ellipses);
   rox_array2d_double_del(&vw_ellipses);

   rox_array2d_double_del(&verr_cylinders);
   rox_array2d_double_del(&wb1_cylinders);
   rox_array2d_double_del(&wb2_cylinders);
   rox_array2d_double_del(&vw_cylinders);

   rox_array2d_double_del(&JtJ);
   rox_array2d_double_del(&Jte);

   rox_array2d_double_del(&JtJ_segments);
   rox_array2d_double_del(&Jte_segments);

   rox_array2d_double_del(&JtJ_ellipses);
   rox_array2d_double_del(&Jte_ellipses);

   rox_array2d_double_del(&JtJ_cylinders);
   rox_array2d_double_del(&Jte_cylinders);

   rox_array2d_double_del(&iJtJ);
   rox_array2d_double_del(&x);
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_get_score(Rox_Double * score, Rox_Odometry_CadModel odometry_cadmodel)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double score_max = 0.0;

   if ( !odometry_cadmodel || !score ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *score = 0.0;

   score_max = odometry_cadmodel->tracking_segment->search_edge->_search_range;

   Rox_Double score_segments = -1.0; // Init score to an impossible value (score must be between 0 and 1)
   error = rox_objset_edge_segment_get_score(&score_segments, odometry_cadmodel->objset_edge_segment, score_max);
   ROX_ERROR_CHECK_TERMINATE ( error );

   score_max = odometry_cadmodel->tracking_ellipse->search_edge->_search_range;

   Rox_Double score_ellipses = -1.0; // Init score to an impossible value (score must be between 0 and 1)
   error = rox_objset_edge_ellipse_get_score(&score_ellipses, odometry_cadmodel->objset_edge_ellipse, score_max);
   ROX_ERROR_CHECK_TERMINATE ( error );

   score_max = odometry_cadmodel->tracking_cylinder->search_edge->_search_range;

   Rox_Double score_cylinders = -1.0; // Init score to an impossible value (score must be between 0 and 1)
   error = rox_objset_edge_cylinder_get_score(&score_cylinders, odometry_cadmodel->objset_edge_cylinder, score_max);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // TODO: Not fair if some edges are not set
   Rox_Sint nb_scores = 0;
   score_segments = odometry_cadmodel->objset_edge_segment->used > 0 ? score_segments : 0;
   nb_scores += odometry_cadmodel->objset_edge_segment->used > 0 ? 1 : 0;

   score_ellipses = odometry_cadmodel->objset_edge_ellipse->used > 0 ? score_ellipses : 0;
   nb_scores += odometry_cadmodel->objset_edge_ellipse->used > 0 ? 1 : 0;

   score_cylinders = odometry_cadmodel->objset_edge_cylinder->used > 0 ? score_cylinders : 0;
   nb_scores += odometry_cadmodel->objset_edge_cylinder->used > 0 ? 1 : 0;

   if (nb_scores > 0)
   {
      *score = (score_segments + score_ellipses + score_cylinders) / nb_scores;
   }
   else
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_get_results (
   Rox_Double * score,
   Rox_MatSE3 pose,
   Rox_Odometry_CadModel odometry_cadmodel
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !odometry_cadmodel || !score || !pose ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_odometry_cadmodel_get_score(score, odometry_cadmodel);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(pose, odometry_cadmodel->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_cadmodel_make (
   Rox_Odometry_CadModel odometry_cadmodel,
   const Rox_Image image,
   const Rox_Sint max_iters
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Rox_Double score = -1.0; // Init score to an impossible value (score must be between 0 and 1)

   if (!odometry_cadmodel || !image) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Track edges in image and get new measures
   error = rox_odometry_cadmodel_get_measures(odometry_cadmodel, image);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimate pose given new measurements
   error = rox_odometry_cadmodel_estimate_pose(odometry_cadmodel, max_iters);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Test tracking result
   error = rox_odometry_cadmodel_validate_tracking(odometry_cadmodel);
   ROX_ERROR_CHECK_TERMINATE ( error );
     
   // Update edges
   error = rox_odometry_cadmodel_update_edges(odometry_cadmodel);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
