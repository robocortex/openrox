//==============================================================================
//
//    OPENROX   : File matching3d2d.c
//
//    Contents  : Implementation of motion detection module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "matching_3d2d.h"

#include <user/detection/motion/cluster_struct.h>
#include <generated/array2d_uint.h>
#include <generated/dynvec_uint_struct.h>

#include <generated/dynvec_rect_sint_struct.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <string.h>
#include <system/memory/memory.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/geometry/point/point2d_print.h>
#include <inout/geometry/point/point3d_print.h>

#include <inout/geometry/point/dynvec_point2d_print.h>
#include <inout/geometry/point/dynvec_point3d_print.h>
#include <inout/numeric/array2d_print.h>

#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/maths/random/combination.h>
#include <baseproc/maths/random/combination_struct.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/conversion/array2d_uchar_from_float.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>

#include <baseproc/geometry/point/point2d_tools.h>
#include <baseproc/geometry/point/point3d_tools.h>
#include <baseproc/geometry/measures/distance_point_to_point.h>

#include <core/indirect/euclidean/p3points.h>
#include <baseproc/maths/linalg/matse3.h>

Rox_ErrorCode
rox_array2d_float_erode_(Rox_Array2D_Float output, Rox_Array2D_Float input, Rox_Float threshold);

Rox_ErrorCode
rox_array2d_uchar_apply_mask_(Rox_Image out, Rox_Imask mask);

Rox_ErrorCode
rox_array2d_uint_set_ones_rectangle_(Rox_Imask mask, Rox_Uint posu, Rox_Uint posv, Rox_Uint sizu, Rox_Uint sizv);

//! \ingroup Matching_3D2D
//! \brief Matching 3D to 2D points structure
struct Rox_Matching_3Dto2D_Struct
{
   //! The 3D model points in meters
   Rox_DynVec_Point3D_Double points3D;

   //! The 2D reprojected points in normalized coordinates
   Rox_DynVec_Point2D_Double points2D;

   //! The random combination for the 2D points
   Rox_Combination combination_points2D;

   //! The random combination for the 3D points
   Rox_Combination combination_points3D;

   //! The camera calibration
   Rox_Array2D_Double calib;

   //! The pose computed after matching
   Rox_Array2D_Double pose;

   //! The flag to know is model is identified and pose computed
   Rox_Uint is_identified;
};

Rox_ErrorCode rox_matching_3d_to_2d_points_new(Rox_Matching_3Dto2D * matching, Rox_DynVec_Point3D_Double points3D, Rox_Array2D_Double calib)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Matching_3Dto2D ret = NULL;
   
   if(!matching) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   ret = (Rox_Matching_3Dto2D) rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   
   // Create a new dynamic vector of 2D points 
   error = rox_dynvec_point2d_double_new(&ret->points2D, points3D->used);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Force use of number of cells equal to  number of 3D model points 
   error = rox_dynvec_point2d_double_usecells(ret->points2D, points3D->used);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create a new dynamic vector of 3D points
   error = rox_dynvec_point3d_double_new(&ret->points3D, points3D->used);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy the 3D points
   error = rox_dynvec_point3d_double_clone(ret->points3D, points3D);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Create a new combination for selecting 3 from the set of 3D points 
   error = rox_combination_new(&ret->combination_points3D, points3D->used, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Create a new pose
   error = rox_array2d_double_new(&ret->pose,4,4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create a new calib
   error = rox_array2d_double_new(&ret->calib,3,3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(ret->calib, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The combination is not created since we don't know yet how many 2D points will be used
   ret->combination_points2D = NULL;

   // Set to zero by default
   ret->is_identified = 0;

   *matching = ret;

function_terminate:
   if(error) rox_matching_3d_to_2d_points_del(&ret);
   return error;
}

Rox_ErrorCode
rox_dynvec_point2d_double_remove_usedcell(Rox_DynVec_Point2D_Double points2D, Rox_Sint index);

Rox_ErrorCode rox_dynvec_point2d_double_remove_usedcell(Rox_DynVec_Point2D_Double points2D, Rox_Sint index)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if((index <= (Rox_Sint)points2D->used) && (points2D->used > 0))
   {
      error = rox_point2d_double_copy(&points2D->data[index], &points2D->data[points2D->used-1]);
      points2D->used = points2D->used -1;
   }
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_double_matching(Rox_Double * score, Rox_Uint * matched_points, Rox_DynVec_Point2D_Double points2D_reprojected, Rox_DynVec_Point2D_Double points2D_measured, Rox_Double matching_threshold)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double distance_min = 0.0;
   Rox_Uint index = 0;

   Rox_DynVec_Point2D_Double points2D_measured_copy = NULL;

   *matched_points = 0;
   *score = 0.0;

   // Create a new dynamic vector of 2D points
   error = rox_dynvec_point2d_double_new(&points2D_measured_copy, points2D_measured->used);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Copy the 2D points
   error = rox_dynvec_point2d_double_clone(points2D_measured_copy, points2D_measured);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // For each reprojected point of the model find the closest measure
   for ( Rox_Uint i = 0; i < points2D_reprojected->used; i++)
   {
      error = rox_dynvec_point2d_double_minimum_distance(&distance_min, &index, &points2D_reprojected->data[i], points2D_measured_copy);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if(distance_min < matching_threshold)
      {
         // We found a matched point
         *matched_points = *matched_points + 1;
         // Update score
         *score = *score + distance_min;
         // Take away the measured point from the list ?

         error = rox_dynvec_point2d_double_remove_usedcell(points2D_measured_copy, index);
         ROX_ERROR_CHECK_TERMINATE ( error );

         //rox_dynvec_point2d_double_print(points2D_measured_copy);
     }
   }

function_terminate:
   rox_dynvec_point2d_double_del(&points2D_measured_copy);
   return error;
}

Rox_ErrorCode rox_matching_3d_to_2d_points_make(Rox_Matching_3Dto2D matching, Rox_DynVec_Point2D_Double points2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint max_draws = 100000;
   Rox_Uint valid_poses = 0;
   Rox_MatSE3 pose_debug = NULL;
   Rox_MatSE3 pose_error = NULL;
   Rox_Array2D_Double K = NULL;
   Rox_Double matching_score = 0.0;
   Rox_Uint matching_points = 0;
   Rox_Double matching_threshold = 0.1;//1.0; // in pixels
   Rox_Double err_tra = 0.0, err_rot = 0.0;
   Rox_Uint k = 0;
   Rox_Uint id_pose = 0;

   Rox_Double data_debug[16] = {0.007069616909909, 0.999972178145731, -0.002379800670276, -20.352946583916967, 0.144951104814326, 0.001329953258385, 0.989437925509952, -16.921811234550862, 0.989413562535836, -0.007339901825885, -0.144937669745767, 92.424129271671191, 0.0, 0.0, 0.0, 1.0};
   // double fu = 1122; double fv = 1123; double cu = 959; double cv = 539;

   Rox_Point2D_Double_Struct points2D_subset[3];
   Rox_Point3D_Double_Struct points3D_subset[3];

   Rox_Array2D_Double_Collection possible_poses = NULL;
   
   error = rox_array2d_double_collection_new(&possible_poses, 4, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the camera calibration matrix
   K = matching->calib;

   // Create a new pose
   error = rox_matse3_new(&pose_error);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Create a new pose
   error = rox_matse3_new(&pose_debug);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_set_data(pose_debug, data_debug);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   // Init to zero
   matching->is_identified = 0;

   // Create a new combination for 2D points
   error = rox_combination_new(&matching->combination_points2D, points2D->used, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for(k = 0; k < max_draws; k++)
   {
      Rox_DynVec_Uint draw_point2D = NULL;
      Rox_DynVec_Uint draw_point3D = NULL;

      // Draw a new combination for 2D points
      error = rox_combination_draw(matching->combination_points2D);
      ROX_ERROR_CHECK_TERMINATE ( error );

      draw_point2D = matching->combination_points2D->draw;

      // Draw a new combination for 3D points
      error = rox_combination_draw(matching->combination_points3D);
      ROX_ERROR_CHECK_TERMINATE ( error );

      draw_point3D = matching->combination_points3D->draw;

      // Set the true matching for debug

      rox_point3d_double_copy(&points3D_subset[0], &matching->points3D->data[draw_point3D->data[0]]);
      rox_point3d_double_copy(&points3D_subset[1], &matching->points3D->data[draw_point3D->data[1]]);
      rox_point3d_double_copy(&points3D_subset[2], &matching->points3D->data[draw_point3D->data[2]]);

      rox_point2d_double_copy(&points2D_subset[0], &points2D->data[draw_point2D->data[0]]);
      rox_point2d_double_copy(&points2D_subset[1], &points2D->data[draw_point2D->data[1]]);
      rox_point2d_double_copy(&points2D_subset[2], &points2D->data[draw_point2D->data[2]]);


      // Compute all possible solutions
      error = rox_pose_from_p3p_pix(possible_poses, &valid_poses, points3D_subset, points2D_subset, K);
      ROX_ERROR_CHECK_TERMINATE(error)

      // Check which solution is best using the rest of the data
      for (id_pose = 0; id_pose < valid_poses; id_pose++)
      {
         Rox_Array2D_Double current_pose = NULL;
         current_pose = rox_array2d_double_collection_get(possible_poses, id_pose);

         // Project the model in the image
         error =  rox_point3d_double_transform_project(matching->points2D->data, current_pose, K, matching->points3D->data, matching->points3D->used);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_dynvec_point2d_double_matching(&matching_score, &matching_points, matching->points2D, points2D, matching_threshold);
         ROX_ERROR_CHECK_TERMINATE(error)

         // Compute errors with respect to measured points
         error = rox_matse3_mulinvmat(pose_error, pose_debug, current_pose);
         ROX_ERROR_CHECK_TERMINATE(error)

         //error = rox_array2d_double_print(pose_error);
         //ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_matse3_error(&err_tra, &err_rot, pose_error);
         ROX_ERROR_CHECK_TERMINATE(error)

         //if((err_tra < 0.01) && (err_rot < 0.01))
         if((matching_points >= 4))//&& (matching_score/matching_points < 20))// && (err_rot < 0.5)) // err_rot < 0.5 rad -> err_rot < 30 deg
         {
            error = rox_array2d_double_copy(matching->pose, current_pose);
            ROX_ERROR_CHECK_TERMINATE(error)

            // Stop if matching is found
            matching->is_identified = 1;

            goto function_terminate;
         }
      }
   }

function_terminate:

   error = rox_array2d_double_collection_del(&possible_poses);
   ROX_ERROR_CHECK(error);

   // Delete the combination for 2D points
   error = rox_combination_del(&matching->combination_points2D);
   ROX_ERROR_CHECK(error);

   // Create a new pose
   error = rox_matse3_del(&pose_error);
   ROX_ERROR_CHECK(error);

   // Create a new pose
   error = rox_matse3_del(&pose_debug);
   ROX_ERROR_CHECK(error);

   return error;
}

Rox_ErrorCode rox_matching_3d_to_2d_points_get_pose(Rox_Uint * is_identified, Rox_Array2D_Double pose, Rox_Matching_3Dto2D matching)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // TOD check inputs

   *is_identified = matching->is_identified;
   if(matching->is_identified == 1)
   {
      error = rox_array2d_double_copy(pose, matching->pose);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_matching_3d_to_2d_points_del(Rox_Matching_3Dto2D * matching)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Matching_3Dto2D todel = NULL;

   if(matching != NULL)
   {
      todel = *matching;
      *matching = NULL;

      // Delete the pose
      error = rox_array2d_double_del(&todel->pose);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Delete the combination for 3D points
      error = rox_combination_del(&todel->combination_points3D);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point3d_double_del(&todel->points3D);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point2d_double_del(&todel->points2D);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_del(&todel->calib);
      ROX_ERROR_CHECK_DISPLAY(error);

      if(todel != NULL)
      rox_memory_delete(todel);
   }

function_terminate:
   return error;
}
