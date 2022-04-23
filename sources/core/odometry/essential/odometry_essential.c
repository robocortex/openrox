//==============================================================================
//
//    OPENROX   : File odometry_essential.c
//
//    Contents  : Implementation of odometry_essential module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_essential.h"

#include <baseproc/maths/maths_macros.h>

#include <generated/dynvec_point2d_float_struct.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <core/indirect/essential/ransacessentialpose.h>
#include <core/indirect/essential/essentialminimize.h>
#include <core/indirect/essential/essentialerror.h>
#include <core/indirect/euclidean/triangulate.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_essential_new(Rox_Odometry_Essential * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Essential ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Odometry_Essential)rox_memory_allocate(sizeof(*ret), 1);

   ret->pose = 0;
   ret->calib = 0;
   ret->essential = 0;
   ret->list_current = 0;
   ret->list_reference = 0;
   ret->list_current_coarse_inliers = 0;
   ret->list_reference_coarse_inliers = 0;
   ret->list_current_corrected = 0;
   ret->list_reference_corrected = 0;
   ret->list_match_flag = 0;

   error = rox_array2d_double_new(&ret->pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->essential, 3, 3);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_float_new(&ret->list_reference, 10);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_float_new(&ret->list_current, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_dynvec_point2d_float_new(&ret->list_current_coarse_inliers, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_float_new(&ret->list_reference_coarse_inliers, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_float_new(&ret->list_current_corrected, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_point2d_float_new(&ret->list_reference_corrected, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_dynvec_uint_new(&ret->list_match_flag, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (error)
   {
      rox_odometry_essential_del(&ret);
      goto function_terminate;
   }

   rox_array2d_double_fillunit(ret->calib);

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_essential_del(Rox_Odometry_Essential * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Essential todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   rox_array2d_double_del(&todel->pose);
   rox_array2d_double_del(&todel->essential);
   rox_array2d_double_del(&todel->calib);
   rox_dynvec_point2d_float_del(&todel->list_reference);
   rox_dynvec_point2d_float_del(&todel->list_current);
   rox_dynvec_point2d_float_del(&todel->list_reference_coarse_inliers);
   rox_dynvec_point2d_float_del(&todel->list_current_coarse_inliers);
   rox_dynvec_point2d_float_del(&todel->list_reference_corrected);
   rox_dynvec_point2d_float_del(&todel->list_current_corrected);
   rox_dynvec_uint_del(&todel->list_match_flag);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_essential_set_calibration(Rox_Odometry_Essential obj, Rox_Array2D_Double calib)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(obj->calib, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_essential_set_rawmatches(Rox_Odometry_Essential obj, Rox_DynVec_Point2D_Float ref, Rox_DynVec_Point2D_Float cur)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !ref || !cur) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (ref->used != cur->used) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_float_clone(obj->list_reference, ref);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_float_clone(obj->list_current, cur);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_essential_process(Rox_Odometry_Essential obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_fillunit(obj->pose);
   rox_dynvec_point2d_float_reset(obj->list_reference_corrected);
   rox_dynvec_point2d_float_reset(obj->list_current_corrected);
   rox_dynvec_uint_reset(obj->list_match_flag);

   // At least 5 points must be in the raw lists
   if (obj->list_current->used < 5) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, obj->calib);
   Rox_Double px = dk[0][0];
   Rox_Double py = dk[1][1];
   Rox_Double u0 = dk[0][2];
   Rox_Double v0 = dk[1][2];

   // Coarse estimation of the pose using the epipolar constraint.
   error = rox_ransac_essential_pose(obj->pose, obj->calib, obj->list_current_coarse_inliers, obj->list_reference_coarse_inliers, obj->list_current, obj->list_reference);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  Refine pose estimation using the epipolar constraint on the SE(3) group
   error = rox_essential_minimize(obj->pose, obj->list_match_flag, obj->list_reference_coarse_inliers, obj->list_current_coarse_inliers);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  Build the essential matrix using the pose
   error = rox_transformtools_build_essential(obj->essential, obj->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  Set the counter to zero
   rox_dynvec_uint_reset(obj->list_match_flag);
   obj->count_valid_match = 0;

   //  Loop over point list
   for ( Rox_Uint idpt = 0; idpt < obj->list_current->used; idpt++)
   {
      Rox_Point2D_Double_Struct pcur, pref, ccur, cref, difcur, difref;
      Rox_Point2D_Float_Struct corcur, corref;
      Rox_Double distcur, distref;
      Rox_Uint valid;

      pref.u = obj->list_reference->data[idpt].u;
      pref.v = obj->list_reference->data[idpt].v;
      pcur.u = obj->list_current->data[idpt].u;
      pcur.v = obj->list_current->data[idpt].v;

      //  Compute the optimal coordinates for this point
      error = rox_pose_triangulate_correction_gold(&cref, &ccur, &pref, &pcur, obj->pose);
      ROX_ERROR_CHECK_TERMINATE ( error );

      corcur.u = (Rox_Float) ccur.u;
      corcur.v = (Rox_Float) ccur.v;
      corref.u = (Rox_Float) cref.u;
      corref.v = (Rox_Float) cref.v;

      //  Transform to pixel for distance check
      pcur.u = px * pcur.u + u0;
      pcur.v = py * pcur.v + v0;
      pref.u = px * pref.u + u0;
      pref.v = py * pref.v + v0;
      ccur.u = px * ccur.u + u0;
      ccur.v = py * ccur.v + v0;
      cref.u = px * cref.u + u0;
      cref.v = py * cref.v + v0;

      //  Compute distance between optimal and measurement
      difcur.u = pcur.u - ccur.u;
      difcur.v = pcur.v - ccur.v;
      difref.u = pref.u - cref.u;
      difref.v = pref.v - cref.v;
      distcur = sqrt(difcur.u * difcur.u + difcur.v * difcur.v);
      distref = sqrt(difref.u * difref.u + difref.v * difref.v);

      valid = 0;
      if (distcur <= 1.0 && distref <= 1.0)
      {
         obj->count_valid_match++;
         valid = 1;
      }

      rox_dynvec_point2d_float_append(obj->list_current_corrected, &corcur);
      rox_dynvec_point2d_float_append(obj->list_reference_corrected, &corref);
      rox_dynvec_uint_append(obj->list_match_flag, &valid);
   }


   if (obj->count_valid_match < 5) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_essential_get_pose(Rox_Array2D_Double pose, Rox_Odometry_Essential obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_copy(pose, obj->pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_essential_get_valid_matches(Rox_DynVec_Uint list_match_flag, Rox_Odometry_Essential obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!obj || !list_match_flag) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_uint_clone(list_match_flag, obj->list_match_flag);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}