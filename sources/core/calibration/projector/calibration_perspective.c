//==============================================================================
//
//    OPENROX   : File calibration_perspective.c
//
//    Contents  : Implementation of calibration_perspective module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "calibration_perspective.h"
#include "calibration_perspective_struct.h"

#include <stdio.h>

#include <generated/array2d_double.h>

#include <generated/objset_dynvec_point2d_double_struct.h>
#include <generated/objset_dynvec_point3d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/determinant/detgl3.h>
#include <baseproc/array/minmax/minmax.h>
#include <baseproc/array/median/median.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/solve/svd_solve.h>
#include <baseproc/array/decomposition/cholesky.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d_transform.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/maths/linalg/generators/algut3.h>
#include <baseproc/maths/linalg/matse3.h>

#include <baseproc/calculus/jacobians/interaction_point2d_pix_matse3.h>
#include <baseproc/calculus/jacobians/interaction_point2d_nor_matse3.h>
#include <baseproc/calculus/jacobians/interaction_point2d_pix_matut3.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>

#define threshold_reprojection 10.0 // 1.5

Rox_ErrorCode rox_calibration_projector_perspective_new( Rox_Calibration_Projector_Perspective * obj )
{
   Rox_ErrorCode error;

   Rox_Calibration_Projector_Perspective ret = NULL;

   if(obj == 0)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   *obj = 0;

   ret = (Rox_Calibration_Projector_Perspective)rox_memory_allocate(sizeof(*ret), 1);
   if(ret == 0)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   // Set to NULL all pointers
   ret->poses = 0;
   ret->poses_cpy = 0;
   ret->homographies = 0;
   ret->K = 0;
   ret->K_cpy = 0;
   ret->valid_flags = 0;
   ret->A0 = 0;
   ret->A1 = 0;
   ret->A2 = 0;
   ret->A3 = 0;
   ret->A4 = 0;
   ret->A5 = 0;

   ret->C0 = 0;
   ret->C1 = 0;
   ret->C2 = 0;
   ret->C3 = 0;
   ret->C4 = 0;
   ret->C5 = 0;

   ret->M = 0;
   ret->MtA = 0;
   ret->Kn = 0;
   ret->Kt = 0;
   ret->S = 0;

   ret->obs3D = 0;
   ret->obs2D = 0;
   ret->ref2D = 0;

   ret->valid_image = 0;

   error = rox_array2d_double_new(&ret->K, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->K_cpy, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->poses, 10); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->poses_cpy, 10); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_array2d_double_new(&ret->homographies, 10); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit(ret->K); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_fillunit(ret->K_cpy); ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&ret->A0, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->A1, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->A2, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->A3, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->A4, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->A5, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&ret->C0, 2, 2); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->C1, 2, 2); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->C2, 2, 2); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->C3, 2, 2); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->C4, 2, 2); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->C5, 2, 2); ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&ret->M,  3, 2); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->MtA,2, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->Kn, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->Kt, 3, 3); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&ret->S,  3, 3); ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_dynvec_uint_new(&ret->valid_flags, 20); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_dynvec_point3d_double_new(&ret->obs3D, 20); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_objset_dynvec_point2d_double_new(&ret->obs2D, 20); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_double_new(&ret->ref2D, 50); ROX_ERROR_CHECK_TERMINATE(error)
   *obj = ret;

function_terminate:
   if(error) rox_calibration_projector_perspective_del(&ret);
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_del( Rox_Calibration_Projector_Perspective *obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_Projector_Perspective todel;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = 0;

   if(!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_array2d_double_del(&todel->K);
   rox_array2d_double_del(&todel->K_cpy);
   rox_objset_array2d_double_del(&todel->poses);
   rox_objset_array2d_double_del(&todel->poses_cpy);
   rox_objset_array2d_double_del(&todel->homographies);

   rox_array2d_double_del(&todel->A0);
   rox_array2d_double_del(&todel->A1);
   rox_array2d_double_del(&todel->A2);
   rox_array2d_double_del(&todel->A3);
   rox_array2d_double_del(&todel->A4);
   rox_array2d_double_del(&todel->A5);

   rox_array2d_double_del(&todel->C0);
   rox_array2d_double_del(&todel->C1);
   rox_array2d_double_del(&todel->C2);
   rox_array2d_double_del(&todel->C3);
   rox_array2d_double_del(&todel->C4);
   rox_array2d_double_del(&todel->C5);

   rox_array2d_double_del(&todel->M);
   rox_array2d_double_del(&todel->MtA);
   rox_array2d_double_del(&todel->Kn);
   rox_array2d_double_del(&todel->Kt);
   rox_array2d_double_del(&todel->S);

   rox_dynvec_uint_del(&todel->valid_flags);
   rox_objset_dynvec_point3d_double_del(&todel->obs3D);
   rox_objset_dynvec_point2d_double_del(&todel->obs2D);
   rox_dynvec_point2d_double_del(&todel->ref2D);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_set_model_points(
   Rox_Calibration_Projector_Perspective obj,
   Rox_Point2D_Double refs2D,
   Rox_Uint count)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (obj == 0)    {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (refs2D == 0) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (count <4)    {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   // Reset model
   rox_dynvec_point2d_double_reset(obj->ref2D);

   for (Rox_Uint i = 0; i < count; i++)
   {
      error = rox_dynvec_point2d_double_append(obj->ref2D, &refs2D[i]); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_set_intrinsics(Rox_Calibration_Projector_Perspective obj, Rox_Array2D_Double K)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if(!obj || !K) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   error = rox_array2d_double_copy(obj->K, K);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_get_intrinsics(Rox_Array2D_Double K, Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if(!obj || !K) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   error = rox_array2d_double_copy(K, obj->K);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_add_homography(Rox_Calibration_Projector_Perspective obj, Rox_Array2D_Double H)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double pose = 0, pose_cpy = 0;
   Rox_Array2D_Double homography = 0;
   Rox_Uint valid_flag = 1;
   Rox_Double det;

   if(obj == 0 || H == 0) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_array2d_double_detgl3(&det, H);

   if (fabs(det) <= 0.00001) 
   {error = ROX_ERROR_DETERMINANT_NULL; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_new(&homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&pose, 4, 4);       
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&pose_cpy, 4, 4);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(homography, H);    
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_append(obj->homographies, homography); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_objset_array2d_double_append(obj->poses, pose);         
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_objset_array2d_double_append(obj->poses_cpy, pose_cpy); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_uint_append(obj->valid_flags, &valid_flag);      
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->valid_image++;

function_terminate:
   if(error)
   {
      rox_array2d_double_del(&homography);
      rox_array2d_double_del(&pose);
      rox_array2d_double_del(&pose_cpy);
   }
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_add_current_points(Rox_Calibration_Projector_Perspective obj, Rox_Point2D_Double  obs2D, Rox_Uint count )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_DynVec_Point3D_Double pts3D = 0;
   Rox_DynVec_Point2D_Double pts2D = 0;

   if(obj == 0 || obs2D == 0)
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if(count<4)
      {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_dynvec_point3d_double_new(&pts3D, 50);
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_double_new(&pts2D, 50);
   ROX_ERROR_CHECK_TERMINATE(error)
   for (Rox_Uint i = 0; i < count; i++)
   {
      Rox_Point2D_Double_Struct pt2D = obs2D[i];
      Rox_Point3D_Double_Struct pt3D;
      pt3D.X = pt2D.u;
      pt3D.Y = pt2D.v;
      pt3D.Z = 0.0;
      
      error = rox_dynvec_point3d_double_append(pts3D, &pt3D); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_dynvec_point2d_double_append(pts2D, &pt2D); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_objset_dynvec_point3d_double_append(obj->obs3D, pts3D); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_objset_dynvec_point2d_double_append(obj->obs2D, pts2D); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if(error)
   {
      rox_dynvec_point3d_double_del(&pts3D);
      rox_dynvec_point2d_double_del(&pts2D);
   }

   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_add_measure(
   Rox_Calibration_Projector_Perspective obj,
   Rox_Array2D_Double H,
   Rox_Point2D_Double  obs2D,
   Rox_Uint count )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_calibration_projector_perspective_add_current_points(obj, obs2D, count); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_calibration_projector_perspective_add_homography(obj, H); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_compute_parameters(Rox_Calibration_Projector_Perspective obj, Rox_Uint method)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint Kddl = method;
   Rox_Uint recalibrate = 0;

   Rox_Double min, max, med, std, mean;

   if (obj == 0)                    
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (method > 5 || method == 0)   
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (obj->homographies->used < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (obj->valid_image == 0)       
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(obj->valid_image == 1 && Kddl > 2) Kddl = 2;
   if(obj->valid_image == 2 && Kddl > 3) Kddl = 3;

   // Copy K init
   error = rox_array2d_double_copy(obj->K_cpy, obj->K);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Check homographies
   error = rox_calibration_projector_perspective_check_homographies(obj);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Init with linearized solution
   error = rox_calibration_projector_perspective_process_linear(obj, Kddl);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Compute poses
   for (Rox_Uint i = 0; i < obj->homographies->used; i++)
   {
      // Compute poses
      error = rox_transformtools_build_pose_intermodel(obj->poses->data[i], obj->homographies->data[i], obj->K); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_copy(obj->poses_cpy->data[i], obj->poses->data[i]); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Refine with non linear optimization
   error = rox_calibration_projector_perspective_process_nolinear(obj, Kddl);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Get stats
   for (Rox_Uint i = 0; i < obj->homographies->used; i++)
   {
      if(obj->valid_flags->data[i] == 0) continue;

      error = rox_calibration_projector_perspective_get_statistics(&min, &max, &mean, &med, &std, obj, i);
      ROX_ERROR_CHECK_TERMINATE(error)

      if(max > threshold_reprojection)
      {
         obj->valid_flags->data[i] = 0;
         obj->valid_image--;
         recalibrate = 1;
      }
   }

   if(recalibrate)
   {
      // Restore data
      error = rox_array2d_double_copy(obj->K, obj->K_cpy); ROX_ERROR_CHECK_TERMINATE(error)

      // Init with linearized solution
      error = rox_calibration_projector_perspective_process_linear(obj, Kddl); ROX_ERROR_CHECK_TERMINATE(error)

      // Create data
      for (Rox_Uint i = 0; i < obj->homographies->used; i++)
      {
         // Compute poses
         if(obj->valid_flags->data[i] == 0) continue;

         error = rox_transformtools_build_pose_intermodel(obj->poses->data[i], obj->homographies->data[i], obj->K); 
         ROX_ERROR_CHECK_TERMINATE ( error );
         
         error = rox_array2d_double_copy(obj->poses_cpy->data[i],obj->poses->data[i]); 
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      // Refine with non linear optimization
      error = rox_calibration_projector_perspective_process_nolinear(obj, Kddl); ROX_ERROR_CHECK_TERMINATE(error)

      // Get stats
      for (Rox_Uint i = 0; i < obj->homographies->used; i++)
      {
         if(obj->valid_flags->data[i] == 0) continue;

         error = rox_calibration_projector_perspective_get_statistics(&min, &max, &mean, &med, &std, obj, i);
         ROX_ERROR_CHECK_TERMINATE ( error );

         if (max > threshold_reprojection)
         { error = ROX_ERROR_PROCESS_FAILED; ROX_ERROR_CHECK_TERMINATE ( error ); }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_process_linear(
   Rox_Calibration_Projector_Perspective obj,
   Rox_Uint method )
{
   Rox_ErrorCode error;

   if(obj == 0) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   switch(method)
   {
      case 1:
         {
            error = rox_calibration_projector_perspective_linear_fu(obj);
            break;
         }
      case 2:
         {
            error = rox_calibration_projector_perspective_linear_fu_fv(obj);
            break;
         }
      case 3:
         {
            error = rox_calibration_projector_perspective_linear_fu_cu_cv(obj);
            break;
         }
      case 4:
         {
            error = rox_calibration_projector_perspective_linear_fu_fv_cu_cv(obj);
            break;
         }
      case 5:
         {
            error = rox_calibration_projector_perspective_linear_fu_fv_cu_cv_s(obj);
            break;
         }
      default:
         {
            error = ROX_ERROR_INVALID_VALUE;
            break;
         }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_linear_fu( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double C  = NULL, v = NULL, x = NULL, Gt = NULL;

   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the measure counter
   Rox_Uint nbpose = obj->homographies->used;
   if (nbpose<1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init buffers
   error = rox_calibration_projector_perspective_init_buffers(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&C, 2 * nbpose, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&v, 2 * nbpose, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, 1, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dv  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dv , v);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dC  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC , C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dx  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dx , x);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dS  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dS , obj->S);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC0 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC0 , obj->C0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC1 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC1 , obj->C1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dK  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dK, obj->K);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dKn = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dKn, obj->Kn);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // As the image center is choosen for the normalization, un and vn are always null
   Rox_Double un = (dK[0][2] - dKn[0][2]) / dKn[0][0];
   Rox_Double vn = (dK[1][2] - dKn[1][2]) / dKn[1][1];

   error = rox_array2d_double_scale(obj->A3, obj->A3, -un);
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_scale(obj->A4, obj->A4, -vn);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_add(obj->A0, obj->A0, obj->A2);
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_add(obj->A0, obj->A0, obj->A3);
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_add(obj->A0, obj->A0, obj->A4);
   ROX_ERROR_CHECK_TERMINATE(error)

   for (Rox_Uint i = 0; i < nbpose; i++)
   {
      if(obj->valid_flags->data[i] == 0) continue;

      error = rox_array2d_double_new_subarray2d(&Gt, obj->homographies->data[i], 0, 0, 3, 2); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->M, obj->Kt, Gt);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // We do not need Gt any more, delete
      error = rox_array2d_double_del(&Gt);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C0, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C1, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      dC[2*i  ][0] =  dC1[0][1];
      dC[2*i+1][0] =  dC1[0][0] - dC1[1][1];

      dv[2*i  ][0] = -dC0[0][1];
      dv[2*i+1][0] = -dC0[0][0] + dC0[1][1];

      rox_array2d_double_del(&Gt);
   }

   error = rox_svd_solve(x, C, v); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   dS[0][0] = 1.0; dS[0][1] = 0.0; dS[0][2] = -un;
   dS[1][0] = 0.0; dS[1][1] = 1.0; dS[1][2] = -vn;
   dS[2][0] = -un; dS[2][1] = -vn; dS[2][2] = dx[0][0];
   // Update intrinsics
   error = rox_calibration_projector_perspective_update_linear_instrinsics(obj); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&x);
   rox_array2d_double_del(&v);
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&Gt);

   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_linear_fu_fv( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double C  = NULL, v = NULL, x = NULL, Gt = NULL;

   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the measure counter
   Rox_Uint nbpose = obj->homographies->used;
   if (nbpose < 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init buffers
   error = rox_calibration_projector_perspective_init_buffers(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&C, 2*nbpose, 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&v, 2*nbpose, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );


   Rox_Double ** dv  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dv , v);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dC  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC , C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dx  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dx , x);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dS  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dS , obj->S);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC0 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC0 , obj->C0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC1 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC1 , obj->C1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC2 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC2 , obj->C2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dK  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dK, obj->K);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dKn = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dKn, obj->Kn);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // As the image center is choosen for the normalization, un and vn are always null
   Rox_Double un = (dK[0][2] - dKn[0][2]) / dKn[0][0];
   Rox_Double vn = (dK[1][2] - dKn[1][2]) / dKn[1][1];

   error = rox_array2d_double_scale(obj->A3, obj->A3, -un); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale(obj->A4, obj->A4, -vn); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_add(obj->A0, obj->A0, obj->A3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_add(obj->A2, obj->A2, obj->A4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < nbpose; i++)
   {
      if(obj->valid_flags->data[i] == 0) continue;

      error = rox_array2d_double_new_subarray2d(&Gt, obj->homographies->data[i], 0, 0, 3, 2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->M, obj->Kt, Gt);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // We do not need Gt any more, delete
      error = rox_array2d_double_del(&Gt);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C0, obj->MtA, obj->M);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C1, obj->MtA, obj->M);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C2, obj->MtA, obj->M);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Set C and v
      dC[2*i][0] =  dC1[0][1];
      dC[2*i][1] =  dC2[0][1];

      dC[2*i+1][0] =  dC1[0][0] - dC1[1][1];
      dC[2*i+1][1] =  dC2[0][0] - dC2[1][1];

      dv[2*i  ][0] = -dC0[0][1];
      dv[2*i+1][0] = -dC0[0][0] + dC0[1][1];
   }

   error = rox_svd_solve(x, C, v); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   dS[0][0] = 1.0; dS[0][1] =          0.0; dS[0][2] = -un;
   dS[1][0] = 0.0; dS[1][1] =     dx[1][0]; dS[1][2] = -vn*dx[1][0];
   dS[2][0] = -un; dS[2][1] = -vn*dx[1][0]; dS[2][2] =     dx[0][0];

   // Update intrinsics
   error = rox_calibration_projector_perspective_update_linear_instrinsics(obj); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&x);
   rox_array2d_double_del(&v);
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&Gt);

   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_linear_fu_cu_cv( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double C = NULL, v = NULL, x = NULL, Gt = NULL;

   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the measure counter
   Rox_Uint nbpose = obj->homographies->used;
   if (nbpose < 2)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init buffers
   error = rox_calibration_projector_perspective_init_buffers(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&C, 2*nbpose, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&v, 2*nbpose, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(C, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(v, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );


   Rox_Double ** dv  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dv , v);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dC  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC , C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dx  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dx , x);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dS  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dS , obj->S);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC0 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC0 , obj->C0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC1 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC1 , obj->C1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC2 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC2 , obj->C2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dC3 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC3 , obj->C3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_add(obj->A0, obj->A0, obj->A2); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < nbpose; i++)
   {
      if(obj->valid_flags->data[i] == 0) continue;

      error = rox_array2d_double_new_subarray2d(&Gt, obj->homographies->data[i], 0, 0, 3, 2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->M, obj->Kt, Gt);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // We do not need Gt any more, delete
      error = rox_array2d_double_del(&Gt);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C0, obj->MtA, obj->M);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C1, obj->MtA, obj->M);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A3);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C2, obj->MtA, obj->M);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A4);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C3, obj->MtA, obj->M);
      ROX_ERROR_CHECK_TERMINATE ( error );

      dC[2*i][0] =  dC1[0][1];
      dC[2*i][1] =  dC2[0][1];
      dC[2*i][2] =  dC3[0][1];

      dC[2*i+1][0] =  dC1[0][0] - dC1[1][1];
      dC[2*i+1][1] =  dC2[0][0] - dC2[1][1];
      dC[2*i+1][2] =  dC3[0][0] - dC3[1][1];

      dv[2*i  ][0] = -dC0[0][1];
      dv[2*i+1][0] = -dC0[0][0] + dC0[1][1];
   }

   error = rox_svd_solve(x, C, v); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   dS[0][0] =      1.0; dS[0][1] =      0.0; dS[0][2] = dx[1][0];
   dS[1][0] =      0.0; dS[1][1] =      1.0; dS[1][2] = dx[2][0];
   dS[2][0] = dx[1][0]; dS[2][1] = dx[2][0]; dS[2][2] = dx[0][0];

   // Update intrinsics
   error = rox_calibration_projector_perspective_update_linear_instrinsics(obj); 
   ROX_ERROR_CHECK_TERMINATE ( error );
 
function_terminate:

   rox_array2d_double_del(&v);
   rox_array2d_double_del(&x);
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&Gt);

   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_linear_fu_fv_cu_cv( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double C = NULL, v = NULL, x = NULL, Gt = NULL;

   if(!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the measure counter
   Rox_Uint nbpose = obj->homographies->used;
   if (nbpose<2) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init buffers
   error = rox_calibration_projector_perspective_init_buffers(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&C, 2*nbpose, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&v, 2*nbpose, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, 4, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );


   Rox_Double ** dv  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dv , v);
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   Rox_Double ** dC  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC , C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dx  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dx , x);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dS  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dS , obj->S);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC0 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC0 , obj->C0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC1 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC1 , obj->C1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC2 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC2 , obj->C2);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC3 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC3 , obj->C3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC4 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC4 , obj->C4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < nbpose; i++)
   {
      if(obj->valid_flags->data[i] == 0) continue;

      error = rox_array2d_double_new_subarray2d(&Gt, obj->homographies->data[i], 0, 0, 3, 2); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->M, obj->Kt, Gt); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // We do not need Gt any more, delete
      error = rox_array2d_double_del(&Gt);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C0, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C1, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C2, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A3);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C3, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A4);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C4, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      dC[2*i][0] =  dC1[0][1];
      dC[2*i][1] =  dC2[0][1];
      dC[2*i][2] =  dC3[0][1];
      dC[2*i][3] =  dC4[0][1];

      dC[2*i+1][0] =  dC1[0][0] - dC1[1][1];
      dC[2*i+1][1] =  dC2[0][0] - dC2[1][1];
      dC[2*i+1][2] =  dC3[0][0] - dC3[1][1];
      dC[2*i+1][3] =  dC4[0][0] - dC4[1][1];

      dv[2*i  ][0] = -dC0[0][1];
      dv[2*i+1][0] = -dC0[0][0] + dC0[1][1];
   }

   error = rox_svd_solve(x, C, v); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   dS[0][0] =      1.0; dS[0][1] =      0.0; dS[0][2] = dx[2][0];
   dS[1][0] =      0.0; dS[1][1] = dx[1][0]; dS[1][2] = dx[3][0];
   dS[2][0] = dx[2][0]; dS[2][1] = dx[3][0]; dS[2][2] = dx[0][0];

   // Update intrinsics
   error = rox_calibration_projector_perspective_update_linear_instrinsics(obj); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_array2d_double_del(&x);
   rox_array2d_double_del(&v);
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&Gt);

   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_linear_fu_fv_cu_cv_s( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double C = NULL, v = NULL, x = NULL, Gt = NULL;

   if( obj == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the measure counter
   Rox_Uint nbpose = obj->homographies->used;
   if (nbpose<3) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Init buffers
   error = rox_calibration_projector_perspective_init_buffers(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, 5, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&v, 2*nbpose, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&C, 2*nbpose, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dv  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dv , v);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dC  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC , C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dx  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dx , x);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dS  = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dS , obj->S);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC0 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC0 , obj->C0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC1 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC1 , obj->C1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC2 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC2 , obj->C2);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC3 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC3 , obj->C3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC4 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC4 , obj->C4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dC5 = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dC5 , obj->C5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < nbpose; i++)
   {
      if(obj->valid_flags->data[i] == 0) continue;

      error = rox_array2d_double_new_subarray2d(&Gt, obj->homographies->data[i], 0, 0, 3, 2); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->M, obj->Kt, Gt); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // We do not need Gt any more, delete
      error = rox_array2d_double_del(&Gt);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C0, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(obj->C1, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A2);
      ROX_ERROR_CHECK_TERMINATE ( error );


      error = rox_array2d_double_mulmatmat(obj->C2, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A3);
      ROX_ERROR_CHECK_TERMINATE ( error );


      error = rox_array2d_double_mulmatmat(obj->C3, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A4);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_mulmatmat(obj->C4, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(obj->MtA, obj->M, obj->A5);
      ROX_ERROR_CHECK_TERMINATE ( error );


      error = rox_array2d_double_mulmatmat(obj->C5, obj->MtA, obj->M); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      dC[2*i][0] =  dC1[0][1];
      dC[2*i][1] =  dC2[0][1];
      dC[2*i][2] =  dC3[0][1];
      dC[2*i][3] =  dC4[0][1];
      dC[2*i][4] =  dC5[0][1];

      dC[2*i+1][0] =  dC1[0][0] - dC1[1][1];
      dC[2*i+1][1] =  dC2[0][0] - dC2[1][1];
      dC[2*i+1][2] =  dC3[0][0] - dC3[1][1];
      dC[2*i+1][3] =  dC4[0][0] - dC4[1][1];
      dC[2*i+1][4] =  dC5[0][0] - dC5[1][1];

      dv[2*i][0] = -dC0[0][1];
      dv[2*i+1][0] = -dC0[0][0] + dC0[1][1];
   }

   error = rox_svd_solve(x, C, v); ROX_ERROR_CHECK_TERMINATE(error)

   dS[0][0] =      1.0; dS[0][1] = dx[4][0]; dS[0][2] = dx[2][0];
   dS[1][0] = dx[4][0]; dS[1][1] = dx[1][0]; dS[1][2] = dx[3][0];
   dS[2][0] = dx[2][0]; dS[2][1] = dx[3][0]; dS[2][2] = dx[0][0];

   // Update intrinsics
   error = rox_calibration_projector_perspective_update_linear_instrinsics(obj); ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:

   rox_array2d_double_del(&x);
   rox_array2d_double_del(&v);
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&Gt);

   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_init_buffers( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error;

   Rox_Double **dK  = 0, **dKn = 0;

   if(obj == 0) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Reset all matices
   error = rox_array2d_double_fillval(obj->A0, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->A1, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->A2, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->A3, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->A4, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->A5, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // A0 = {1,0,0, 0,0,0, 0,0,0}
   error = rox_array2d_double_set_value(obj->A0, 0, 0, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // A1 = {0,0,0, 0,0,0, 0,0,1}
   error = rox_array2d_double_set_value(obj->A1, 2, 2, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // A2 = {0,0,0, 0,1,0, 0,0,0}
   error = rox_array2d_double_set_value(obj->A2, 1, 1, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // A3 = {0,0,1, 0,0,0, 1,0,0}
   error = rox_array2d_double_set_value(obj->A3, 0, 2, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(obj->A3, 2, 0, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // A4 = {0,0,0, 0,0,1, 0,1,0}
   error = rox_array2d_double_set_value(obj->A4, 1, 2, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(obj->A4, 2, 1, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // A5 = {0,1,0, 1,0,0, 0,0,0}
   error = rox_array2d_double_set_value(obj->A5, 0, 1, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value(obj->A5, 1, 0, 1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error  = rox_array2d_double_get_data_pointer_to_pointer( &dK, obj->K);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dKn, obj->Kn);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // dk = {1024, 0, cu, 0, 1024, cv, 0 0 1}
   dK[0][0] = 1024.0; dK[1][1] = 1024.0;

   // Normalization matrix. Choose 1024 to get exact division.

   error = rox_array2d_double_fillunit(obj->Kn); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   dKn[0][0] = 1024.0;
   dKn[1][1] = 1024.0;
   dKn[0][2] = dK[0][2];
   dKn[1][2] = dK[1][2];


   error = rox_array2d_double_mat3x3_inverse(obj->Kt, obj->Kn); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_update_linear_instrinsics( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error;
   Rox_Double value;


   if (obj == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_cholesky_decomposition(obj->Kt, obj->S); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Use S as a temporary matrix to store the transpose
   error = rox_array2d_double_transpose(obj->S, obj->Kt); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute K = inv(transpose(Kt))
   error = rox_array2d_double_mat3x3_inverse(obj->K, obj->S); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Normalize the matrix such that K[2][2] = 1
   error = rox_array2d_double_get_value(&value, obj->K, 2, 2); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->K, obj->K, 1.0/value); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Un-normalize solution
   error = rox_array2d_double_mulmatmat(obj->Kt, obj->Kn, obj->K); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->K, obj->Kt); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_process_nolinear(Rox_Calibration_Projector_Perspective obj, Rox_Uint method)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint nbpos, nbpts;
   Rox_Uint miter = 10, Tddl = 6, Kddl = method;
   Rox_Uint start_row = 0, start_col = 0;

   Rox_Point2D_Double pc = 0;
   Rox_Double *zc = 0;

   Rox_Array2D_Double JK = NULL;
   Rox_Array2D_Double JT = NULL;

   Rox_Array2D_Double xK_ddl = NULL;
   Rox_Array2D_Double xT = NULL;
   Rox_Array2D_Double xK = NULL;

   Rox_Array2D_Double x = NULL;
   Rox_Array2D_Double A = NULL;
   Rox_Array2D_Double b = NULL;

   Rox_Double cu = 0.0;
   Rox_Double cv = 0.0;


   if (obj == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (method>6 || method ==0) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbpos = obj->homographies->used;
   if (nbpos<1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbpts = obj->ref2D->used;
   if (nbpts<1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Allocate the vector for the algut3
   error = rox_array2d_double_new(&xK, 6, 1);  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&x, Kddl+Tddl*nbpos, 1);  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&A, 2*nbpts*nbpos, Kddl+Tddl*nbpos); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&b, 2*nbpts*nbpos,1);  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_value(&cu, obj->K, 0, 2); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_value(&cv, obj->K, 1, 2); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Init structures
   pc = (Rox_Point2D_Double )rox_memory_allocate(sizeof(*pc), nbpts);
   if (!pc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   zc = (Rox_Double *)rox_memory_allocate(sizeof(*zc), nbpts);
   if (!zc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Virtual Visual Servoing
   for (Rox_Uint k = 0; k < miter; k++)
   {
      // Reset normal equations A*x = b

      error = rox_array2d_double_fillval(A, 0.0); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_fillval(b, 0.0); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // For each image
      for(Rox_Uint i = 0; i < nbpos; i++)
      {
         if(obj->valid_flags->data[i] == 0) continue;

         // Compute indexes
         start_row = 2 * nbpts * i;
         start_col = Kddl + Tddl * i;

         // Compute the current points and depths

         error = rox_point2d_double_transform_project(pc, zc, obj->K, obj->poses->data[i], obj->obs3D->data[i]->data, nbpts); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Compute the Jacobian matrix for T
         error = rox_array2d_double_new_subarray2d(&JT, A, start_row, start_col, 2*nbpts, Tddl); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_interaction_matse3_point2d_pix(JT, pc, zc, obj->K, nbpts); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Compute the Jacobian matrix for K
         error = rox_array2d_double_new_subarray2d(&JK, A, start_row, 0, 2*nbpts, Kddl); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_jacobian_points_2d_campar(JK, pc, cu, cv, nbpts, Kddl); 
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Build vector b
         for(Rox_Uint j = 0; j < nbpts; j++)
         {
            Rox_Uint row = start_row + j * 2;


            error = rox_array2d_double_set_value(b, row  , 0, pc[j].u - obj->ref2D->data[j].u); 
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_set_value(b, row+1, 0, pc[j].v - obj->ref2D->data[j].v); 
            ROX_ERROR_CHECK_TERMINATE ( error );
         }

         rox_array2d_double_del(&JK);
         rox_array2d_double_del(&JT);
      }

      // Solve x = pinv(A)*b

      error = rox_svd_solve(x, A, b); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update intrinsic parameters and poses
      error = rox_array2d_double_new_subarray2d(&xK_ddl, x, 0, 0, Kddl, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );


      error = rox_array2d_double_scale(xK_ddl, xK_ddl, -1.0); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_campar_ddl ( xK, xK_ddl, cu, cv, Kddl ); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_scale_inplace(xK, -1.0);
      ROX_ERROR_CHECK_TERMINATE( error );


      error = rox_matut3_update_left ( obj->K, xK ); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      rox_array2d_double_del(&xK_ddl);

      for(Rox_Uint i = 0; i < nbpos; i++)
      {
         if(obj->valid_flags->data[i] == 0) continue;

         error = rox_array2d_double_new_subarray2d(&xT, x, Kddl + 6*i, 0, Tddl, 1);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_scale_inplace(xT, -1.0);
         ROX_ERROR_CHECK_TERMINATE( error );

         error = rox_matse3_update_left(obj->poses->data[i], xT);

         ROX_ERROR_CHECK_TERMINATE ( error );

         rox_array2d_double_del(&xT);
      }
   }

function_terminate:

   // Delete data
   rox_memory_delete(pc);
   rox_memory_delete(zc);

   rox_array2d_double_del(&JK);
   rox_array2d_double_del(&JT);

   rox_array2d_double_del(&A);
   rox_array2d_double_del(&b);
   rox_array2d_double_del(&x);

   rox_array2d_double_del(&xK_ddl);
   rox_array2d_double_del(&xT);
   rox_array2d_double_del(&xK);

   return error;
}


#if 0
Rox_ErrorCode rox_calibration_projector_perspective_process_nolinear_normalize(Rox_Calibration_Projector_Perspective obj, Rox_Uint method)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint i, k, nbpos, nbpts;
   Rox_Uint miter = 10, Tddl = 6, Kddl = method;
   Rox_Uint start_row = 0, start_col = 0;

   Rox_Point2D_Double *pr = 0;
   Rox_Point2D_Double *pc = 0;
   Rox_Point2D_Double *qc = 0;
   Rox_Point2D_Double *qr = 0;

   Rox_Double *zc = 0;

   Rox_Array2D_Double JK = NULL;
   Rox_Array2D_Double JT = NULL;

   Rox_Array2D_Double xK_ddl = NULL;
   Rox_Array2D_Double xT = NULL;
   Rox_Array2D_Double xK = NULL;

   Rox_Array2D_Double x = NULL;
   Rox_Array2D_Double A = NULL;
   Rox_Array2D_Double b = NULL;

    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
    if (method>6 || method ==0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   nbpos = obj->homographies->used;
   if (nbpos<1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   nbpts = obj->ref2D->used;
   if (nbpts<1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   // Allocate the vector for the algut3
   error = rox_array2d_double_new(&xK, 6, 1);  ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&x, Kddl+Tddl*nbpos, 1);  ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&A, 2*nbpts*nbpos, Kddl+Tddl*nbpos); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new(&b, 2*nbpts*nbpos,1);  ROX_ERROR_CHECK_TERMINATE(error)

   // Init structures
   pr = (Rox_Point2D_Double*)rox_memory_allocate(sizeof(*pr), nbpts);
   if(!pr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   pc = (Rox_Point2D_Double*)rox_memory_allocate(sizeof(*pc), nbpts);
   if(!pc) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   qr = (Rox_Point2D_Double*)rox_memory_allocate(sizeof(*qr), nbpts);
   if(!qr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   qc = (Rox_Point2D_Double*)rox_memory_allocate(sizeof(*qc), nbpts);
   if(!qc) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   zc = (Rox_Double*)rox_memory_allocate(sizeof(*zc), nbpts);
   if(!zc) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Virtual Visual Servoing
   for(k = 0; k < miter; k++)
   {
      // Reset normal equations A*x = b
      error = rox_array2d_double_fillval(A, 0.0); ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_array2d_double_fillval(b, 0.0); ROX_ERROR_CHECK_TERMINATE(error)

      
      error = rox_array2d_double_mat3x3_inverse(obj->Kt, obj->K); ROX_ERROR_CHECK_TERMINATE ( error );
      // For each image
      for(i = 0; i < nbpos; i++)
      {
         if(obj->valid_flags->data[i] == 0) continue;

         // Compute indexes
         start_row = 2 * nbpts * i;
         start_col = Kddl + Tddl * i;

         // Compute the reference points
         error = rox_point2d_double_homography(pr, obj->model2D->data, obj->homographies->data[i], nbpts); ROX_ERROR_CHECK_TERMINATE(error)

         // Compute the current points and depths
         error = rox_point2d_double_transform_project(pc, zc, obj->K, obj->poses->data[i], obj->model->data, nbpts); ROX_ERROR_CHECK_TERMINATE(error)

         // Compute normalized points from points in pixels
         error = rox_point2d_double_homography(qc, pc, obj->Kt, nbpts); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_point2d_double_homography(qr, pr, obj->Kt, nbpts); ROX_ERROR_CHECK_TERMINATE(error)

         // Compute the Jacobian matrix for T
         error = rox_array2d_double_new_subarray2d(&JT, A, start_row, start_col, 2*nbpts, Tddl); ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_interaction_matse3_point2d_nor(JT, qc, zc, nbpts); ROX_ERROR_CHECK_TERMINATE(error)

         // Compute the Jacobian matrix for K
         error = rox_array2d_double_new_subarray2d(&JK, A, start_row, 0, 2*nbpts, Kddl); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_jacobian_points_2d_campar(JK, qc, 0, 0, nbpts, Kddl); ROX_ERROR_CHECK_TERMINATE(error)

         // Build vector b
         error = rox_array2d_double_set_value(b, start_row+0, 0, qc[0].u - qr[0].u); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_array2d_double_set_value(b, start_row+1, 0, qc[0].v - qr[0].v); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_array2d_double_set_value(b, start_row+2, 0, qc[1].u - qr[1].u); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_array2d_double_set_value(b, start_row+3, 0, qc[1].v - qr[1].v); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_array2d_double_set_value(b, start_row+4, 0, qc[2].u - qr[2].u); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_array2d_double_set_value(b, start_row+5, 0, qc[2].v - qr[2].v); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_array2d_double_set_value(b, start_row+6, 0, qc[3].u - qr[3].u); ROX_ERROR_CHECK_TERMINATE(error)
         error = rox_array2d_double_set_value(b, start_row+7, 0, qc[3].v - qr[3].v); ROX_ERROR_CHECK_TERMINATE(error)

         rox_array2d_double_del(&JK);
         rox_array2d_double_del(&JT);
      }

      // Solve x = pinv(A)*b
      error = rox_svd_solve(x, A, b); ROX_ERROR_CHECK_TERMINATE(error)

      // Update intrinsic parameters and poses
      error = rox_array2d_double_new_subarray2d(&xK_ddl, x, 0, 0, Kddl, 1);ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_campar_ddl(xK, xK_ddl, 0, 0, Kddl);
      ROX_ERROR_CHECK_TERMINATE(error)
      error = rox_matut3_update_left(obj->Kt, xK);
      ROX_ERROR_CHECK_TERMINATE(error)
      rox_array2d_double_del(&xK_ddl);

      error = rox_array2d_double_mat3x3_inverse(obj->K, obj->Kt);
      ROX_ERROR_CHECK_TERMINATE(error)

      for(i = 0; i < nbpos; i++)
      {
         if(obj->valid_flags->data[i] == 0) continue;

         error = rox_array2d_double_new_subarray2d(&xT, x, Kddl + 6*i, 0, Tddl, 1);
         ROX_ERROR_CHECK_TERMINATE(error)

         error = rox_array2d_double_scale_inplace(xT, -1.0);
         ROX_ERROR_CHECK_TERMINATE( error );

         error = rox_matse3_update_left(obj->poses->data[i], xT);
         ROX_ERROR_CHECK_TERMINATE(error)
         rox_array2d_double_del(&xT);
      }
   }

function_terminate:

   // Delete data
   rox_memory_delete(pr);
   rox_memory_delete(pc);
   rox_memory_delete(qr);
   rox_memory_delete(qc);
   rox_memory_delete(zc);

   rox_array2d_double_del(&JK);
   rox_array2d_double_del(&JT);

   rox_array2d_double_del(&A);
   rox_array2d_double_del(&b);
   rox_array2d_double_del(&x);

   rox_array2d_double_del(&xK_ddl);
   rox_array2d_double_del(&xT);
   rox_array2d_double_del(&xK);

   return error;
}
#endif

Rox_ErrorCode rox_calibration_projector_perspective_get_statistics(
      Rox_Double *min,
      Rox_Double *max,
      Rox_Double *mean,
      Rox_Double *median,
      Rox_Double *std,
      Rox_Calibration_Projector_Perspective obj,
      Rox_Uint id )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double reproj = 0;
   Rox_Array2D_Double norm = 0;
   Rox_Double **dn, sum = 0;

   Rox_Uint nbpoints;
   Rox_Double u1, u2, v1, v2;
   Rox_Uint j;

   
   if (!min || !max || !mean || !median || !std || !obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (id > obj->homographies->used - 1) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbpoints = obj->ref2D->used;

   reproj = (Rox_Point2D_Double )rox_memory_allocate(sizeof(*reproj), nbpoints);
   if(!reproj)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   
   error = rox_array2d_double_new(&norm, nbpoints, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dn, norm); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute 3d-2d projection
   error = rox_point3d_double_transform_project(reproj, obj->poses->data[id], obj->K, obj->obs3D->data[id]->data, nbpoints);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Compute norm
   for(j = 0; j < nbpoints; j++)
   {
      // Compute norm
      u1 = reproj[j].u; v1 = reproj[j].v;
      u2 = obj->ref2D->data[j].u; v2 = obj->ref2D->data[j].v;

      dn[j][0] = sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
   }

   // make stats
   error = rox_array2d_double_minmax(min, max, norm); 
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_median(median, norm);
   ROX_ERROR_CHECK_TERMINATE(error)

   sum = 0;
   for(j = 0; j < nbpoints; j++)
   {
      sum += dn[j][0];
   }

   *mean = sum / (Rox_Double)nbpoints;

   sum = 0;
   for(j = 0; j < nbpoints; j++)
   {
      sum += (dn[j][0] - (*mean)) * (dn[j][0] - (*mean));
   }

   *std = sqrt(sum / (Rox_Double)nbpoints);

function_terminate:
   rox_memory_delete(reproj);
   rox_array2d_double_del(&norm);

   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_save_data( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint i;

   if(!obj) 
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Copy intrinsics

   error = rox_array2d_double_copy(obj->K_cpy, obj->K); ROX_ERROR_CHECK_TERMINATE ( error );

   for (i = 0; i < obj->poses->used; i++)
   {
      error = rox_array2d_double_copy(obj->poses_cpy->data[i], obj->poses->data[i]); ROX_ERROR_CHECK_TERMINATE ( error );
   }

   function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_restore_data( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint i;

   if(!obj) 
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Copy intrinsics

   error = rox_array2d_double_copy(obj->K, obj->K_cpy); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (i = 0; i < obj->poses->used; i++)
   {
      error = rox_array2d_double_copy(obj->poses->data[i], obj->poses_cpy->data[i]); ROX_ERROR_CHECK_TERMINATE ( error );
   }

   function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_projector_perspective_check_homographies( Rox_Calibration_Projector_Perspective obj )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double norm = 0;
   Rox_Uint nbimages, nbpoints;

   Rox_Point2D_Double reproj = 0;
   Rox_Point2D_Double ref2D = 0;
   Rox_Double ** dn;

   
   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nbimages = obj->homographies->used;
   nbpoints = obj->ref2D->used;

   reproj = (Rox_Point2D_Double )rox_memory_allocate(sizeof(*reproj), nbpoints);
   if (!reproj)

   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ref2D = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(*ref2D), nbpoints);
   if (!ref2D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&norm, nbpoints, 1); ROX_ERROR_CHECK_TERMINATE ( error );


   error = rox_array2d_double_get_data_pointer_to_pointer( &dn, norm); ROX_ERROR_CHECK_TERMINATE ( error );

   // Rebuild ref2D
   for(Rox_Uint i = 0; i < nbpoints; i++)
   {
      ref2D[i].u = obj->ref2D->data[i].u;
      ref2D[i].v = obj->ref2D->data[i].v;
   }

   for(Rox_Uint i = 0; i < nbimages; i++)
   {
      Rox_Double min, max, mean, sum, med, u1, v1, u2, v2; //var,

      error = rox_point2d_double_homography(reproj, obj->obs2D->data[i]->data, obj->homographies->data[i], nbpoints);
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute norm
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         // Compute norm
         u1 = reproj[j].u; v1 = reproj[j].v;
         u2 = ref2D[j].u; v2 = ref2D[j].v;

         dn[j][0] = sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
      }

      // make stats

      error = rox_array2d_double_minmax(&min, &max, norm); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_median(&med, norm);
      ROX_ERROR_CHECK_TERMINATE ( error );

      sum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         sum += dn[j][0];
      }

      mean = sum / (Rox_Double)nbpoints;

      sum = 0;
      for(Rox_Uint j = 0; j < nbpoints; j++)
      {
         sum += (dn[j][0] - mean) * (dn[j][0] - mean);
      }
      // var = sqrt(sum / (Rox_Double)nbpoints);

      if(max > threshold_reprojection)
      {
         obj->valid_flags->data[i] = 0;
      }
   }

function_terminate:
   rox_memory_delete(reproj);
   rox_memory_delete(ref2D);
   rox_array2d_double_del(&norm);

   return error;
}
