//==============================================================================
//
//    OPENROX   : File pointcloud_similarity.c
//
//    Contents  : Implementation of pointcloud_similarity module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "pointcloud_similarity.h"

#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/determinant/detgl3.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/substract/substract.h>
#include <inout/system/errors_print.h>

#include <generated/dynvec_point3d_double_struct.h>

Rox_ErrorCode rox_pointcloud_similarity(Rox_Array2D_Double similarity, Rox_DynVec_Point3D_Double reference3D, Rox_DynVec_Point3D_Double current3D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Double_Struct meanref, meancur;
   Rox_Point3D_Double_Struct difmeanref, difmeancur;
   Rox_Array2D_Double covariance;
   Rox_Array2D_Double U,D,V,S,R,T,bufcol,buffer;
   Rox_Uint id;
   Rox_Double ** dc, **ds, **dd;
   Rox_Double det, trace, varcur, scale;

   // y == ref, x == cur
   if (similarity == NULL || current3D == NULL || reference3D == NULL)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(similarity, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (current3D->used != reference3D->used) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   covariance = NULL;
   U = NULL;
   S = NULL;
   V = NULL;
   D = NULL;
   R = NULL;
   T = NULL;
   buffer = NULL;
   bufcol = NULL;

   error = rox_array2d_double_new(&covariance, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&U, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&S, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&V, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&D, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&buffer, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&R, similarity, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&T, similarity, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&bufcol, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dc, covariance);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &ds, S);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_get_data_pointer_to_pointer( &dd, D);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute pointclouds centroids
   meanref.X = 0;
   meanref.Y = 0;
   meanref.Z = 0;
   meancur.X = 0;
   meancur.Y = 0;
   meancur.Z = 0;
   for (id = 0; id < current3D->used; id++)
   {
      meanref.X += current3D->data[id].X;
      meanref.Y += current3D->data[id].Y;
      meanref.Z += current3D->data[id].Z;
      meancur.X += reference3D->data[id].X;
      meancur.Y += reference3D->data[id].Y;
      meancur.Z += reference3D->data[id].Z;

   }
   meanref.X /= (double) current3D->used;
   meanref.Y /= (double) current3D->used;
   meanref.Z /= (double) current3D->used;
   meancur.X /= (double) current3D->used;
   meancur.Y /= (double) current3D->used;
   meancur.Z /= (double) current3D->used;

   // Compute point clouds variances
   //varref = 0;
   varcur = 0;
   rox_array2d_double_fillval(covariance, 0);
   for (id = 0; id < current3D->used; id++)
   {
      difmeanref.X = current3D->data[id].X - meanref.X;
      difmeanref.Y = current3D->data[id].Y - meanref.Y;
      difmeanref.Z = current3D->data[id].Z - meanref.Z;
      difmeancur.X = reference3D->data[id].X - meancur.X;
      difmeancur.Y = reference3D->data[id].Y - meancur.Y;
      difmeancur.Z = reference3D->data[id].Z - meancur.Z;

      dc[0][0] += difmeanref.X * difmeancur.X;
      dc[0][1] += difmeanref.X * difmeancur.Y;
      dc[0][2] += difmeanref.X * difmeancur.Z;
      dc[1][0] += difmeanref.Y * difmeancur.X;
      dc[1][1] += difmeanref.Y * difmeancur.Y;
      dc[1][2] += difmeanref.Y * difmeancur.Z;
      dc[2][0] += difmeanref.Z * difmeancur.X;
      dc[2][1] += difmeanref.Z * difmeancur.Y;
      dc[2][2] += difmeanref.Z * difmeancur.Z;

      //varref += difmeanref.X * difmeanref.X;
      //varref += difmeanref.Y * difmeanref.Y;
      //varref += difmeanref.Z * difmeanref.Z;
      varcur += difmeancur.X * difmeancur.X;
      varcur += difmeancur.Y * difmeancur.Y;
      varcur += difmeancur.Z * difmeancur.Z;
   }

   // Normalize covariance
   // varref = varref / (double)current3D->used;
   varcur = varcur / (double)current3D->used;
   rox_array2d_double_scale(covariance, covariance, 1.0 / ((double)current3D->used));

   // Get covariance determinant
   error = rox_array2d_double_detgl3(&det, covariance);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Decompose covariance matrix
   error = rox_array2d_double_svd(U,D,V,covariance);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build S
   rox_array2d_double_fillunit(S);
   if (det < 0.0)
   {
      ds[2][2] = -1;
   }

   // Build R
   error = rox_array2d_double_mulmatmattrans(buffer, S, V);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(R, U, buffer);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build scale
   trace = ds[0][0] * dd[0][0] + ds[1][1] * dd[1][0] + ds[2][2] * dd[2][0];
   scale = trace / varcur;

   // Build translation
   rox_array2d_double_set_value(T, 0, 0, meancur.X);
   rox_array2d_double_set_value(T, 1, 0, meancur.Y);
   rox_array2d_double_set_value(T, 2, 0, meancur.Z);
   rox_array2d_double_mulmatmat(bufcol, R, T);
   rox_array2d_double_scale(bufcol, bufcol, scale);
   rox_array2d_double_set_value(T, 0, 0, meanref.X);
   rox_array2d_double_set_value(T, 1, 0, meanref.Y);
   rox_array2d_double_set_value(T, 2, 0, meanref.Z);
   rox_array2d_double_substract(T, T, bufcol);

   rox_array2d_double_scale(R, R, scale);

function_terminate: 
   rox_array2d_double_del(&covariance);
   rox_array2d_double_del(&U);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&D);
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&T);
   rox_array2d_double_del(&bufcol);
   rox_array2d_double_del(&buffer);

   return error;
}
