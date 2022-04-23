//==============================================================================
//
//    OPENROX   : File vvs_points_se3.c
//
//    Contents  : Implementation of vvs_points_se3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "vvspointsse3.h"

#include <float.h>
#include <string.h>

#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point3d_float_struct.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/dynvec_point2d_tools.h>

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/array/robust/huber.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/array/scale/scale.h>

#include <baseproc/calculus/linsys/linsys_point2d_pix_matse3_weighted.h>
#include <baseproc/calculus/linsys/linsys_point2d_nor_matse3_weighted.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_vvs_score (
   Rox_Double * ret_score,
   const Rox_Array2D_Double weight,
   const Rox_Array2D_Double verr
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ret_score || !weight || !verr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint count = 0;

   error = rox_array2d_double_get_rows(&count, weight);            
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(verr, count, 1);          
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * de = NULL;
   error = rox_array2d_double_get_data_pointer ( &de, verr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * dw = NULL;
   error = rox_array2d_double_get_data_pointer ( &dw, weight);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double score = 0.0;
   for (Rox_Sint id = 0; id < count; id++)
   {
      Rox_Double val = de[id] * dw[id];
      score += val;
   }

   *ret_score = score / (Rox_Double)(count);

function_terminate:
   return error;
}


Rox_ErrorCode rox_jacobian_se3_flat_solver (

   Rox_Array2D_Double pose, 
   const Rox_Array2D_Double LtL, 
   const Rox_Array2D_Double Lte
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double sol = NULL, iLtL = NULL;

   if (!LtL || !Lte || !pose)
   { error = ROX_ERROR_NONE; ROX_ERROR_CHECK_TERMINATE ( error ); }


   error = rox_array2d_double_check_size(pose, 4, 4);               
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(LtL, 6, 6);                
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(Lte, 6, 1);                
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&sol, 6, 1);                      
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&iLtL, 6, 6);                     
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svdinverse(iLtL, LtL);                
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(sol, iLtL, Lte);            
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // In all strictness we should multiply sol by -1.0 
   error = rox_array2d_double_scale_inplace(sol, -1.0);
   ROX_ERROR_CHECK_TERMINATE( error );
   
   error = rox_matse3_update_left(pose, sol);                       
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_array2d_double_del(&iLtL);
   rox_array2d_double_del(&sol);

   return error;
}


Rox_ErrorCode rox_points_float_refine_pose_vvs(
   Rox_Array2D_Double       pose,
   Rox_Array2D_Double       calib,
   Rox_DynVec_Point2D_Float measures,
   Rox_DynVec_Point3D_Float references,
   Rox_Double               maxdist_prefilter 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double              LtL=NULL;
   Rox_Array2D_Double              Lte=NULL;
   Rox_Array2D_Double              diff=NULL, subdiff=NULL;
   Rox_Array2D_Double              weight=NULL;
   Rox_Array2D_Double              work1=NULL;
   Rox_Array2D_Double              work2=NULL;
   Rox_Array2D_Double              dist=NULL, subdist=NULL;
   Rox_DynVec_Point3D_Float mreft = NULL;
   Rox_DynVec_Point2D_Float mrefp = NULL;
   Rox_Uint                        nb_points=0, idpt=0, countvalid=0;
   Rox_Double                        sqsum1=0.0, sqsum2=0.0, delta=0.0, invcount=0.0;
   Rox_Double                        *ddist=NULL;

   const Rox_Uint max_iter = 100;

   if (!pose || !calib || !measures || !references)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nb_points = measures->used;
   if (nb_points < 4)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (references->used != nb_points)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(  pose, 4, 4 );           
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size( calib, 3, 3 );           
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_float_new( &mreft, 100 );            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_float_new( &mrefp, 100 );            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(  &LtL,           6, 6 );        
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(  &Lte,           6, 1 );        
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &diff, 2*nb_points, 1 );        
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new( &dist,   nb_points, 1 );        
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate dynamic
   mreft->used = 0;
   mrefp->used = 0;
   rox_dynvec_point3d_float_usecells(mreft, nb_points);
   rox_dynvec_point2d_float_usecells(mrefp, nb_points);

   // Iterate minimization
   sqsum1   = 0.0;
   sqsum2   = 0.0;
   invcount = 1.0 / (Rox_Double)(references->used);

   error = rox_array2d_double_get_data_pointer ( &ddist, dist );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute distances
   error = rox_point3d_float_transform(mreft->data, pose, references->data, references->used);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point2d_float_project(mrefp->data, mreft->data, calib, mreft->used);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_double_distance_point2d_float(dist, mrefp, measures);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Reorder point to keep only the close one
   countvalid = 0;
   for (idpt = 0; idpt < measures->used; idpt++)
   {
      if (ddist[idpt] < maxdist_prefilter)
      {
         references->data[countvalid] = references->data[idpt];
         measures->data[countvalid] = measures->data[idpt];
         countvalid++;
      }
   }

   // At least 4 points are needed for visual servoing
   if (countvalid < 4)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   references->used = countvalid;
   measures->used   = countvalid;
   mreft->used      = countvalid;
   mrefp->used      = countvalid;

   error = rox_array2d_double_new(  &weight,   countvalid, 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(   &work1,   countvalid, 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(   &work2,   countvalid, 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &subdiff, 2*countvalid, 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &subdist,   countvalid, 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint iter = 0; iter < max_iter; iter++)
   {
      // Compute points projection
      error = rox_point3d_float_transform(mreft->data, pose, references->data, references->used);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point2d_float_project(mrefp->data, mreft->data, calib, mreft->used);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Re-Compute distances
      error = rox_array2d_point2d_double_distance_point2d_float(subdist, mrefp, measures);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Linear algebra
      error = rox_array2d_double_tukey(weight, work1, work2, subdist);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute difference
      error = rox_array2d_point2d_double_difference_point2d_float(subdiff, mrefp, measures);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Score estimation
      error = rox_vvs_score(&sqsum2, weight, subdist);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Exit criteria
      delta = fabs(sqsum2 - sqsum1) * invcount;
      sqsum1 = sqsum2;
      if (delta < 1e-8 && iter > 0)
         break;

      error = rox_jacobian_se3_from_points_pixels_weighted_premul_float(LtL, Lte, subdiff, weight, mreft, calib);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_jacobian_se3_flat_solver(pose, LtL, Lte);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Destroy buffers

function_terminate:

   rox_array2d_double_del(&LtL);
   rox_array2d_double_del(&Lte);
   rox_array2d_double_del(&diff);
   rox_array2d_double_del(&dist);
   rox_array2d_double_del(&weight);
   rox_array2d_double_del(&work1);
   rox_array2d_double_del(&work2);
   rox_array2d_double_del(&subdiff);
   rox_array2d_double_del(&subdist);
   rox_dynvec_point2d_float_del(&mrefp);
   rox_dynvec_point3d_float_del(&mreft);

   return error;
}


Rox_ErrorCode rox_points_double_refine_pose_vvs (
   Rox_Array2D_Double pose,
   Rox_Array2D_Double calib,
   Rox_DynVec_Point2D_Double measures,
   Rox_DynVec_Point3D_Double references,
   Rox_Double maxdist_prefilter
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double LtL = NULL;
   Rox_Array2D_Double Lte = NULL;
   Rox_Array2D_Double diff = NULL, subdiff = NULL;
   Rox_Array2D_Double weight = NULL;
   Rox_Array2D_Double work1 = NULL;
   Rox_Array2D_Double work2 = NULL;
   Rox_Array2D_Double dist = NULL, subdist = NULL;
   Rox_DynVec_Point3D_Double mreft = NULL;
   Rox_DynVec_Point2D_Double mrefp = NULL;
   Rox_Uint nb_points = 0, idpt = 0, countvalid = 0;
   Rox_Double sqsum1 = 0.0, sqsum2 = 0.0, delta = 0.0, invcount = 0.0;
   Rox_Double * ddist = NULL;

   const Rox_Uint max_iter = 100;

   if ( !pose || !calib || !measures || !references )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   nb_points = measures->used;
   if (nb_points < 4)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (references->used != nb_points)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size (pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size (calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_double_new (&mreft, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_new (&mrefp, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &LtL, 6, 6 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &Lte, 6, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &diff, nb_points * 2, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new ( &dist, nb_points, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate dynamic
   mreft->used = 0;
   mrefp->used = 0;
   rox_dynvec_point3d_double_usecells ( mreft, nb_points );
   rox_dynvec_point2d_double_usecells ( mrefp, nb_points );

   // Iterate minimization
   sqsum1 = 0.0;
   sqsum2 = 0.0;
   invcount = 1.0 / (Rox_Double)(references->used);

   error = rox_array2d_double_get_data_pointer ( &ddist, dist );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute distances
   error = rox_point3d_double_transform ( mreft->data, pose, references->data, references->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point2d_double_project ( mrefp->data, mreft->data, calib, mreft->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_double_distance_point2d_double ( dist, mrefp, measures );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Reorder point to keep only the closest one
   countvalid = 0;
   for (idpt = 0; idpt < measures->used; idpt++)
   {
      if (ddist[idpt] < maxdist_prefilter)
      {
         references->data[countvalid] = references->data[idpt];
         measures->data[countvalid] = measures->data[idpt];
         countvalid++;
      }
   }

   // At least 4 points are needed for visual servoing
   if (countvalid < 4)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   references->used = countvalid;
   measures->used = countvalid;
   mreft->used = countvalid;
   mrefp->used = countvalid;

   error = rox_array2d_double_new (&weight, countvalid, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new (&work1, countvalid, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new (&work2, countvalid, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new (&subdiff, countvalid * 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new (&subdist, countvalid, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint iter = 0; iter < max_iter; iter++)
   {
      // Compute points projection
      error = rox_point3d_double_transform (mreft->data, pose, references->data, references->used);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point2d_double_project (mrefp->data, mreft->data, calib, mreft->used);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Re-Compute distances
      error = rox_array2d_point2d_double_distance_point2d_double (subdist, mrefp, measures);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Linear algebra
      error = rox_array2d_double_tukey ( weight, work1, work2, subdist );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute difference
      error = rox_array2d_point2d_double_difference_point2d_double ( subdiff, mrefp, measures );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Score estimation
      error = rox_vvs_score(&sqsum2, weight, subdist);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Exit criteria
      delta = fabs(sqsum2 - sqsum1) * invcount;
      sqsum1 = sqsum2;
      if (delta < 1e-8 && iter > 0) break;

      error = rox_jacobian_se3_from_points_pixels_weighted_premul_double ( LtL, Lte, subdiff, weight, mreft, calib );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_jacobian_se3_flat_solver ( pose, LtL, Lte );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

// Destroy buffers
function_terminate:

   rox_array2d_double_del(&LtL);
   rox_array2d_double_del(&Lte);
   rox_array2d_double_del(&diff);
   rox_array2d_double_del(&dist);
   rox_array2d_double_del(&weight);
   rox_array2d_double_del(&work1);
   rox_array2d_double_del(&work2);
   rox_array2d_double_del(&subdiff);
   rox_array2d_double_del(&subdist);
   rox_dynvec_point2d_double_del(&mrefp);
   rox_dynvec_point3d_double_del(&mreft);

   return error;
}
