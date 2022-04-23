//==============================================================================
//
//    OPENROX   : File vvs_se3_so3z.c
//
//    Contents  : Implementation of vvs_se3_so3z module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "vvs_se3_so3z.h"

#include <float.h>
#include <string.h>

#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point3d_float_struct.h>

#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/point/dynvec_point2d_tools.h>
#include <baseproc/geometry/transforms/transform_tools.h>

#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/scale/scale.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/maths_macros.h>

#include <baseproc/calculus/linsys/linsys_point2d_nor_matse3_matso3z.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_intmat_se3_so3z_flat_solver (
   Rox_Array2D_Double cTr,
   Rox_Array2D_Double Tr,
   Rox_Array2D_Double LtL,
   Rox_Array2D_Double Ltd )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double sol = NULL, solcTr = NULL, solTr = NULL, iLtL = NULL;
   Rox_Double **dsol = NULL, **dsolTr = NULL;

   if ( !LtL || !Ltd || !cTr || !Tr )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(cTr, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size( Tr, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(LtL, 7, 7); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(Ltd, 7, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   //XXX Some perfs improvement could be achieved here
   error = rox_array2d_double_new(&sol   , 7 , 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&iLtL  , 7 , 7); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&solTr , 6 , 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dsol, sol );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(solTr, 0);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer(&dsolTr, solTr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Solve
   error = rox_array2d_double_svdinverse(iLtL, LtL);     
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(sol, iLtL, Ltd); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_array2d_double_new_subarray2d( &solcTr, sol, 0, 0, 6, 1 );
   dsolTr[5][0] = dsol[6][0];

   // In all strictness we should multiply sol, solcTr and solTr by -1.0

   error = rox_array2d_double_scale_inplace(solcTr, -1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_update_left(cTr, solcTr); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale_inplace(solTr, -1.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_update_left( Tr,  solTr); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del( &solcTr );
   rox_array2d_double_del( &iLtL );
   rox_array2d_double_del( &sol );
   rox_array2d_double_del( &solTr );

   return error;
}

Rox_ErrorCode rox_points_float_refine_pose_vvs_se3_so3z(
   Rox_Array2D_Double cTr,
   Rox_Array2D_Double Tr,
   Rox_DynVec_Point2D_Float qr,
   Rox_DynVec_Point3D_Float mr,
   Rox_DynVec_Point2D_Float ql,
   Rox_DynVec_Point3D_Float ml,
   Rox_Double maxdist_prefilter )
{
   // Declare variables
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Uint max_iter=1000;
   Rox_Uint iter=0;
   Rox_Array2D_Double LtL=NULL;
   Rox_Array2D_Double Ltd=NULL;
   Rox_Array2D_Double diffr=NULL, diffl=NULL, subdiffr=NULL, subdiffl=NULL;
   Rox_Array2D_Double distr=NULL, distl=NULL, subdistr=NULL, subdistl=NULL;
   Rox_Array2D_Double weightr=NULL, weightl=NULL;
   Rox_Array2D_Double work1r=NULL, work1l=NULL;
   Rox_Array2D_Double work2r=NULL, work2l=NULL;
   Rox_Array2D_Double cRr=NULL;
   Rox_Array2D_Double K=NULL; // "Not" necessary, but existing functions to project need it
   Rox_Array2D_Double permute_slice=NULL;
   Rox_Array2D_Double perm_slice_rot=NULL;
   Rox_Array2D_Double psr=NULL;

   Rox_DynVec_Point3D_Float mrc=NULL, mlc=NULL, mlr=NULL, mltmp=NULL;
   Rox_DynVec_Point2D_Float qrc=NULL, qlc=NULL;

   Rox_Uint nr_points=0, nl_points=0, idpt=0, countvalidr=0, countvalidl=0;
   Rox_Double sqsum1r=0.0, sqsum1l=0.0, sqsum2r=0.0, sqsum2l=0.0, delta=0.0, invcount=0.0;
   Rox_Double *ddist=NULL;

   // Check input variables
   if ( !cTr || !Tr || !mr || !ml || !qr || !ql )
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   nr_points = qr->used;
   nl_points = ql->used;
   if ( ( nr_points + nl_points ) < 4 ) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if ( mr->used != nr_points)          { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if ( ml->used != nl_points)          { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size( cTr, 4, 4); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_check_size( Tr, 4, 4);  ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate declared memory
   error = rox_dynvec_point3d_float_new( &mrc,   100 );          ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point3d_float_new( &mlc,   100 );          ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point3d_float_new( &mlr,   100 );          ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point3d_float_new( &mltmp, 100 );          ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_float_new( &qrc,   100 );          ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_float_new( &qlc,   100 );          ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &diffr , nr_points * 2 , 1 ); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &distr , nr_points     , 1 ); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &diffl , nl_points * 2 , 1 ); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &distl , nl_points     , 1 ); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &LtL   , 7 , 7 );             ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &Ltd   , 7 , 1 );             ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &K     , 3 , 3 );             ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &permute_slice  , 3 , 3 );    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &perm_slice_rot , 4 , 4 );    ROX_ERROR_CHECK_TERMINATE ( error );

   // Fill K mat 
   error = rox_matut3_set_unit ( K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate right points utility storage
   mrc->used = 0;
   qrc->used = 0;
   error = rox_dynvec_point3d_float_usecells( mrc, nr_points ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_float_usecells( qrc, nr_points ); ROX_ERROR_CHECK_TERMINATE(error)

   // Allocate left points utility storage
   mlc->used   = 0;
   mlr->used   = 0;
   mltmp->used = 0;
   qlc->used   = 0;
   error = rox_dynvec_point3d_float_usecells( mlc  , nl_points ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point3d_float_usecells( mlr  , nl_points ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point3d_float_usecells( mltmp, nl_points ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_float_usecells( qlc  , nl_points ); ROX_ERROR_CHECK_TERMINATE(error)

   // Initialize minimization control variables
   sqsum1r   = 0.0;
   sqsum2r   = 0.0;
   sqsum1l   = 0.0;
   sqsum2l   = 0.0;
   invcount  = 1.0 / (Rox_Double)( mr->used + ml->used );

   // Compute right points distances
   error = rox_point3d_float_transform( mrc->data, cTr, mr->data, mr->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point2d_float_project( qrc->data, mrc->data, K, mrc->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_double_distance_point2d_float( distr, qrc, qr );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute left points distances
   // Compute left points correction in R frame
   error = rox_point3d_float_transform( mlr->data, Tr, ml->data, ml->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute left points transformation in C frame
   error = rox_point3d_float_transform( mlc->data, cTr, mlr->data, mlr->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute left points projection
   error = rox_point2d_float_project( qlc->data, mlc->data, K, mlc->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_double_distance_point2d_float( distl, qlc, ql );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Reorder right points to keep only the close ones
   error = rox_array2d_double_get_data_pointer ( &ddist, distr );
   ROX_ERROR_CHECK_TERMINATE ( error );

   countvalidr = 0;
   for (idpt = 0; idpt < qr->used; idpt++)
      if (ddist[idpt] < maxdist_prefilter)
      {
         mr->data[countvalidr] = mr->data[idpt];
         qr->data[countvalidr] = qr->data[idpt];
         countvalidr++;
      }

   mr->used  = countvalidr;
   qr->used  = countvalidr;
   mrc->used = countvalidr;
   qrc->used = countvalidr;

   // Reorder left points to keep only the close ones
   error = rox_array2d_double_get_data_pointer ( &ddist, distl);
   ROX_ERROR_CHECK_TERMINATE ( error );

   countvalidl = 0;
   for (idpt = 0; idpt < ql->used; idpt++)
      if (ddist[idpt] < maxdist_prefilter)
      {
         ml->data[countvalidl] = ml->data[idpt];
         ql->data[countvalidl] = ql->data[idpt];
         countvalidl++;
      }

   ml->used    = countvalidl;
   ql->used    = countvalidl;
   mlc->used   = countvalidl;
   mlr->used   = countvalidl;
   mltmp->used = countvalidl;
   qlc->used   = countvalidl;

   // Check there are still enough constraints
   if ( ( countvalidr + countvalidl ) <= 4 )
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Resize containers based on the count of right valid points
   error = rox_array2d_double_new( &weightr  , countvalidr     , 1 ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &work1r   , countvalidr     , 1 ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &work2r   , countvalidr     , 1 ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &subdiffr , countvalidr * 2 , 1 ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &subdistr , countvalidr     , 1 ); ROX_ERROR_CHECK_TERMINATE(error)

   // Resize containers based on the count of left valid points
   error = rox_array2d_double_new( &weightl  , countvalidl     , 1 ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &work1l   , countvalidl     , 1 ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &work2l   , countvalidl     , 1 ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &subdiffl , countvalidl * 2 , 1 ); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_new( &subdistl , countvalidl     , 1 ); ROX_ERROR_CHECK_TERMINATE(error)

   // Prepare some permutation and rotation matrix data
   // Set permute-slicing matrix
   error = rox_array2d_double_fillval( permute_slice, 0.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( permute_slice, 0, 1, 1.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( permute_slice, 1, 0, -1.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get access to the rotation part of cTr
   error = rox_array2d_double_new_subarray2d( &cRr, cTr, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Prepare a 4x4 matrix to slice, permute and rotate
   error = rox_array2d_double_fillval( perm_slice_rot, 0.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_set_value( perm_slice_rot, 3, 3, 1.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Access the first 3x3 part of the permute_slice_rot matrix
   error = rox_array2d_double_new_subarray2d( &psr, perm_slice_rot, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Iterative minimization
   for ( iter = 0; iter < max_iter; iter++ )
   {
      // Compute right points projection
      error = rox_point3d_float_transform( mrc->data, cTr, mr->data, mr->used );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point2d_float_project( qrc->data, mrc->data, K, mrc->used );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute left points correction in R frame
      error = rox_point3d_float_transform( mlr->data, Tr, ml->data, ml->used );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute left points projection
      error = rox_point3d_float_transform( mlc->data, cTr, mlr->data, mlr->used );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point2d_float_project( qlc->data, mlc->data, K, mlc->used );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute artefact mltmp
      error = rox_array2d_double_mulmatmat( psr, cRr, permute_slice );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_point3d_float_transform( mltmp->data, perm_slice_rot, mlr->data, mlr->used );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Re-Compute right distances
      error = rox_array2d_point2d_double_distance_point2d_float( subdistr, qrc, qr );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Re-Compute left distances
      error = rox_array2d_point2d_double_distance_point2d_float( subdistl, qlc, ql );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Author of these lines thinks computing tukey weights independently
      // for right and left points makes sense since both sets will probably
      // exhibit different error scales. Would it just be for the left points
      // being subjects to the Tr unknown.

      // Compute robust weights for right points
      error = rox_array2d_double_tukey( weightr, work1r, work2r, subdistr );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute robust weights for left points
      error = rox_array2d_double_tukey( weightl, work1l, work2l, subdistl );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute right points errors
      error = rox_array2d_point2d_double_difference_point2d_float( subdiffr, qrc, qr );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute left points errors
      error = rox_array2d_point2d_double_difference_point2d_float( subdiffl, qlc, ql );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Right points score estimation
      error = rox_vvs_score( &sqsum2r, weightr, subdistr );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Left points score estimation
      error = rox_vvs_score( &sqsum2l, weightl, subdistl );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Check convergence
      delta   = fabs( (sqsum2r + sqsum2l) - (sqsum1r + sqsum1l) ) * invcount;
      sqsum1r = sqsum2r;
      sqsum1l = sqsum2l;
      if ( (delta < 1e-10) && (iter > 0) )
         break;

      // Compute interaction matrix and prepare its pseudo-inversion
      error = rox_intmat_se3_so3z_weighted_premul_float(
         LtL, Ltd, subdiffr, weightr, mrc, subdiffl, weightl, mlc, mltmp );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Solve problem
      error = rox_intmat_se3_so3z_flat_solver( cTr, Tr, LtL, Ltd );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   
function_terminate:
   rox_array2d_double_del( &psr );
   rox_array2d_double_del( &cRr );
   rox_array2d_double_del( &LtL );
   rox_array2d_double_del( &Ltd );
   rox_array2d_double_del( &K );
   rox_array2d_double_del( &permute_slice );
   rox_array2d_double_del( &perm_slice_rot );

   rox_array2d_double_del( &diffr );
   rox_array2d_double_del( &distr );
   rox_array2d_double_del( &weightr );
   rox_array2d_double_del( &work1r );
   rox_array2d_double_del( &work2r );
   rox_array2d_double_del( &subdiffr );
   rox_array2d_double_del( &subdistr );

   rox_array2d_double_del( &diffl );
   rox_array2d_double_del( &distl );
   rox_array2d_double_del( &weightl );
   rox_array2d_double_del( &work1l );
   rox_array2d_double_del( &work2l );
   rox_array2d_double_del( &subdiffl );
   rox_array2d_double_del( &subdistl );

   rox_dynvec_point2d_float_del( &qrc );
   rox_dynvec_point2d_float_del( &qlc );

   rox_dynvec_point3d_float_del( &mrc );
   rox_dynvec_point3d_float_del( &mlc );
   rox_dynvec_point3d_float_del( &mlr );
   rox_dynvec_point3d_float_del( &mltmp );

   return error;
}
