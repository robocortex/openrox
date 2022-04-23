//==============================================================================
//
//    OPENROX   : File vvs_se3_so3z_so3z.c
//
//    Contents  : Implementation of vvs_se3_so3z_so3z module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "vvs_se3_so3z_so3z.h"

#include <float.h>

#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point3d_float_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/point/dynvec_point2d_tools.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/maths/linalg/matse3.h>

#include <baseproc/calculus/linsys/linsys_point2d_nor_matse3_matso3z_matso3z.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_intmat_se3_so3z_so3z_flat_solver
(
   Rox_MatSE3 cTb,
   Rox_MatSE3 bTg,
   Rox_MatSE3 pTs,
   Rox_Matrix LtL,
   Rox_Matrix Ltd 
)
{
   Rox_ErrorCode      error = ROX_ERROR_NONE;
   Rox_Array2D_Double sol=NULL, sol_cTb=NULL, sol_bTg=NULL, sol_pTs=NULL, iLtL=NULL;
   Rox_Double           **dsol=NULL, **dsol_bTg=NULL, **dsol_pTs=NULL;
   Rox_Double         lambda=0.9;

   if ( !LtL || !Ltd || !cTb || !bTg || !pTs )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( cTb ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_check_size ( bTg ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matse3_check_size ( pTs ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(LtL, 8, 8); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_check_size(Ltd, 8, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   //XXX Some perfs improvement could be achieved here
   error = rox_array2d_double_new(&sol  , 8 , 1);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&iLtL , 8 , 8);   
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&sol_bTg, 6 , 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&sol_pTs, 6 , 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dsol, sol);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(sol_bTg, 0);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dsol_bTg, sol_bTg );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(sol_pTs, 0);   
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer(&dsol_pTs, sol_pTs);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Solve
   error = rox_array2d_double_svdinverse(iLtL, LtL);     ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(sol, iLtL, Ltd); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale( sol, sol, lambda ); ROX_ERROR_CHECK_TERMINATE ( error );

   rox_array2d_double_new_subarray2d( &sol_cTb, sol, 0, 0, 6, 1 );
   dsol_bTg[5][0] = dsol[6][0];
   dsol_pTs[5][0] = dsol[7][0];

   error = rox_array2d_double_scale( sol_pTs, sol_pTs, -1.0 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // In all strictness we should multiply sol, sol_cTb and sol_bTg by -1.0 but not sol_pTs

   error = rox_array2d_double_scale_inplace(sol_cTb, -1.0);
   ROX_ERROR_CHECK_TERMINATE( error );
      
   error = rox_matse3_update_left( cTb, sol_cTb); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_scale_inplace(sol_bTg, -1.0);
   ROX_ERROR_CHECK_TERMINATE( error );
      
   error = rox_matse3_update_left( bTg, sol_bTg); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_matse3_update_right(pTs, sol_pTs); 
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_double_del( &sol_cTb );
   rox_array2d_double_del( &iLtL    );
   rox_array2d_double_del( &sol     );
   rox_array2d_double_del( &sol_bTg );
   rox_array2d_double_del( &sol_pTs );

   return error;
}

Rox_ErrorCode rox_points_float_refine_pose_vvs_se3_so3z_so3z
(
   Rox_MatSE3       cTb,
   Rox_MatSE3       bTg,
   Rox_MatSE3       pTs,
   Rox_MatSE3       gTp,
   Rox_DynVec_Point2D_Float qb,
   Rox_DynVec_Point3D_Float mb,
   Rox_DynVec_Point2D_Float qs,
   Rox_DynVec_Point3D_Float ms,
   Rox_Double               maxdist_prefilter  
)
{
   // Declare variables 
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Uint max_iter=1000;
   //const Rox_Uint max_iter=2;
   Rox_Uint iter=0;
   Rox_Array2D_Double LtL=NULL;
   Rox_Array2D_Double Ltd=NULL;
   Rox_Array2D_Double diff_b=NULL, diff_s=NULL, subdiff_b=NULL, subdiff_s=NULL;
   Rox_Array2D_Double dist_b=NULL, dist_s=NULL, subdist_b=NULL, subdist_s=NULL;
   Rox_Array2D_Double weight_b=NULL, weight_s=NULL;
   Rox_Array2D_Double work1_b=NULL, work1_s=NULL;
   Rox_Array2D_Double work2_b=NULL, work2_s=NULL;
   Rox_Array2D_Double cRb = NULL;
   Rox_Array2D_Double gTs = NULL, bTs = NULL, cTs = NULL, cRs = NULL;
   Rox_Array2D_Double K=NULL; // "Not" necessary, but existing functions to project need it
   Rox_Array2D_Double perm_slice_g=NULL, perm_slice_s=NULL;
   Rox_Array2D_Double perm_slice_rot=NULL;
   Rox_Array2D_Double psr=NULL;

   Rox_DynVec_Point3D_Float mb_c = NULL;
   Rox_DynVec_Point3D_Float ms_b = NULL, ms_c = NULL, msu_g = NULL, msu_s = NULL;
   Rox_DynVec_Point2D_Float qb_c = NULL, qs_c = NULL;

   Rox_Uint nb_points=0, ns_points=0, idpt=0, countvalidb=0, countvalids=0;
   Rox_Double sqsum1b=0.0, sqsum1s=0.0, sqsum2b=0.0, sqsum2s=0.0, delta=0.0, invcount=0.0;
   Rox_Double * ddist = NULL;

   // Check input variables 
   if ( !cTb || !bTg || !gTp || !pTs || !mb || !ms || !qb || !qs )
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   nb_points = qb->used;
   ns_points = qs->used;

   if ( ( nb_points + ns_points ) < 4 ) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( mb->used != nb_points)          
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if ( ms->used != ns_points)          
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size( cTb ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_check_size( bTg ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_check_size( gTp ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_check_size( pTs ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate declared memory 
   error = rox_dynvec_point3d_float_new( &mb_c,  100 );         
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_float_new( &ms_b,  100 );         
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_float_new( &ms_c,  100 );         
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_float_new( &msu_g, 100 );         
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point3d_float_new( &msu_s, 100 );         
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_float_new( &qb_c,  100 );         
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_float_new( &qs_c,  100 );         
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &diff_b, nb_points * 2, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &dist_b, nb_points    , 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &diff_s, ns_points * 2, 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &dist_s, ns_points    , 1 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &LtL  , 8 , 8 );             
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &Ltd  , 8 , 1 );             
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &perm_slice_g ,  3 , 3 );    
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &perm_slice_s ,  3 , 3 );    
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &perm_slice_rot, 4 , 4 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matut3_new( &K );               
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new( &gTs );               
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new( &bTs );               
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_new( &cTs );               
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Fill K mat 
   error = rox_matut3_set_unit ( K );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate b points utility storage 
   mb_c->used = 0;
   qb_c->used = 0;
   error = rox_dynvec_point3d_float_usecells( mb_c, nb_points ); 
   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_dynvec_point2d_float_usecells( qb_c, nb_points ); 
   ROX_ERROR_CHECK_TERMINATE(error)

   // Allocate s points utility storage 
   ms_c->used  = 0;
   ms_b->used  = 0;
   msu_g->used = 0;
   qs_c->used  = 0;
   error = rox_dynvec_point3d_float_usecells( ms_b , ns_points ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point3d_float_usecells( ms_c , ns_points ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point3d_float_usecells( msu_g, ns_points ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point3d_float_usecells( msu_s, ns_points ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_float_usecells( qs_c , ns_points ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Initialize minimization control variables
   sqsum1b  = 0.0;
   sqsum2b  = 0.0;
   sqsum1s  = 0.0;
   sqsum2s  = 0.0;
   invcount = 1.0 / (Rox_Double)( nb_points + ns_points );

   // Compute b points distances
   error = rox_point3d_float_transform( mb_c->data, cTb, mb->data, mb->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point2d_float_project( qb_c->data, mb_c->data, K, mb_c->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_double_distance_point2d_float( dist_b, qb_c, qb );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute s points distances
   // Compute gTs, bTs and cTs
   error = rox_matse3_mulmatmat( gTs, gTp, pTs );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_mulmatmat( bTs, bTg, gTs );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matse3_mulmatmat( cTs, cTb, bTs );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute s points transformation in C frame
   error = rox_point3d_float_transform( ms_c->data, cTs, ms->data, ms->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute s points projection
   error = rox_point2d_float_project( qs_c->data, ms_c->data, K, ms_c->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_point2d_double_distance_point2d_float( dist_s, qs_c, qs );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Reorder b points to keep only the close ones 
   error = rox_array2d_double_get_data_pointer(&ddist, dist_b);
   ROX_ERROR_CHECK_TERMINATE ( error );

   countvalidb = 0;
   for (idpt = 0; idpt < qb->used; idpt++)
      if (ddist[idpt] < maxdist_prefilter)
      {
         mb->data[countvalidb] = mb->data[idpt];
         qb->data[countvalidb] = qb->data[idpt];
         countvalidb++;
      }

   mb->used   = countvalidb;
   qb->used   = countvalidb;
   mb_c->used = countvalidb;
   qb_c->used = countvalidb;

   // Reorder s points to keep only the close ones 
   error = rox_array2d_double_get_data_pointer ( &ddist, dist_s);
   ROX_ERROR_CHECK_TERMINATE ( error );

   countvalids = 0;
   for (idpt = 0; idpt < qs->used; idpt++)
      if (ddist[idpt] < maxdist_prefilter)
      {
         ms->data[countvalids] = ms->data[idpt];
         qs->data[countvalids] = qs->data[idpt];
         countvalids++;
      }

   ms->used    = countvalids;
   qs->used    = countvalids;
   ms_b->used  = countvalids;
   ms_c->used  = countvalids;
   msu_g->used = countvalids;
   msu_s->used = countvalids;
   qs_c->used  = countvalids;

   // Check there are still enough constraints 
   if ( ( countvalidb + countvalids ) <= 5 )
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Resize containers based on the count of b valid points 
   error = rox_array2d_double_new( &weight_b , countvalidb    , 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &work1_b  , countvalidb    , 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &work2_b  , countvalidb    , 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &subdiff_b, countvalidb * 2, 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &subdist_b, countvalidb    , 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Resize containers based on the count of s valid points 
   error = rox_array2d_double_new( &weight_s , countvalids    , 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &work1_s  , countvalids    , 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &work2_s  , countvalids    , 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &subdiff_s, countvalids * 2, 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new( &subdist_s, countvalids    , 1 );    
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Prepare some permutation and rotation matrix data 
   // Set permute-slicing matrix for bTg
   error = rox_array2d_double_fillval( perm_slice_g, 0.0 );            
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( perm_slice_g, 0, 1, +1.0 );   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( perm_slice_g, 1, 0, -1.0 );   
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set permute-slicing matrix for pTs
   error = rox_array2d_double_fillval( perm_slice_s, 0.0 );            
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( perm_slice_s, 0, 1, -1.0 );   
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( perm_slice_s, 1, 0, +1.0 );   
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get access to the rotation part of cTb
   error = rox_array2d_double_new_subarray2d( &cRb, cTb, 0, 0, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get access to the rotation part of cTs
   error = rox_array2d_double_new_subarray2d( &cRs, cTs, 0, 0, 3, 3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Prepare a 4x4 matrix to slice, permute and rotate
   error = rox_array2d_double_fillval( perm_slice_rot, 0.0 );          
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_set_value( perm_slice_rot, 3, 3, 1.0 );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Access the first 3x3 part of the permute_slice_rot matrix
   error = rox_array2d_double_new_subarray2d( &psr, perm_slice_rot, 0, 0, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE(error)

   // Iterative minimization 
   for ( iter = 0; iter < max_iter; iter++ )
   {
      // Compute b points projection 
      error = rox_point3d_float_transform( mb_c->data, cTb, mb->data, mb->used );
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_point2d_float_project( qb_c->data, mb_c->data, K, mb_c->used );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Re-compute gTs, bTs and cTs
      error = rox_array2d_double_mulmatmat( gTs, gTp, pTs );
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_array2d_double_mulmatmat( bTs, bTg, gTs );
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_array2d_double_mulmatmat( cTs, cTb, bTs );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute s points tranformation to b frame 
      error = rox_point3d_float_transform( ms_b->data, bTs, ms->data, ms->used );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute s points projection 
      error = rox_point3d_float_transform( ms_c->data, cTs, ms->data, ms->used );
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_point2d_float_project( qs_c->data, ms_c->data, K, ms_c->used );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute artefact msu_g
      error = rox_array2d_double_mulmatmat( psr, cRb, perm_slice_g );
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_point3d_float_transform( msu_g->data, perm_slice_rot, ms_b->data, ms_b->used );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute artefact msu_s 
      error = rox_array2d_double_mulmatmat( psr, cRs, perm_slice_s );
      ROX_ERROR_CHECK_TERMINATE(error)

      error = rox_point3d_float_transform( msu_s->data, perm_slice_rot, ms->data, ms->used );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Re-Compute b distances 
      error = rox_array2d_point2d_double_distance_point2d_float( subdist_b, qb_c, qb );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Re-Compute s distances 
      error = rox_array2d_point2d_double_distance_point2d_float( subdist_s, qs_c, qs );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Author of these lines thinks computing tukey weights independently
      // for s and b points makes sense since both sets will probably
      // exhibit different error scales. Would it just be for the s points
      // being subjects to transformation unknowns.

      // Compute robust weights for b points 
      error = rox_array2d_double_tukey( weight_b, work1_b, work2_b, subdist_b );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute robust weights for s points 
      error = rox_array2d_double_tukey( weight_s, work1_s, work2_s, subdist_s );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute b points errors 
      error = rox_array2d_point2d_double_difference_point2d_float( subdiff_b, qb_c, qb );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Compute s points errors 
      error = rox_array2d_point2d_double_difference_point2d_float( subdiff_s, qs_c, qs );
      ROX_ERROR_CHECK_TERMINATE(error)

      // B points score estimation 
      error = rox_vvs_score( &sqsum2b, weight_b, subdist_b );
      ROX_ERROR_CHECK_TERMINATE(error)

      // S points score estimation 
      error = rox_vvs_score( &sqsum2s, weight_s, subdist_s );
      ROX_ERROR_CHECK_TERMINATE(error)

      // Check convergence 
      delta   = fabs( (sqsum2b + sqsum2s) - (sqsum1b + sqsum1s) ) * invcount;
      sqsum1b = sqsum2b;
      sqsum1s = sqsum2s;
      if ( (delta < 1e-10) && (iter > 0) )
         break;

      // Compute interaction matrix and prepare its pseudo-inversion 
      error = rox_linsys_point2d_nor_matse3_matso3z_matso3z(LtL, Ltd, subdiff_b, weight_b, mb_c, subdiff_s, weight_s, ms_c, msu_g, msu_s);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Solve problem 
      error = rox_intmat_se3_so3z_so3z_flat_solver(cTb, bTg, pTs, LtL, Ltd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del( &psr );
   rox_array2d_double_del( &cRb );
   rox_array2d_double_del( &LtL );
   rox_array2d_double_del( &Ltd );
   rox_array2d_double_del( &K );
   rox_array2d_double_del( &gTs );
   rox_array2d_double_del( &bTs );
   rox_array2d_double_del( &cTs );
   rox_array2d_double_del( &cRs );
   rox_array2d_double_del( &perm_slice_g );
   rox_array2d_double_del( &perm_slice_s );
   rox_array2d_double_del( &perm_slice_rot );

   rox_array2d_double_del( &diff_b );
   rox_array2d_double_del( &dist_b );
   rox_array2d_double_del( &weight_b );
   rox_array2d_double_del( &work1_b );
   rox_array2d_double_del( &work2_b );
   rox_array2d_double_del( &subdiff_b );
   rox_array2d_double_del( &subdist_b );

   rox_array2d_double_del( &diff_s );
   rox_array2d_double_del( &dist_s );
   rox_array2d_double_del( &weight_s );
   rox_array2d_double_del( &work1_s );
   rox_array2d_double_del( &work2_s );
   rox_array2d_double_del( &subdiff_s );
   rox_array2d_double_del( &subdist_s );

   rox_dynvec_point2d_float_del( &qb_c );
   rox_dynvec_point2d_float_del( &qs_c );

   rox_dynvec_point3d_float_del( &mb_c );
   rox_dynvec_point3d_float_del( &ms_c );
   rox_dynvec_point3d_float_del( &ms_b );
   rox_dynvec_point3d_float_del( &msu_g );
   rox_dynvec_point3d_float_del( &msu_s );

   return error;
}
