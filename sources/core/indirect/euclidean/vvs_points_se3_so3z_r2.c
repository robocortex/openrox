//==============================================================================
//
//    OPENROX   : File vvs_points_se3_so3z_r2.c
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

#include "vvs_points_se3_so3z_r2.h"

#include <generated/objset_matse3_struct.h>

#include <generated/objset_dynvec_point2d_double_struct.h>
#include <generated/objset_dynvec_point3d_double_struct.h>

#include <generated/objset_dynvec_point2d_double.h>
#include <generated/objset_dynvec_point3d_double.h>

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point3d_double.h>

#include <generated/dynvec_double.h>
#include <generated/dynvec_double_struct.h>

#include <baseproc/geometry/point/dynvec_point2d_tools.h>
#include <baseproc/geometry/point/dynvec_point2d_projection_from_dynvec_point3d.h>
#include <baseproc/geometry/point/objset_dynvec_point3d_matse3_transform.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/calculus/jacobians/interaction_point2d_nor_matse3_matso3z_r2.h>

#include <inout/geometry/point/objset_dynvec_point3d_print.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

#include <math.h>

#define MAX_ITERS  10
#define LAMBDA     0.9
#define MEAN_NOR_ERROR_THRESHOLD 0.01

Rox_ErrorCode rox_state_update (
   Rox_MatSE3 cTo,
   Rox_ObjSet_MatSE3 oTb,              // Should be an objset since there are many possible planes
   Rox_Matrix x
);

Rox_ErrorCode rox_objset_dynvec_point2d_error (
   Rox_Double *dq,
   Rox_ObjSet_DynVec_Point2D_Double qr,
   Rox_ObjSet_DynVec_Point2D_Double qc
);

Rox_ErrorCode rox_point2d_error (
   Rox_Matrix dq,
   Rox_DynVec_Point2D_Double qr,
   Rox_DynVec_Point2D_Double qc
);

Rox_ErrorCode rox_build_linearized_system (
   Rox_Matrix A,  // A = L'*L
   Rox_Matrix b,  // b = L'*e
   Rox_ObjSet_DynVec_Point2D_Double qr,
   Rox_ObjSet_DynVec_Point2D_Double qc,
   Rox_MatSE3 cTo,
   Rox_ObjSet_DynVec_Point3D_Double mo
);

Rox_ErrorCode rox_objset_dynvec_point3d_perspective_projection (
   Rox_ObjSet_DynVec_Point2D_Double qc,
   Rox_MatSE3 cTo,
   Rox_ObjSet_DynVec_Point3D_Double mo
);

Rox_ErrorCode rox_vvs_points_pix_se3_so3z_r2 (
   Rox_MatSE3 cTo,
   Rox_ObjSet_MatSE3 oTb,              // Should be an objset since there are many possible planes
   const Rox_Matrix Kc,
   const Rox_ObjSet_DynVec_Point2D_Double pr,  // Should be a dynvec since there are many possible planes
   const Rox_ObjSet_DynVec_Point3D_Double mb   // Should be a dynvec since there are many possible planes
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_ObjSet_DynVec_Point2D_Double qr = NULL;

   if ( !cTo || !oTb )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !Kc || !pr || !mb )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint nb_models = pr->used;

   error = rox_objset_dynvec_point2d_double_new ( &qr, nb_models );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint k=0; k<nb_models; k++)
   {
      Rox_DynVec_Point2D_Double qr_k = NULL;
      Rox_DynVec_Point2D_Double pr_k = pr->data[k];

      Rox_Uint nb_points = pr_k->used;

      error = rox_dynvec_point2d_double_new ( &qr_k, nb_points );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point2d_double_usecells ( qr_k, nb_points );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Transform pixel coordinates to normalized coordinates ( qr = inv(Kc) * pr )
      error = rox_dynvec_point2d_convert_pixel_double_to_meter_double ( qr_k, pr_k, Kc );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Append set of points qr_k to objset qr

      error = rox_objset_dynvec_point2d_double_append ( qr, qr_k ); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Display sets for debug
   // error = rox_objset_dynvec_point2d_double_print(qr);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // VS with normalized coordinates
   error = rox_vvs_points_nor_se3_so3z_r2 ( cTo, oTb, qr, mb );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_objset_dynvec_point2d_double_del ( &qr );

   return error;
}

Rox_ErrorCode rox_vvs_points_nor_se3_so3z_r2 (
   Rox_MatSE3 cTo,
   Rox_ObjSet_MatSE3 oTb,              // Should be an objset since there are many possible planes
   const Rox_ObjSet_DynVec_Point2D_Double qr,  // Should be a dynvec since there are many possible planes
   const Rox_ObjSet_DynVec_Point3D_Double mb   // Should be a dynvec since there are many possible planes
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_ObjSet_DynVec_Point3D_Double mo = NULL;
   Rox_ObjSet_DynVec_Point2D_Double qc = NULL;

   Rox_Matrix A_inv = NULL;
   Rox_Matrix A = NULL;
   Rox_Matrix b = NULL;
   Rox_Matrix x = NULL;

   if ( !cTo || !oTb )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !qr || !mb )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint nb_models = mb->used;

   // Define the models of the points on the planes
   error = rox_objset_dynvec_point3d_double_new ( &mo, nb_models);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_dynvec_point2d_double_new ( &qc, nb_models);
   ROX_ERROR_CHECK_TERMINATE ( error );


   // Allocate dynvecs
   for ( Rox_Sint k=0; k<nb_models; k++)
   {
      Rox_DynVec_Point3D_Double dynvec_mo = NULL;
      Rox_Sint nb_points = mb->data[k]->used;

      error = rox_dynvec_point3d_double_new ( &dynvec_mo, nb_points );

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point3d_double_usecells( dynvec_mo, nb_points );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_objset_dynvec_point3d_double_append ( mo, dynvec_mo );
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_DynVec_Point2D_Double dynvec_qc = NULL;

      error = rox_dynvec_point2d_double_new ( &dynvec_qc, nb_points );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point2d_double_usecells( dynvec_qc, nb_points );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_objset_dynvec_point2d_double_append ( qc, dynvec_qc );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_matrix_new ( &A_inv, 6+3*(nb_models-1), 6+3*(nb_models-1) );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &A, 6+3*(nb_models-1), 6+3*(nb_models-1) );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_set_unit ( A );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &b, 6+3*(nb_models-1), 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &x, 6+3*(nb_models-1), 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint iter=0; iter < MAX_ITERS; iter++)
   {
      // Compute models in Fo frame : mo = oTb * mb

      error = rox_objset_dynvec_point3d_double_transform ( mo, oTb, mb );   
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Project points in Fc frame : qc = perspective_projection ( cTo, mo )

      error = rox_objset_dynvec_point3d_perspective_projection ( qc, cTo, mo );   
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute interaction matrix and error ( S = L'*L; e = L'*(qc-qr) )
      error = rox_build_linearized_system ( A, b, qr, qc, cTo, mo);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Solve linear system

      error = rox_array2d_double_svdinverse(A_inv, A);     
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(x, A_inv, b); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_scale_inplace(x, -LAMBDA); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_print(x);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update state
      error = rox_state_update(cTo, oTb, x);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Compute final error
   Rox_Double dq_norm = 0.0;

   error = rox_objset_dynvec_point2d_error ( &dq_norm, qr, qc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if ( dq_norm > MEAN_NOR_ERROR_THRESHOLD )
   {
      error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:

   rox_objset_dynvec_point3d_double_del ( &mo );
   rox_objset_dynvec_point2d_double_del ( &qc );

   rox_matrix_del ( &A_inv );
   rox_matrix_del ( &A );
   rox_matrix_del ( &b );
   rox_matrix_del ( &x );

   return error;
}

Rox_ErrorCode rox_state_update (
   Rox_MatSE3 cTo,
   Rox_ObjSet_MatSE3 oTb,              // Should be an objset since there are many possible planes
   Rox_Matrix x
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Matrix v = NULL;
   Rox_Matrix xk = NULL;

   error = rox_array2d_double_new ( &v, 6, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** v_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &v_data, v);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &xk, x, 0, 0, 6, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale_inplace(xk, -1.0);
   ROX_ERROR_CHECK_TERMINATE( error );

   error = rox_matse3_update_left(cTo, xk);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_del( &xk);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k=1; k<oTb->used; k++)
   {
      error = rox_array2d_double_new_subarray2d( &xk, x, 6+3*(k-1), 0, 3, 1 );
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Double ** xk_data = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer ( &xk_data, xk );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matrix_set_zero (v);
      ROX_ERROR_CHECK_TERMINATE ( error );

      v_data[0][0] = xk_data[0][0];
      v_data[1][0] = xk_data[1][0];
      v_data[5][0] = xk_data[2][0];


      error = rox_array2d_double_scale_inplace(v, -1.0);
      ROX_ERROR_CHECK_TERMINATE( error );

      error = rox_matse3_update_left(oTb->data[k], v);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_del( &xk);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del ( &v );

   return error;
}


Rox_ErrorCode rox_build_linearized_system (
   Rox_Matrix A,  // A = L'*L
   Rox_Matrix b,  // b = L'*e
   Rox_ObjSet_DynVec_Point2D_Double qr,
   Rox_ObjSet_DynVec_Point2D_Double qc,
   Rox_MatSE3 cTo,
   Rox_ObjSet_DynVec_Point3D_Double mo
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // size(A) = 6+3*(nb_planes-1) x 6+3*(nb_planes-1)
   // size(b) = 6+3*(nb_planes-1)

   error = rox_matrix_set_zero(A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_set_zero(b);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint nb_points = mo->data[0]->used;

   Rox_Matrix Lo = NULL;
   Rox_Matrix Lb = NULL;
   Rox_Matrix dq = NULL;

   error = rox_matrix_new ( &Lo, 2*nb_points, 6);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &Lb, 2*nb_points, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &dq, 2*nb_points, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Matrix A_11 = NULL;
   Rox_Matrix A_1k = NULL;
   Rox_Matrix A_k1 = NULL;
   Rox_Matrix A_kk = NULL;

   Rox_Matrix S_11 = NULL;

   Rox_Matrix b_1 = NULL;
   Rox_Matrix b_k = NULL;

   Rox_Matrix s_1 = NULL;

   error = rox_matrix_new ( &S_11, 6, 6);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new ( &s_1, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d ( &A_11, A, 0, 0, 6, 6 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d ( &b_1, b, 0, 0, 6, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k=0; k < mo->used; k++)
   {
      Rox_Uint initial_row = 0;
      Rox_Uint initial_col = 0;
      Rox_Sint rows = 0;
      Rox_Sint cols = 0;

      error = rox_point2d_error ( dq, qr->data[k], qc->data[k] );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_interaction_matse3_matso3z_r2_point2d_nor ( Lo, Lb, cTo, mo->data[k]); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute A and b
      // A = A + Lo'*Lo
      error = rox_array2d_double_mulmattransmat(S_11, Lo, Lo);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add(A_11, A_11, S_11);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmattransmat(s_1, Lo, dq);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add(b_1, b_1, s_1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if ( k > 0 )
      {
         // Set A_1k
         initial_row = 6*(k-1)   ;  rows = 6;
         initial_col = 6+3*(k-1) ;  cols = 3;

         error = rox_array2d_double_new_subarray2d( &A_1k, A, initial_row, initial_col, rows, cols );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_mulmattransmat(A_1k, Lo, Lb);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matrix_del ( &A_1k );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Set A_k1
         initial_row = 6+3*(k-1);   rows = 3;
         initial_col = 6*(k-1)  ;   cols = 6;

         error = rox_array2d_double_new_subarray2d( &A_k1, A, initial_row, initial_col, rows, cols );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_mulmattransmat(A_k1, Lb, Lo);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matrix_del ( &A_k1 );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Set A_kk
         initial_row = 6+3*(k-1);   rows = 3;
         initial_col = 6+3*(k-1);   cols = 3;

         error = rox_array2d_double_new_subarray2d( &A_kk, A, initial_row, initial_col, rows, cols );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_mulmattransmat(A_kk, Lb, Lb);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matrix_del ( &A_kk );
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Set b_k
         initial_row = 6+3*(k-1); rows = 3;
         initial_col = 0;         cols = 1;

         error = rox_array2d_double_new_subarray2d( &b_k, b, initial_row, initial_col, rows, cols );
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_mulmattransmat(b_k, Lb, dq);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matrix_del ( &b_k );
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

function_terminate:

   rox_matrix_del ( &dq );

   rox_matrix_del ( &Lo );
   rox_matrix_del ( &Lb );

   rox_matrix_del ( &S_11 );
   rox_matrix_del ( &A_11 );

   rox_matrix_del ( &s_1 );

   rox_matrix_del ( &b_1 );
   rox_matrix_del ( &b_k );


   return error;
}

Rox_ErrorCode rox_objset_dynvec_point2d_error (
   Rox_Double * dq,
   Rox_ObjSet_DynVec_Point2D_Double qr,
   Rox_ObjSet_DynVec_Point2D_Double qc
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint count = 0;

   *dq = 0.0;

   for (Rox_Uint k = 0; k < qr->used; k++)
   {
      Rox_DynVec_Point2D_Double qr_k = qr->data[k];
      Rox_DynVec_Point2D_Double qc_k = qc->data[k];

      for (Rox_Uint i = 0; i < qr_k->used; i++)
      {
         Rox_Double dx = qr_k->data[i].u - qc_k->data[i].u;
         Rox_Double dy = qr_k->data[i].v - qc_k->data[i].v;
         *dq += sqrt(dx*dx+dy*dy);
         count = count+1;
      }
   }
   if (count > 0) *dq = *dq/count;

//function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_error (
   Rox_Matrix dq,
   Rox_DynVec_Point2D_Double qr,
   Rox_DynVec_Point2D_Double qc
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double ** dq_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dq_data, dq);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k = 0; k < qr->used; k++)
   {
      dq_data[2*k  ][0] = qr->data[k].u - qc->data[k].u;
      dq_data[2*k+1][0] = qr->data[k].v - qc->data[k].v;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_dynvec_point3d_perspective_projection (
   Rox_ObjSet_DynVec_Point2D_Double qc,
   Rox_MatSE3 cTo,
   Rox_ObjSet_DynVec_Point3D_Double mo
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Matrix calib = NULL;
   error = rox_matrix_new ( &calib , 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_set_unit ( calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint nb_models = mo->used;

   for (Rox_Uint k=0; k<nb_models; k++)
   {
      Rox_Uint nb_points = qc->data[k]->used;
      Rox_DynVec_Double z = NULL;

      error = rox_dynvec_double_new ( &z, nb_points );
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_double_usecells( z, nb_points );
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Projects points
      error = rox_dynvec_point2d_double_transform_project (qc->data[k], z, calib, cTo, mo->data[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_double_del ( &z );
      ROX_ERROR_CHECK_TERMINATE ( error );

   }

function_terminate:

   rox_matrix_del ( &calib );

   return error;

}
