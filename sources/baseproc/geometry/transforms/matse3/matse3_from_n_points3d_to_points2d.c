//==============================================================================
//
//    OPENROX   : File matse3_from_n_points3d_to_points2d.c
//
//    Contents  : Implementation of matse3 from points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "matse3_from_n_points3d_to_points2d.h"

#include <math.h>

#include <generated/array2d_double.h>
#include <generated/dynvec_double.h>
#include <generated/objset_array2d_double_struct.h>

#include <baseproc/maths/linalg/matse3.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/copy/copy.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>

#include <baseproc/array/eigenv/real_eigenvalues_eigenvectors.h>
#include <baseproc/array/eigenv/real_eigenvalues_from_tridiagonal_matrix.h>
#include <baseproc/array/eigenv/real_eigenvectors_from_eigenvalues.h>


#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/array/inverse/svdinverse.h>

#include <baseproc/array/tridiagonal/tridiagonal.h>
#include <baseproc/array/median/median.h>

#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillzero.h>
#include <baseproc/array/fill/fillunit.h>

#include <baseproc/geometry/transforms/transform_tools.h>

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <baseproc/maths/linalg/matso3.h>

#include <inout/numeric/array2d_save.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

// Internal functions
Rox_ErrorCode rox_sort_solutions_ascending_cost(Rox_ObjSet_Array2D_Double Te, Rox_ObjSet_Array2D_Double ze, Rox_DynVec_Double ce);
Rox_ErrorCode rox_compute_normalized_data(Rox_Array2D_Double A, Rox_Array2D_Double P, Rox_Array2D_Double W, const Rox_Point2D_Double q, const Rox_Point3D_Double m, const Rox_Sint n);
Rox_ErrorCode rox_solve_gibbs_parametrization_numeric(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double A);
Rox_ErrorCode rox_compute_coefficients_gibbs_parametrization(Rox_Array2D_Double C, const Rox_Array2D_Double A);
Rox_ErrorCode rox_solve_gibbs_equations_system_macaulay_resultant_numeric(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double C);
Rox_ErrorCode rox_matrix_macaulay_gibbs(Rox_Array2D_Double M, const Rox_Array2D_Double C);
Rox_ErrorCode rox_extract_real_solutions_macaulay(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double M);
Rox_ErrorCode rox_compute_valid_solutions(Rox_ObjSet_Array2D_Double Te, Rox_ObjSet_Array2D_Double ze, Rox_DynVec_Double ce, const Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double P, const Rox_Array2D_Double W, const Rox_Point2D_Double q, const Rox_Point3D_Double m, const Rox_Uint nbp);

// Exported functions
Rox_ErrorCode rox_matse3_from_n_points3d_to_points2d_double(Rox_ObjSet_Array2D_Double T, Rox_ObjSet_Array2D_Double z, const Rox_Point2D_Double q, const Rox_Point3D_Double m, const Rox_Sint n)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nb_used = 0;

   Rox_Array2D_Double A = NULL;
   Rox_Array2D_Double P = NULL;
   Rox_Array2D_Double W = NULL;
   Rox_ObjSet_Array2D_Double R = NULL;
   Rox_DynVec_Double c = NULL;

   if (!T || !m || !q) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&A, 9, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&P, 3, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&W, n, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_compute_normalized_data(A, P, W, q, m, n);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create an object set of size 27 (maximum number of real solutions)
   error = rox_objset_array2d_double_new(&R, 27);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new(&c, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_solve_gibbs_parametrization_numeric(R, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_get_used(&nb_used, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_compute_valid_solutions(T, z, c, R, P, W, q, m, n);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_get_used(&nb_used, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&A);
   rox_array2d_double_del(&P);
   rox_array2d_double_del(&W);
   rox_objset_array2d_double_del(&R);
   rox_dynvec_double_del(&c);

   return error;
}

Rox_ErrorCode rox_compute_normalized_data(Rox_Array2D_Double A, Rox_Array2D_Double P, Rox_Array2D_Double W, const Rox_Point2D_Double q, const Rox_Point3D_Double m, const Rox_Sint n)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double v = NULL;
   Rox_Array2D_Double vk = NULL;
   Rox_Array2D_Double U = NULL;
   Rox_Array2D_Double Ui = NULL;
   Rox_Array2D_Double Qk = NULL;
   Rox_Array2D_Double Mk = NULL;
   Rox_Array2D_Double Pk = NULL;
   Rox_Array2D_Double Fk = NULL;
   Rox_Array2D_Double Sk = NULL;
   Rox_Array2D_Double wk = NULL;
   Rox_Array2D_Double Ak = NULL;
   Rox_Array2D_Double skew = NULL;
   Rox_ObjSet_Array2D_Double skew2 = NULL;

   if (!A || !P || !W || !m || !q) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&v, 3, n);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** vd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &vd, v);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute v
   for (Rox_Sint k = 0; k < n; k++)
   {
      Rox_Double u = q[k].u;
      Rox_Double v = q[k].v;

      vd[0][k] = u / sqrt(u*u+v*v+1);
      vd[1][k] = v / sqrt(u*u+v*v+1);
      vd[2][k] = 1 / sqrt(u*u+v*v+1);
   }

   // Compute U
   error = rox_array2d_double_new(&U, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero(U);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&vk, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&skew, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_new(&skew2, n);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint k=0; k<n; k++)
   {
      Rox_Array2D_Double Vk = NULL;

      rox_array2d_double_set_value(vk, 0, 0, vd[0][k]);
      rox_array2d_double_set_value(vk, 1, 0, vd[1][k]);
      rox_array2d_double_set_value(vk, 2, 0, vd[2][k]);

      // skew = skew(vk)
      error = rox_transformtools_skew_from_vector(skew, vk);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_new(&Vk, 3, 3);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Vk = skew(vk)^2
      error = rox_matrix_power(Vk, skew, 2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_objset_array2d_double_append(skew2, Vk);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // U = U + skew2;
      error = rox_array2d_double_add(U, U, Vk);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   // U = U/n;
   error = rox_array2d_double_scale_inplace(U,  1.0/((double) n));
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute Ui = inv(U)
   error = rox_array2d_double_new(&Ui, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mat3x3_inverse(Ui, U);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute P such that t = P * vR
   error = rox_array2d_double_fillzero(P);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Qk, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Mk, 3, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Mkd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Mkd, Mk);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Pk, 3, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Array2D_Double * V = NULL;
   error = rox_objset_array2d_double_get_data_pointer ( &V, skew2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint k=0; k<n; k++)
   {
      Rox_Double X = m[k].X;
      Rox_Double Y = m[k].Y;
      Rox_Double Z = m[k].Z;

      //M(:,:,k) = [ m(1,k) m(2,k) m(3,k) 0 0 0 0 0 0 ;
      //             0 0 0 m(1,k) m(2,k) m(3,k) 0 0 0 ;
      //             0 0 0 0 0 0 m(1,k) m(2,k) m(3,k) ];
      Mkd[0][0] = X; Mkd[0][1] = Y; Mkd[0][2] = Z; Mkd[0][3] = 0; Mkd[0][4] = 0; Mkd[0][5] = 0; Mkd[0][6] = 0; Mkd[0][7] = 0; Mkd[0][8] = 0;
      Mkd[1][0] = 0; Mkd[1][1] = 0; Mkd[1][2] = 0; Mkd[1][3] = X; Mkd[1][4] = Y; Mkd[1][5] = Z; Mkd[1][6] = 0; Mkd[1][7] = 0; Mkd[1][8] = 0;
      Mkd[2][0] = 0; Mkd[2][1] = 0; Mkd[2][2] = 0; Mkd[2][3] = 0; Mkd[2][4] = 0; Mkd[2][5] = 0; Mkd[2][6] = X; Mkd[2][7] = Y; Mkd[2][8] = Z;

      rox_array2d_double_set_value(vk, 0, 0, vd[0][k]);
      rox_array2d_double_set_value(vk, 1, 0, vd[1][k]);
      rox_array2d_double_set_value(vk, 2, 0, vd[2][k]);

      //Qk(:,:,k) = Vi * skew(v(:,k))^2;
      error = rox_array2d_double_mulmatmat(Qk, Ui, V[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(Pk, Qk, Mk);
      ROX_ERROR_CHECK_TERMINATE ( error );
     
      // P = P - Q(:,:,k) * M(:,:,k); 

      error = rox_array2d_double_substract(P, P, Pk);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   // P=P/n;
   error = rox_array2d_double_scale_inplace(P, 1.0/((double) n));
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute matrix A such that cost function = vR' * A * vR
   error = rox_array2d_double_fillzero(A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Fk, 3, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Sk, 3, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Ak, 9, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&wk, 1, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute matrix W such that g(k) = W(k,:) * vR
   for (Rox_Sint k=0; k<n; k++)
   {
      Rox_Double X = m[k].X;
      Rox_Double Y = m[k].Y;
      Rox_Double Z = m[k].Z;

      Mkd[0][0] = X; Mkd[0][1] = Y; Mkd[0][2] = Z; Mkd[0][3] = 0; Mkd[0][4] = 0; Mkd[0][5] = 0; Mkd[0][6] = 0; Mkd[0][7] = 0; Mkd[0][8] = 0;
      Mkd[1][0] = 0; Mkd[1][1] = 0; Mkd[1][2] = 0; Mkd[1][3] = X; Mkd[1][4] = Y; Mkd[1][5] = Z; Mkd[1][6] = 0; Mkd[1][7] = 0; Mkd[1][8] = 0;
      Mkd[2][0] = 0; Mkd[2][1] = 0; Mkd[2][2] = 0; Mkd[2][3] = 0; Mkd[2][4] = 0; Mkd[2][5] = 0; Mkd[2][6] = X; Mkd[2][7] = Y; Mkd[2][8] = Z;

      // F(:,:,k) = M(:,:,k) + P ;
      error = rox_array2d_double_add(Fk, Mk, P);
      ROX_ERROR_CHECK_TERMINATE ( error );

      rox_array2d_double_set_value(vk, 0, 0, vd[0][k]);
      rox_array2d_double_set_value(vk, 1, 0, vd[1][k]);
      rox_array2d_double_set_value(vk, 2, 0, vd[2][k]);

      // W(k,:) = v(:,k)' * F(:,:,k);
      error = rox_array2d_double_mulmattransmat(wk, vk, Fk);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_set_row(W, k, wk);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // S(:,:,k) = V(:,:,k) * F(:,:,k)
      error = rox_array2d_double_mulmatmat(Sk, V[k], Fk);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Ak = S(:,:,k)' * S(:,:,k);
      error = rox_array2d_double_mulmattransmat(Ak, Sk, Sk);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // A = A + Ak
      error = rox_array2d_double_add(A, A, Ak);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&v);
   rox_array2d_double_del(&vk);
   rox_array2d_double_del(&wk);
   rox_array2d_double_del(&U);
   rox_array2d_double_del(&Ui);
   rox_array2d_double_del(&Qk);
   rox_array2d_double_del(&Mk);
   rox_array2d_double_del(&Pk);
   rox_array2d_double_del(&Fk);
   rox_array2d_double_del(&Sk);
   rox_array2d_double_del(&Ak);
   rox_array2d_double_del(&skew);
   rox_objset_array2d_double_del(&skew2);

   return error;
}

Rox_ErrorCode rox_solve_gibbs_parametrization_numeric(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double A)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double C = NULL;

   if (!A || !Re ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&C, 3, 20);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_compute_coefficients_gibbs_parametrization(C, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_solve_gibbs_equations_system_macaulay_resultant_numeric(Re, C);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&C);

   return error;
}

Rox_ErrorCode rox_compute_coefficients_gibbs_parametrization(Rox_Array2D_Double C, const Rox_Array2D_Double A)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!A || !C ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** Cd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Cd, C );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Ad = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Ad, A );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Cd[0][ 0] = 2*Ad[0][0] - 2*Ad[0][4] - 2*Ad[0][8] - 2*Ad[4][0] + 2*Ad[4][4] + 2*Ad[4][8] - 2*Ad[8][0] + 2*Ad[8][4] + 2*Ad[8][8];
   Cd[0][ 1] = 3*Ad[0][1] + 3*Ad[0][3] + 3*Ad[1][0] - 3*Ad[1][4] - 3*Ad[1][8] + 3*Ad[3][0] - 3*Ad[3][4] - 3*Ad[3][8] - 3*Ad[4][1] - 3*Ad[4][3] - 3*Ad[8][1] - 3*Ad[8][3];
   Cd[0][ 2] = 3*Ad[0][2] + 3*Ad[0][6] + 3*Ad[2][0] - 3*Ad[2][4] - 3*Ad[2][8] - 3*Ad[4][2] - 3*Ad[4][6] + 3*Ad[6][0] - 3*Ad[6][4] - 3*Ad[6][8] - 3*Ad[8][2] - 3*Ad[8][6];
   Cd[0][ 3] = 3*Ad[0][7] - 3*Ad[0][5] + 3*Ad[4][5] - 3*Ad[4][7] - 3*Ad[5][0] + 3*Ad[5][4] + 3*Ad[5][8] + 3*Ad[7][0] - 3*Ad[7][4] - 3*Ad[7][8] + 3*Ad[8][5] - 3*Ad[8][7];
   Cd[0][ 4] = 2*Ad[0][4] - 2*Ad[0][0] + 4*Ad[1][1] + 4*Ad[1][3] + 4*Ad[3][1] + 4*Ad[3][3] + 2*Ad[4][0] - 2*Ad[4][4] + 2*Ad[8][8];
   Cd[0][ 5] = 2*Ad[0][5] + 2*Ad[0][7] + 4*Ad[1][2] + 4*Ad[1][6] + 4*Ad[2][1] + 4*Ad[2][3] + 4*Ad[3][2] + 4*Ad[3][6] - 2*Ad[4][5] - 2*Ad[4][7] + 2*Ad[5][0] - 2*Ad[5][4] - 2*Ad[5][8] + 4*Ad[6][1] + 4*Ad[6][3] + 2*Ad[7][0] - 2*Ad[7][4] - 2*Ad[7][8] - 2*Ad[8][5] - 2*Ad[8][7];
   Cd[0][ 6] = 2*Ad[0][2] - 2*Ad[0][6] - 4*Ad[1][5] + 4*Ad[1][7] + 2*Ad[2][0] - 2*Ad[2][4] - 2*Ad[2][8] - 4*Ad[3][5] + 4*Ad[3][7] - 2*Ad[4][2] + 2*Ad[4][6] - 4*Ad[5][1] - 4*Ad[5][3] - 2*Ad[6][0] + 2*Ad[6][4] + 2*Ad[6][8] + 4*Ad[7][1] + 4*Ad[7][3] - 2*Ad[8][2] + 2*Ad[8][6];
   Cd[0][ 7] = 2*Ad[0][8] - 2*Ad[0][0] + 4*Ad[2][2] + 4*Ad[2][6] + 2*Ad[4][4] + 4*Ad[6][2] + 4*Ad[6][6] + 2*Ad[8][0] - 2*Ad[8][8];
   Cd[0][ 8] = 2*Ad[0][3] - 2*Ad[0][1] - 2*Ad[1][0] + 2*Ad[1][4] + 2*Ad[1][8] - 4*Ad[2][5] + 4*Ad[2][7] + 2*Ad[3][0] - 2*Ad[3][4] - 2*Ad[3][8] + 2*Ad[4][1] - 2*Ad[4][3] - 4*Ad[5][2] - 4*Ad[5][6] - 4*Ad[6][5] + 4*Ad[6][7] + 4*Ad[7][2] + 4*Ad[7][6] + 2*Ad[8][1] - 2*Ad[8][3];
   Cd[0][ 9] = 2*Ad[0][0] - 2*Ad[4][4] - 2*Ad[4][8] + 4*Ad[5][5] - 4*Ad[5][7] - 4*Ad[7][5] + 4*Ad[7][7] - 2*Ad[8][4] - 2*Ad[8][8];
   Cd[0][10] = Ad[1][4] - Ad[0][3] - Ad[1][0] - Ad[0][1] - Ad[1][8] - Ad[3][0] + Ad[3][4] - Ad[3][8] + Ad[4][1] + Ad[4][3] - Ad[8][1] - Ad[8][3];
   Cd[0][11] = 2*Ad[1][5] - Ad[0][6] - Ad[0][2] + 2*Ad[1][7] - Ad[2][0] + Ad[2][4] - Ad[2][8] + 2*Ad[3][5] + 2*Ad[3][7] + Ad[4][2] + Ad[4][6] + 2*Ad[5][1] + 2*Ad[5][3] - Ad[6][0] + Ad[6][4] - Ad[6][8] + 2*Ad[7][1] + 2*Ad[7][3] - Ad[8][2] - Ad[8][6];
   Cd[0][12] = Ad[0][5] - Ad[0][7] + 2*Ad[1][2] - 2*Ad[1][6] + 2*Ad[2][1] + 2*Ad[2][3] + 2*Ad[3][2] - 2*Ad[3][6] - Ad[4][5] + Ad[4][7] + Ad[5][0] - Ad[5][4] + Ad[5][8] - 2*Ad[6][1] - 2*Ad[6][3] - Ad[7][0] + Ad[7][4] - Ad[7][8] + Ad[8][5] - Ad[8][7];
   Cd[0][13] = Ad[1][8] - Ad[0][3] - Ad[1][0] - Ad[1][4] - Ad[0][1] + 2*Ad[2][5] + 2*Ad[2][7] - Ad[3][0] - Ad[3][4] + Ad[3][8] - Ad[4][1] - Ad[4][3] + 2*Ad[5][2] + 2*Ad[5][6] + 2*Ad[6][5] + 2*Ad[6][7] + 2*Ad[7][2] + 2*Ad[7][6] + Ad[8][1] + Ad[8][3];
   Cd[0][14] = 4*Ad[2][2] - 4*Ad[1][1] + 4*Ad[3][3] - 4*Ad[5][5] - 4*Ad[6][6] + 4*Ad[7][7];
   Cd[0][15] = Ad[0][1] + Ad[0][3] + Ad[1][0] + Ad[1][4] + Ad[1][8] - 2*Ad[2][5] + 2*Ad[2][7] + Ad[3][0] + Ad[3][4] + Ad[3][8] + Ad[4][1] + Ad[4][3] - 2*Ad[5][2] + 2*Ad[5][6] + 2*Ad[6][5] - 2*Ad[6][7] + 2*Ad[7][2] - 2*Ad[7][6] + Ad[8][1] + Ad[8][3];
   Cd[0][16] = Ad[2][8] - Ad[0][6] - Ad[2][0] - Ad[2][4] - Ad[0][2] - Ad[4][2] - Ad[4][6] - Ad[6][0] - Ad[6][4] + Ad[6][8] + Ad[8][2] + Ad[8][6];
   Cd[0][17] = Ad[0][5] - Ad[0][7] - 2*Ad[1][2] - 2*Ad[1][6] - 2*Ad[2][1] + 2*Ad[2][3] + 2*Ad[3][2] + 2*Ad[3][6] + Ad[4][5] - Ad[4][7] + Ad[5][0] + Ad[5][4] - Ad[5][8] - 2*Ad[6][1] + 2*Ad[6][3] - Ad[7][0] - Ad[7][4] + Ad[7][8] - Ad[8][5] + Ad[8][7];
   Cd[0][18] = Ad[0][2] + Ad[0][6] + 2*Ad[1][5] - 2*Ad[1][7] + Ad[2][0] + Ad[2][4] + Ad[2][8] - 2*Ad[3][5] + 2*Ad[3][7] + Ad[4][2] + Ad[4][6] + 2*Ad[5][1] - 2*Ad[5][3] + Ad[6][0] + Ad[6][4] + Ad[6][8] - 2*Ad[7][1] + 2*Ad[7][3] + Ad[8][2] + Ad[8][6];
   Cd[0][19] = Ad[0][7] - Ad[0][5] - Ad[4][5] + Ad[4][7] - Ad[5][0] - Ad[5][4] - Ad[5][8] + Ad[7][0] + Ad[7][4] + Ad[7][8] - Ad[8][5] + Ad[8][7];

   Cd[1][ 0] = Ad[0][1] + Ad[0][3] + Ad[1][0] - Ad[1][4] - Ad[1][8] + Ad[3][0] - Ad[3][4] - Ad[3][8] - Ad[4][1] - Ad[4][3] - Ad[8][1] - Ad[8][3];
   Cd[1][ 1] = 2*Ad[0][4] - 2*Ad[0][0] + 4*Ad[1][1] + 4*Ad[1][3] + 4*Ad[3][1] + 4*Ad[3][3] + 2*Ad[4][0] - 2*Ad[4][4] + 2*Ad[8][8];
   Cd[1][ 2] = Ad[0][5] + Ad[0][7] + 2*Ad[1][2] + 2*Ad[1][6] + 2*Ad[2][1] + 2*Ad[2][3] + 2*Ad[3][2] + 2*Ad[3][6] - Ad[4][5] - Ad[4][7] + Ad[5][0] - Ad[5][4] - Ad[5][8] + 2*Ad[6][1] + 2*Ad[6][3] + Ad[7][0] - Ad[7][4] - Ad[7][8] - Ad[8][5] - Ad[8][7];
   Cd[1][ 3] = Ad[0][2] - Ad[0][6] - 2*Ad[1][5] + 2*Ad[1][7] + Ad[2][0] - Ad[2][4] - Ad[2][8] - 2*Ad[3][5] + 2*Ad[3][7] - Ad[4][2] + Ad[4][6] - 2*Ad[5][1] - 2*Ad[5][3] - Ad[6][0] + Ad[6][4] + Ad[6][8] + 2*Ad[7][1] + 2*Ad[7][3] - Ad[8][2] + Ad[8][6];
   Cd[1][ 4] = 3*Ad[1][4] - 3*Ad[0][3] - 3*Ad[1][0] - 3*Ad[0][1] - 3*Ad[1][8] - 3*Ad[3][0] + 3*Ad[3][4] - 3*Ad[3][8] + 3*Ad[4][1] + 3*Ad[4][3] - 3*Ad[8][1] - 3*Ad[8][3];
   Cd[1][ 5] = 4*Ad[1][5] - 2*Ad[0][6] - 2*Ad[0][2] + 4*Ad[1][7] - 2*Ad[2][0] + 2*Ad[2][4] - 2*Ad[2][8] + 4*Ad[3][5] + 4*Ad[3][7] + 2*Ad[4][2] + 2*Ad[4][6] + 4*Ad[5][1] + 4*Ad[5][3] - 2*Ad[6][0] + 2*Ad[6][4] - 2*Ad[6][8] + 4*Ad[7][1] + 4*Ad[7][3] - 2*Ad[8][2] - 2*Ad[8][6];
   Cd[1][ 6] = 2*Ad[0][5] - 2*Ad[0][7] + 4*Ad[1][2] - 4*Ad[1][6] + 4*Ad[2][1] + 4*Ad[2][3] + 4*Ad[3][2] - 4*Ad[3][6] - 2*Ad[4][5] + 2*Ad[4][7] + 2*Ad[5][0] - 2*Ad[5][4] + 2*Ad[5][8] - 4*Ad[6][1] - 4*Ad[6][3] - 2*Ad[7][0] + 2*Ad[7][4] - 2*Ad[7][8] + 2*Ad[8][5] - 2*Ad[8][7];
   Cd[1][ 7] = Ad[1][8] - Ad[0][3] - Ad[1][0] - Ad[1][4] - Ad[0][1] + 2*Ad[2][5] + 2*Ad[2][7] - Ad[3][0] - Ad[3][4] + Ad[3][8] - Ad[4][1] - Ad[4][3] + 2*Ad[5][2] + 2*Ad[5][6] + 2*Ad[6][5] + 2*Ad[6][7] + 2*Ad[7][2] + 2*Ad[7][6] + Ad[8][1] + Ad[8][3];
   Cd[1][ 8] = 4*Ad[2][2] - 4*Ad[1][1] + 4*Ad[3][3] - 4*Ad[5][5] - 4*Ad[6][6] + 4*Ad[7][7];
   Cd[1][ 9] = Ad[0][1] + Ad[0][3] + Ad[1][0] + Ad[1][4] + Ad[1][8] - 2*Ad[2][5] + 2*Ad[2][7] + Ad[3][0] + Ad[3][4] + Ad[3][8] + Ad[4][1] + Ad[4][3] - 2*Ad[5][2] + 2*Ad[5][6] + 2*Ad[6][5] - 2*Ad[6][7] + 2*Ad[7][2] - 2*Ad[7][6] + Ad[8][1] + Ad[8][3];
   Cd[1][10] = 2*Ad[0][0] - 2*Ad[0][4] + 2*Ad[0][8] - 2*Ad[4][0] + 2*Ad[4][4] - 2*Ad[4][8] + 2*Ad[8][0] - 2*Ad[8][4] + 2*Ad[8][8];
   Cd[1][11] = 3*Ad[4][5] - 3*Ad[0][7] - 3*Ad[0][5] + 3*Ad[4][7] - 3*Ad[5][0] + 3*Ad[5][4] - 3*Ad[5][8] - 3*Ad[7][0] + 3*Ad[7][4] - 3*Ad[7][8] - 3*Ad[8][5] - 3*Ad[8][7];
   Cd[1][12] = 3*Ad[0][6] - 3*Ad[0][2] - 3*Ad[2][0] + 3*Ad[2][4] - 3*Ad[2][8] + 3*Ad[4][2] - 3*Ad[4][6] + 3*Ad[6][0] - 3*Ad[6][4] + 3*Ad[6][8] - 3*Ad[8][2] + 3*Ad[8][6];
   Cd[1][13] = 2*Ad[0][0] - 2*Ad[4][4] + 2*Ad[4][8] + 4*Ad[5][5] + 4*Ad[5][7] + 4*Ad[7][5] + 4*Ad[7][7] + 2*Ad[8][4] - 2*Ad[8][8];
   Cd[1][14] = 2*Ad[0][1] - 2*Ad[0][3] + 2*Ad[1][0] - 2*Ad[1][4] + 2*Ad[1][8] + 4*Ad[2][5] + 4*Ad[2][7] - 2*Ad[3][0] + 2*Ad[3][4] - 2*Ad[3][8] - 2*Ad[4][1] + 2*Ad[4][3] + 4*Ad[5][2] - 4*Ad[5][6] - 4*Ad[6][5] - 4*Ad[6][7] + 4*Ad[7][2] - 4*Ad[7][6] + 2*Ad[8][1] - 2*Ad[8][3];
   Cd[1][15] = 4*Ad[2][2] - 2*Ad[0][8] - 2*Ad[0][0] - 4*Ad[2][6] + 2*Ad[4][4] - 4*Ad[6][2] + 4*Ad[6][6] - 2*Ad[8][0] - 2*Ad[8][8];
   Cd[1][16] = Ad[5][8] - Ad[0][7] - Ad[4][5] - Ad[4][7] - Ad[5][0] - Ad[5][4] - Ad[0][5] - Ad[7][0] - Ad[7][4] + Ad[7][8] + Ad[8][5] + Ad[8][7];
   Cd[1][17] = Ad[0][6] - Ad[0][2] - 2*Ad[1][5] - 2*Ad[1][7] - Ad[2][0] - Ad[2][4] + Ad[2][8] + 2*Ad[3][5] + 2*Ad[3][7] - Ad[4][2] + Ad[4][6] - 2*Ad[5][1] + 2*Ad[5][3] + Ad[6][0] + Ad[6][4] - Ad[6][8] - 2*Ad[7][1] + 2*Ad[7][3] + Ad[8][2] - Ad[8][6];
   Cd[1][18] = Ad[0][5] + Ad[0][7] - 2*Ad[1][2] + 2*Ad[1][6] - 2*Ad[2][1] + 2*Ad[2][3] + 2*Ad[3][2] - 2*Ad[3][6] + Ad[4][5] + Ad[4][7] + Ad[5][0] + Ad[5][4] + Ad[5][8] + 2*Ad[6][1] - 2*Ad[6][3] + Ad[7][0] + Ad[7][4] + Ad[7][8] + Ad[8][5] + Ad[8][7];
   Cd[1][19] = Ad[0][2] - Ad[0][6] + Ad[2][0] + Ad[2][4] + Ad[2][8] + Ad[4][2] - Ad[4][6] - Ad[6][0] - Ad[6][4] - Ad[6][8] + Ad[8][2] - Ad[8][6];

   Cd[2][ 0] = Ad[0][2] + Ad[0][6] + Ad[2][0] - Ad[2][4] - Ad[2][8] - Ad[4][2] - Ad[4][6] + Ad[6][0] - Ad[6][4] - Ad[6][8] - Ad[8][2] - Ad[8][6];
   Cd[2][ 1] = Ad[0][5] + Ad[0][7] + 2*Ad[1][2] + 2*Ad[1][6] + 2*Ad[2][1] + 2*Ad[2][3] + 2*Ad[3][2] + 2*Ad[3][6] - Ad[4][5] - Ad[4][7] + Ad[5][0] - Ad[5][4] - Ad[5][8] + 2*Ad[6][1] + 2*Ad[6][3] + Ad[7][0] - Ad[7][4] - Ad[7][8] - Ad[8][5] - Ad[8][7];
   Cd[2][ 2] = 2*Ad[0][8] - 2*Ad[0][0] + 4*Ad[2][2] + 4*Ad[2][6] + 2*Ad[4][4] + 4*Ad[6][2] + 4*Ad[6][6] + 2*Ad[8][0] - 2*Ad[8][8];
   Cd[2][ 3] = Ad[0][3] - Ad[0][1] - Ad[1][0] + Ad[1][4] + Ad[1][8] - 2*Ad[2][5] + 2*Ad[2][7] + Ad[3][0] - Ad[3][4] - Ad[3][8] + Ad[4][1] - Ad[4][3] - 2*Ad[5][2] - 2*Ad[5][6] - 2*Ad[6][5] + 2*Ad[6][7] + 2*Ad[7][2] + 2*Ad[7][6] + Ad[8][1] - Ad[8][3];
   Cd[2][ 4] = 2*Ad[1][5] - Ad[0][6] - Ad[0][2] + 2*Ad[1][7] - Ad[2][0] + Ad[2][4] - Ad[2][8] + 2*Ad[3][5] + 2*Ad[3][7] + Ad[4][2] + Ad[4][6] + 2*Ad[5][1] + 2*Ad[5][3] - Ad[6][0] + Ad[6][4] - Ad[6][8] + 2*Ad[7][1] + 2*Ad[7][3] - Ad[8][2] - Ad[8][6];
   Cd[2][ 5] = 2*Ad[1][8] - 2*Ad[0][3] - 2*Ad[1][0] - 2*Ad[1][4] - 2*Ad[0][1] + 4*Ad[2][5] + 4*Ad[2][7] - 2*Ad[3][0] - 2*Ad[3][4] + 2*Ad[3][8] - 2*Ad[4][1] - 2*Ad[4][3] + 4*Ad[5][2] + 4*Ad[5][6] + 4*Ad[6][5] + 4*Ad[6][7] + 4*Ad[7][2] + 4*Ad[7][6] + 2*Ad[8][1] + 2*Ad[8][3];
   Cd[2][ 6] = 4*Ad[2][2] - 4*Ad[1][1] + 4*Ad[3][3] - 4*Ad[5][5] - 4*Ad[6][6] + 4*Ad[7][7];
   Cd[2][ 7] = 3*Ad[2][8] - 3*Ad[0][6] - 3*Ad[2][0] - 3*Ad[2][4] - 3*Ad[0][2] - 3*Ad[4][2] - 3*Ad[4][6] - 3*Ad[6][0] - 3*Ad[6][4] + 3*Ad[6][8] + 3*Ad[8][2] + 3*Ad[8][6];
   Cd[2][ 8] = 2*Ad[0][5] - 2*Ad[0][7] - 4*Ad[1][2] - 4*Ad[1][6] - 4*Ad[2][1] + 4*Ad[2][3] + 4*Ad[3][2] + 4*Ad[3][6] + 2*Ad[4][5] - 2*Ad[4][7] + 2*Ad[5][0] + 2*Ad[5][4] - 2*Ad[5][8] - 4*Ad[6][1] + 4*Ad[6][3] - 2*Ad[7][0] - 2*Ad[7][4] + 2*Ad[7][8] - 2*Ad[8][5] + 2*Ad[8][7];
   Cd[2][ 9] = Ad[0][2] + Ad[0][6] + 2*Ad[1][5] - 2*Ad[1][7] + Ad[2][0] + Ad[2][4] + Ad[2][8] - 2*Ad[3][5] + 2*Ad[3][7] + Ad[4][2] + Ad[4][6] + 2*Ad[5][1] - 2*Ad[5][3] + Ad[6][0] + Ad[6][4] + Ad[6][8] - 2*Ad[7][1] + 2*Ad[7][3] + Ad[8][2] + Ad[8][6];
   Cd[2][10] = Ad[4][5] - Ad[0][7] - Ad[0][5] + Ad[4][7] - Ad[5][0] + Ad[5][4] - Ad[5][8] - Ad[7][0] + Ad[7][4] - Ad[7][8] - Ad[8][5] - Ad[8][7];
   Cd[2][11] = 2*Ad[0][0] - 2*Ad[4][4] + 2*Ad[4][8] + 4*Ad[5][5] + 4*Ad[5][7] + 4*Ad[7][5] + 4*Ad[7][7] + 2*Ad[8][4] - 2*Ad[8][8];
   Cd[2][12] = Ad[0][1] - Ad[0][3] + Ad[1][0] - Ad[1][4] + Ad[1][8] + 2*Ad[2][5] + 2*Ad[2][7] - Ad[3][0] + Ad[3][4] - Ad[3][8] - Ad[4][1] + Ad[4][3] + 2*Ad[5][2] - 2*Ad[5][6] - 2*Ad[6][5] - 2*Ad[6][7] + 2*Ad[7][2] - 2*Ad[7][6] + Ad[8][1] - Ad[8][3];
   Cd[2][13] = 3*Ad[5][8] - 3*Ad[0][7] - 3*Ad[4][5] - 3*Ad[4][7] - 3*Ad[5][0] - 3*Ad[5][4] - 3*Ad[0][5] - 3*Ad[7][0] - 3*Ad[7][4] + 3*Ad[7][8] + 3*Ad[8][5] + 3*Ad[8][7];
   Cd[2][14] = 2*Ad[0][6] - 2*Ad[0][2] - 4*Ad[1][5] - 4*Ad[1][7] - 2*Ad[2][0] - 2*Ad[2][4] + 2*Ad[2][8] + 4*Ad[3][5] + 4*Ad[3][7] - 2*Ad[4][2] + 2*Ad[4][6] - 4*Ad[5][1] + 4*Ad[5][3] + 2*Ad[6][0] + 2*Ad[6][4] - 2*Ad[6][8] - 4*Ad[7][1] + 4*Ad[7][3] + 2*Ad[8][2] - 2*Ad[8][6];
   Cd[2][15] = Ad[0][5] + Ad[0][7] - 2*Ad[1][2] + 2*Ad[1][6] - 2*Ad[2][1] + 2*Ad[2][3] + 2*Ad[3][2] - 2*Ad[3][6] + Ad[4][5] + Ad[4][7] + Ad[5][0] + Ad[5][4] + Ad[5][8] + 2*Ad[6][1] - 2*Ad[6][3] + Ad[7][0] + Ad[7][4] + Ad[7][8] + Ad[8][5] + Ad[8][7];
   Cd[2][16] = 2*Ad[0][0] + 2*Ad[0][4] - 2*Ad[0][8] + 2*Ad[4][0] + 2*Ad[4][4] - 2*Ad[4][8] - 2*Ad[8][0] - 2*Ad[8][4] + 2*Ad[8][8];
   Cd[2][17] = 3*Ad[0][1] - 3*Ad[0][3] + 3*Ad[1][0] + 3*Ad[1][4] - 3*Ad[1][8] - 3*Ad[3][0] - 3*Ad[3][4] + 3*Ad[3][8] + 3*Ad[4][1] - 3*Ad[4][3] - 3*Ad[8][1] + 3*Ad[8][3];
   Cd[2][18] = 4*Ad[1][1] - 2*Ad[0][4] - 2*Ad[0][0] - 4*Ad[1][3] - 4*Ad[3][1] + 4*Ad[3][3] - 2*Ad[4][0] - 2*Ad[4][4] + 2*Ad[8][8];
   Cd[2][19] = Ad[0][3] - Ad[0][1] - Ad[1][0] - Ad[1][4] - Ad[1][8] + Ad[3][0] + Ad[3][4] + Ad[3][8] - Ad[4][1] + Ad[4][3] - Ad[8][1] + Ad[8][3];

function_terminate:

   return error;
}

Rox_ErrorCode rox_solve_gibbs_equations_system_macaulay_resultant_numeric(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double C)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double M = NULL;

   if (!Re || !C ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&M, 120, 120);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_macaulay_gibbs(M, C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_extract_real_solutions_macaulay(Re, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&M);

   return error;
}

Rox_ErrorCode rox_matrix_macaulay_gibbs(Rox_Array2D_Double M, const Rox_Array2D_Double C)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!M || !C ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** Cd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Cd, C );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Md = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Md, M );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero(M);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Md[0][0] = 1.0;
   Md[0][1] = 1.0;
   Md[0][2] = 1.0;
   Md[0][3] = 1.0;
   Md[1][1] = 1.0;
   Md[1][4] = 1.0;
   Md[1][5] = 1.0;
   Md[1][6] = 1.0;
   Md[2][2] = 1.0;
   Md[2][5] = 1.0;
   Md[2][7] = 1.0;
   Md[2][8] = 1.0;
   Md[3][3] = 1.0;
   Md[3][6] = 1.0;
   Md[3][8] = 1.0;
   Md[3][9] = 1.0;
   Md[4][4] = 1.0;
   Md[4][10] = 1.0;
   Md[4][11] = 1.0;
   Md[4][27] = 1.0;
   Md[5][5] = 1.0;
   Md[5][10] = 1.0;
   Md[5][12] = 1.0;
   Md[5][13] = 1.0;
   Md[6][6] = 1.0;
   Md[6][11] = 1.0;
   Md[6][13] = 1.0;
   Md[6][14] = 1.0;
   Md[7][7] = 1.0;
   Md[7][12] = 1.0;
   Md[7][15] = 1.0;
   Md[7][54] = 1.0;
   Md[8][8] = 1.0;
   Md[8][13] = 1.0;
   Md[8][15] = 1.0;
   Md[8][16] = 1.0;
   Md[9][9] = 1.0;
   Md[9][14] = 1.0;
   Md[9][16] = 1.0;
   Md[9][85] = 1.0;
   Md[10][10] = 1.0;
   Md[10][17] = 1.0;
   Md[10][18] = 1.0;
   Md[10][29] = 1.0;
   Md[11][11] = 1.0;
   Md[11][18] = 1.0;
   Md[11][19] = 1.0;
   Md[11][30] = 1.0;
   Md[12][12] = 1.0;
   Md[12][17] = 1.0;
   Md[12][20] = 1.0;
   Md[12][55] = 1.0;
   Md[13][13] = 1.0;
   Md[13][18] = 1.0;
   Md[13][20] = 1.0;
   Md[13][21] = 1.0;
   Md[14][14] = 1.0;
   Md[14][19] = 1.0;
   Md[14][21] = 1.0;
   Md[14][86] = 1.0;
   Md[15][15] = 1.0;
   Md[15][20] = 1.0;
   Md[15][22] = 1.0;
   Md[15][57] = 1.0;
   Md[16][16] = 1.0;
   Md[16][21] = 1.0;
   Md[16][22] = 1.0;
   Md[16][87] = 1.0;
   Md[17][17] = 1.0;
   Md[17][23] = 1.0;
   Md[17][34] = 1.0;
   Md[17][58] = 1.0;
   Md[18][18] = 1.0;
   Md[18][23] = 1.0;
   Md[18][24] = 1.0;
   Md[18][35] = 1.0;
   Md[19][19] = 1.0;
   Md[19][24] = 1.0;
   Md[19][36] = 1.0;
   Md[19][89] = 1.0;
   Md[20][20] = 1.0;
   Md[20][23] = 1.0;
   Md[20][25] = 1.0;
   Md[20][60] = 1.0;
   Md[21][21] = 1.0;
   Md[21][24] = 1.0;
   Md[21][25] = 1.0;
   Md[21][90] = 1.0;
   Md[22][22] = 1.0;
   Md[22][25] = 1.0;
   Md[22][63] = 1.0;
   Md[22][92] = 1.0;
   Md[23][23] = 1.0;
   Md[23][26] = 1.0;
   Md[23][43] = 1.0;
   Md[23][66] = 1.0;
   Md[24][24] = 1.0;
   Md[24][26] = 1.0;
   Md[24][44] = 1.0;
   Md[24][96] = 1.0;
   Md[25][25] = 1.0;
   Md[25][26] = 1.0;
   Md[25][69] = 1.0;
   Md[25][98] = 1.0;
   Md[26][26] = 1.0;
   Md[26][53] = 1.0;
   Md[26][78] = 1.0;
   Md[26][108] = 1.0;
   Md[27][0] = Cd[0][19];
   Md[27][1] = Cd[0][9];
   Md[27][2] = Cd[0][15];
   Md[27][3] = Cd[0][18];
   Md[27][4] = Cd[0][3];
   Md[27][5] = Cd[0][6];
   Md[27][6] = Cd[0][8];
   Md[27][7] = Cd[0][12];
   Md[27][8] = Cd[0][14];
   Md[27][9] = Cd[0][17];
   Md[27][10] = Cd[0][1];
   Md[27][11] = Cd[0][2];
   Md[27][12] = Cd[0][4];
   Md[27][13] = Cd[0][5];
   Md[27][14] = Cd[0][7];
   Md[27][15] = Cd[0][11];
   Md[27][16] = Cd[0][13];
   Md[27][27] = Cd[0][0];
   Md[27][54] = Cd[0][10];
   Md[27][85] = Cd[0][16];
   Md[28][1] = Cd[0][19];
   Md[28][4] = Cd[0][9];
   Md[28][5] = Cd[0][15];
   Md[28][6] = Cd[0][18];
   Md[28][10] = Cd[0][6];
   Md[28][11] = Cd[0][8];
   Md[28][12] = Cd[0][12];
   Md[28][13] = Cd[0][14];
   Md[28][14] = Cd[0][17];
   Md[28][17] = Cd[0][4];
   Md[28][18] = Cd[0][5];
   Md[28][19] = Cd[0][7];
   Md[28][20] = Cd[0][11];
   Md[28][21] = Cd[0][13];
   Md[28][27] = Cd[0][3];
   Md[28][28] = Cd[0][0];
   Md[28][29] = Cd[0][1];
   Md[28][30] = Cd[0][2];
   Md[28][55] = Cd[0][10];
   Md[28][86] = Cd[0][16];
   Md[29][2] = Cd[0][19];
   Md[29][5] = Cd[0][9];
   Md[29][7] = Cd[0][15];
   Md[29][8] = Cd[0][18];
   Md[29][10] = Cd[0][3];
   Md[29][12] = Cd[0][6];
   Md[29][13] = Cd[0][8];
   Md[29][15] = Cd[0][14];
   Md[29][16] = Cd[0][17];
   Md[29][17] = Cd[0][1];
   Md[29][18] = Cd[0][2];
   Md[29][20] = Cd[0][5];
   Md[29][21] = Cd[0][7];
   Md[29][22] = Cd[0][13];
   Md[29][29] = Cd[0][0];
   Md[29][54] = Cd[0][12];
   Md[29][55] = Cd[0][4];
   Md[29][56] = Cd[0][10];
   Md[29][57] = Cd[0][11];
   Md[29][87] = Cd[0][16];
   Md[30][3] = Cd[0][19];
   Md[30][6] = Cd[0][9];
   Md[30][8] = Cd[0][15];
   Md[30][9] = Cd[0][18];
   Md[30][11] = Cd[0][3];
   Md[30][13] = Cd[0][6];
   Md[30][14] = Cd[0][8];
   Md[30][15] = Cd[0][12];
   Md[30][16] = Cd[0][14];
   Md[30][18] = Cd[0][1];
   Md[30][19] = Cd[0][2];
   Md[30][20] = Cd[0][4];
   Md[30][21] = Cd[0][5];
   Md[30][22] = Cd[0][11];
   Md[30][30] = Cd[0][0];
   Md[30][57] = Cd[0][10];
   Md[30][85] = Cd[0][17];
   Md[30][86] = Cd[0][7];
   Md[30][87] = Cd[0][13];
   Md[30][88] = Cd[0][16];
   Md[31][4] = Cd[0][19];
   Md[31][10] = Cd[0][15];
   Md[31][11] = Cd[0][18];
   Md[31][17] = Cd[0][12];
   Md[31][18] = Cd[0][14];
   Md[31][19] = Cd[0][17];
   Md[31][23] = Cd[0][11];
   Md[31][24] = Cd[0][13];
   Md[31][27] = Cd[0][9];
   Md[31][28] = Cd[0][3];
   Md[31][29] = Cd[0][6];
   Md[31][30] = Cd[0][8];
   Md[31][31] = Cd[0][0];
   Md[31][32] = Cd[0][1];
   Md[31][33] = Cd[0][2];
   Md[31][34] = Cd[0][4];
   Md[31][35] = Cd[0][5];
   Md[31][36] = Cd[0][7];
   Md[31][58] = Cd[0][10];
   Md[31][89] = Cd[0][16];
   Md[32][5] = Cd[0][19];
   Md[32][10] = Cd[0][9];
   Md[32][12] = Cd[0][15];
   Md[32][13] = Cd[0][18];
   Md[32][17] = Cd[0][6];
   Md[32][18] = Cd[0][8];
   Md[32][20] = Cd[0][14];
   Md[32][21] = Cd[0][17];
   Md[32][23] = Cd[0][5];
   Md[32][24] = Cd[0][7];
   Md[32][25] = Cd[0][13];
   Md[32][29] = Cd[0][3];
   Md[32][32] = Cd[0][0];
   Md[32][34] = Cd[0][1];
   Md[32][35] = Cd[0][2];
   Md[32][55] = Cd[0][12];
   Md[32][58] = Cd[0][4];
   Md[32][59] = Cd[0][10];
   Md[32][60] = Cd[0][11];
   Md[32][90] = Cd[0][16];
   Md[33][6] = Cd[0][19];
   Md[33][11] = Cd[0][9];
   Md[33][13] = Cd[0][15];
   Md[33][14] = Cd[0][18];
   Md[33][18] = Cd[0][6];
   Md[33][19] = Cd[0][8];
   Md[33][20] = Cd[0][12];
   Md[33][21] = Cd[0][14];
  Md[33][23] = Cd[0][4];
  Md[33][24] = Cd[0][5];
  Md[33][25] = Cd[0][11];
  Md[33][30] = Cd[0][3];
  Md[33][33] = Cd[0][0];
  Md[33][35] = Cd[0][1];
  Md[33][36] = Cd[0][2];
  Md[33][60] = Cd[0][10];
  Md[33][86] = Cd[0][17];
  Md[33][89] = Cd[0][7];
  Md[33][90] = Cd[0][13];
  Md[33][91] = Cd[0][16];
  Md[34][7] = Cd[0][19];
  Md[34][12] = Cd[0][9];
  Md[34][15] = Cd[0][18];
  Md[34][17] = Cd[0][3];
  Md[34][20] = Cd[0][8];
  Md[34][22] = Cd[0][17];
  Md[34][23] = Cd[0][2];
  Md[34][25] = Cd[0][7];
  Md[34][34] = Cd[0][0];
  Md[34][54] = Cd[0][15];
  Md[34][55] = Cd[0][6];
  Md[34][56] = Cd[0][12];
  Md[34][57] = Cd[0][14];
  Md[34][58] = Cd[0][1];
  Md[34][59] = Cd[0][4];
  Md[34][60] = Cd[0][5];
  Md[34][61] = Cd[0][10];
  Md[34][62] = Cd[0][11];
  Md[34][63] = Cd[0][13];
  Md[34][92] = Cd[0][16];
  Md[35][8] = Cd[0][19];
  Md[35][13] = Cd[0][9];
  Md[35][15] = Cd[0][15];
  Md[35][16] = Cd[0][18];
  Md[35][18] = Cd[0][3];
  Md[35][20] = Cd[0][6];
  Md[35][21] = Cd[0][8];
  Md[35][22] = Cd[0][14];
  Md[35][23] = Cd[0][1];
  Md[35][24] = Cd[0][2];
  Md[35][25] = Cd[0][5];
  Md[35][35] = Cd[0][0];
  Md[35][57] = Cd[0][12];
  Md[35][60] = Cd[0][4];
  Md[35][62] = Cd[0][10];
  Md[35][63] = Cd[0][11];
  Md[35][87] = Cd[0][17];
  Md[35][90] = Cd[0][7];
  Md[35][92] = Cd[0][13];
  Md[35][93] = Cd[0][16];
  Md[36][9] = Cd[0][19];
  Md[36][14] = Cd[0][9];
  Md[36][16] = Cd[0][15];
  Md[36][19] = Cd[0][3];
  Md[36][21] = Cd[0][6];
  Md[36][22] = Cd[0][12];
  Md[36][24] = Cd[0][1];
  Md[36][25] = Cd[0][4];
  Md[36][36] = Cd[0][0];
  Md[36][63] = Cd[0][10];
  Md[36][85] = Cd[0][18];
  Md[36][86] = Cd[0][8];
  Md[36][87] = Cd[0][14];
  Md[36][88] = Cd[0][17];
  Md[36][89] = Cd[0][2];
  Md[36][90] = Cd[0][5];
  Md[36][91] = Cd[0][7];
  Md[36][92] = Cd[0][11];
  Md[36][93] = Cd[0][13];
  Md[36][94] = Cd[0][16];
  Md[37][27] = Cd[0][19];
  Md[37][28] = Cd[0][9];
  Md[37][29] = Cd[0][15];
  Md[37][30] = Cd[0][18];
  Md[37][31] = Cd[0][3];
  Md[37][32] = Cd[0][6];
  Md[37][33] = Cd[0][8];
  Md[37][34] = Cd[0][12];
  Md[37][35] = Cd[0][14];
  Md[37][36] = Cd[0][17];
  Md[37][37] = Cd[0][0];
  Md[37][38] = Cd[0][1];
  Md[37][39] = Cd[0][2];
  Md[37][40] = Cd[0][4];
  Md[37][41] = Cd[0][5];
  Md[37][42] = Cd[0][7];
  Md[37][43] = Cd[0][11];
  Md[37][44] = Cd[0][13];
  Md[37][64] = Cd[0][10];
  Md[37][95] = Cd[0][16];
  Md[38][10] = Cd[0][19];
  Md[38][17] = Cd[0][15];
  Md[38][18] = Cd[0][18];
  Md[38][23] = Cd[0][14];
  Md[38][24] = Cd[0][17];
  Md[38][26] = Cd[0][13];
  Md[38][29] = Cd[0][9];
  Md[38][32] = Cd[0][3];
  Md[38][34] = Cd[0][6];
  Md[38][35] = Cd[0][8];
  Md[38][38] = Cd[0][0];
  Md[38][40] = Cd[0][1];
  Md[38][41] = Cd[0][2];
  Md[38][43] = Cd[0][5];
  Md[38][44] = Cd[0][7];
  Md[38][58] = Cd[0][12];
  Md[38][64] = Cd[0][4];
  Md[38][65] = Cd[0][10];
  Md[38][66] = Cd[0][11];
  Md[38][96] = Cd[0][16];
  Md[39][11] = Cd[0][19];
  Md[39][18] = Cd[0][15];
  Md[39][19] = Cd[0][18];
  Md[39][23] = Cd[0][12];
  Md[39][24] = Cd[0][14];
  Md[39][26] = Cd[0][11];
  Md[39][30] = Cd[0][9];
  Md[39][33] = Cd[0][3];
  Md[39][35] = Cd[0][6];
  Md[39][36] = Cd[0][8];
  Md[39][39] = Cd[0][0];
  Md[39][41] = Cd[0][1];
  Md[39][42] = Cd[0][2];
  Md[39][43] = Cd[0][4];
  Md[39][44] = Cd[0][5];
  Md[39][66] = Cd[0][10];
  Md[39][89] = Cd[0][17];
  Md[39][95] = Cd[0][7];
  Md[39][96] = Cd[0][13];
  Md[39][97] = Cd[0][16];
  Md[40][12] = Cd[0][19];
  Md[40][17] = Cd[0][9];
  Md[40][20] = Cd[0][18];
  Md[40][23] = Cd[0][8];
  Md[40][25] = Cd[0][17];
  Md[40][26] = Cd[0][7];
  Md[40][34] = Cd[0][3];
  Md[40][40] = Cd[0][0];
  Md[40][43] = Cd[0][2];
  Md[40][55] = Cd[0][15];
  Md[40][58] = Cd[0][6];
  Md[40][59] = Cd[0][12];
  Md[40][60] = Cd[0][14];
  Md[40][64] = Cd[0][1];
  Md[40][65] = Cd[0][4];
  Md[40][66] = Cd[0][5];
  Md[40][67] = Cd[0][10];
  Md[40][68] = Cd[0][11];
  Md[40][69] = Cd[0][13];
  Md[40][98] = Cd[0][16];
  Md[41][13] = Cd[0][19];
  Md[41][18] = Cd[0][9];
  Md[41][20] = Cd[0][15];
  Md[41][21] = Cd[0][18];
  Md[41][23] = Cd[0][6];
  Md[41][24] = Cd[0][8];
  Md[41][25] = Cd[0][14];
  Md[41][26] = Cd[0][5];
  Md[41][35] = Cd[0][3];
  Md[41][41] = Cd[0][0];
  Md[41][43] = Cd[0][1];
  Md[41][44] = Cd[0][2];
  Md[41][60] = Cd[0][12];
  Md[41][66] = Cd[0][4];
  Md[41][68] = Cd[0][10];
  Md[41][69] = Cd[0][11];
  Md[41][90] = Cd[0][17];
  Md[41][96] = Cd[0][7];
  Md[41][98] = Cd[0][13];
  Md[41][99] = Cd[0][16];
  Md[42][14] = Cd[0][19];
  Md[42][19] = Cd[0][9];
  Md[42][21] = Cd[0][15];
  Md[42][24] = Cd[0][6];
  Md[42][25] = Cd[0][12];
  Md[42][26] = Cd[0][4];
  Md[42][36] = Cd[0][3];
  Md[42][42] = Cd[0][0];
  Md[42][44] = Cd[0][1];
  Md[42][69] = Cd[0][10];
  Md[42][86] = Cd[0][18];
  Md[42][89] = Cd[0][8];
  Md[42][90] = Cd[0][14];
  Md[42][91] = Cd[0][17];
  Md[42][95] = Cd[0][2];
  Md[42][96] = Cd[0][5];
  Md[42][97] = Cd[0][7];
  Md[42][98] = Cd[0][11];
  Md[42][99] = Cd[0][13];
  Md[42][100] = Cd[0][16];
  Md[43][15] = Cd[0][19];
  Md[43][20] = Cd[0][9];
  Md[43][22] = Cd[0][18];
  Md[43][23] = Cd[0][3];
  Md[43][25] = Cd[0][8];
  Md[43][26] = Cd[0][2];
  Md[43][43] = Cd[0][0];
  Md[43][57] = Cd[0][15];
  Md[43][60] = Cd[0][6];
  Md[43][62] = Cd[0][12];
  Md[43][63] = Cd[0][14];
  Md[43][66] = Cd[0][1];
  Md[43][68] = Cd[0][4];
  Md[43][69] = Cd[0][5];
  Md[43][71] = Cd[0][10];
  Md[43][72] = Cd[0][11];
  Md[43][92] = Cd[0][17];
  Md[43][98] = Cd[0][7];
  Md[43][101] = Cd[0][13];
  Md[43][102] = Cd[0][16];
  Md[44][16] = Cd[0][19];
  Md[44][21] = Cd[0][9];
  Md[44][22] = Cd[0][15];
  Md[44][24] = Cd[0][3];
  Md[44][25] = Cd[0][6];
  Md[44][26] = Cd[0][1];
  Md[44][44] = Cd[0][0];
  Md[44][63] = Cd[0][12];
  Md[44][69] = Cd[0][4];
  Md[44][72] = Cd[0][10];
  Md[44][87] = Cd[0][18];
  Md[44][90] = Cd[0][8];
  Md[44][92] = Cd[0][14];
  Md[44][93] = Cd[0][17];
  Md[44][96] = Cd[0][2];
  Md[44][98] = Cd[0][5];
  Md[44][99] = Cd[0][7];
  Md[44][101] = Cd[0][11];
  Md[44][102] = Cd[0][13];
  Md[44][103] = Cd[0][16];
  Md[45][28] = Cd[0][19];
  Md[45][31] = Cd[0][9];
  Md[45][32] = Cd[0][15];
  Md[45][33] = Cd[0][18];
  Md[45][37] = Cd[0][3];
  Md[45][38] = Cd[0][6];
  Md[45][39] = Cd[0][8];
  Md[45][40] = Cd[0][12];
  Md[45][41] = Cd[0][14];
  Md[45][42] = Cd[0][17];
  Md[45][45] = Cd[0][0];
  Md[45][46] = Cd[0][1];
  Md[45][47] = Cd[0][2];
  Md[45][48] = Cd[0][4];
  Md[45][49] = Cd[0][5];
  Md[45][50] = Cd[0][7];
  Md[45][51] = Cd[0][11];
  Md[45][52] = Cd[0][13];
  Md[45][73] = Cd[0][10];
  Md[45][105] = Cd[0][16];
  Md[46][29] = Cd[0][19];
  Md[46][32] = Cd[0][9];
  Md[46][34] = Cd[0][15];
  Md[46][35] = Cd[0][18];
  Md[46][38] = Cd[0][3];
  Md[46][40] = Cd[0][6];
  Md[46][41] = Cd[0][8];
  Md[46][43] = Cd[0][14];
  Md[46][44] = Cd[0][17];
  Md[46][46] = Cd[0][0];
  Md[46][48] = Cd[0][1];
  Md[46][49] = Cd[0][2];
  Md[46][51] = Cd[0][5];
  Md[46][52] = Cd[0][7];
  Md[46][53] = Cd[0][13];
  Md[46][64] = Cd[0][12];
  Md[46][73] = Cd[0][4];
  Md[46][74] = Cd[0][10];
  Md[46][75] = Cd[0][11];
  Md[46][106] = Cd[0][16];
  Md[47][30] = Cd[0][19];
  Md[47][33] = Cd[0][9];
  Md[47][35] = Cd[0][15];
  Md[47][36] = Cd[0][18];
  Md[47][39] = Cd[0][3];
  Md[47][41] = Cd[0][6];
  Md[47][42] = Cd[0][8];
  Md[47][43] = Cd[0][12];
  Md[47][44] = Cd[0][14];
  Md[47][47] = Cd[0][0];
  Md[47][49] = Cd[0][1];
  Md[47][50] = Cd[0][2];
  Md[47][51] = Cd[0][4];
  Md[47][52] = Cd[0][5];
  Md[47][53] = Cd[0][11];
  Md[47][75] = Cd[0][10];
  Md[47][95] = Cd[0][17];
  Md[47][105] = Cd[0][7];
  Md[47][106] = Cd[0][13];
  Md[47][107] = Cd[0][16];
  Md[48][17] = Cd[0][19];
  Md[48][23] = Cd[0][18];
  Md[48][26] = Cd[0][17];
  Md[48][34] = Cd[0][9];
  Md[48][40] = Cd[0][3];
  Md[48][43] = Cd[0][8];
  Md[48][48] = Cd[0][0];
  Md[48][51] = Cd[0][2];
  Md[48][53] = Cd[0][7];
  Md[48][58] = Cd[0][15];
  Md[48][64] = Cd[0][6];
  Md[48][65] = Cd[0][12];
  Md[48][66] = Cd[0][14];
  Md[48][73] = Cd[0][1];
  Md[48][74] = Cd[0][4];
  Md[48][75] = Cd[0][5];
  Md[48][76] = Cd[0][10];
  Md[48][77] = Cd[0][11];
  Md[48][78] = Cd[0][13];
  Md[48][108] = Cd[0][16];
  Md[49][18] = Cd[0][19];
  Md[49][23] = Cd[0][15];
  Md[49][24] = Cd[0][18];
  Md[49][26] = Cd[0][14];
  Md[49][35] = Cd[0][9];
  Md[49][41] = Cd[0][3];
  Md[49][43] = Cd[0][6];
  Md[49][44] = Cd[0][8];
  Md[49][49] = Cd[0][0];
  Md[49][51] = Cd[0][1];
  Md[49][52] = Cd[0][2];
  Md[49][53] = Cd[0][5];
  Md[49][66] = Cd[0][12];
  Md[49][75] = Cd[0][4];
  Md[49][77] = Cd[0][10];
  Md[49][78] = Cd[0][11];
  Md[49][96] = Cd[0][17];
  Md[49][106] = Cd[0][7];
  Md[49][108] = Cd[0][13];
  Md[49][109] = Cd[0][16];
  Md[50][19] = Cd[0][19];
  Md[50][24] = Cd[0][15];
  Md[50][26] = Cd[0][12];
  Md[50][36] = Cd[0][9];
  Md[50][42] = Cd[0][3];
  Md[50][44] = Cd[0][6];
  Md[50][50] = Cd[0][0];
  Md[50][52] = Cd[0][1];
  Md[50][53] = Cd[0][4];
  Md[50][78] = Cd[0][10];
  Md[50][89] = Cd[0][18];
  Md[50][95] = Cd[0][8];
  Md[50][96] = Cd[0][14];
  Md[50][97] = Cd[0][17];
  Md[50][105] = Cd[0][2];
  Md[50][106] = Cd[0][5];
  Md[50][107] = Cd[0][7];
  Md[50][108] = Cd[0][11];
  Md[50][109] = Cd[0][13];
  Md[50][110] = Cd[0][16];
  Md[51][20] = Cd[0][19];
  Md[51][23] = Cd[0][9];
  Md[51][25] = Cd[0][18];
  Md[51][26] = Cd[0][8];
  Md[51][43] = Cd[0][3];
  Md[51][51] = Cd[0][0];
  Md[51][53] = Cd[0][2];
  Md[51][60] = Cd[0][15];
  Md[51][66] = Cd[0][6];
  Md[51][68] = Cd[0][12];
  Md[51][69] = Cd[0][14];
  Md[51][75] = Cd[0][1];
  Md[51][77] = Cd[0][4];
  Md[51][78] = Cd[0][5];
  Md[51][80] = Cd[0][10];
  Md[51][81] = Cd[0][11];
  Md[51][98] = Cd[0][17];
  Md[51][108] = Cd[0][7];
  Md[51][111] = Cd[0][13];
  Md[51][112] = Cd[0][16];
  Md[52][21] = Cd[0][19];
  Md[52][24] = Cd[0][9];
  Md[52][25] = Cd[0][15];
  Md[52][26] = Cd[0][6];
  Md[52][44] = Cd[0][3];
  Md[52][52] = Cd[0][0];
  Md[52][53] = Cd[0][1];
  Md[52][69] = Cd[0][12];
  Md[52][78] = Cd[0][4];
  Md[52][81] = Cd[0][10];
  Md[52][90] = Cd[0][18];
  Md[52][96] = Cd[0][8];
  Md[52][98] = Cd[0][14];
  Md[52][99] = Cd[0][17];
  Md[52][106] = Cd[0][2];
  Md[52][108] = Cd[0][5];
  Md[52][109] = Cd[0][7];
  Md[52][111] = Cd[0][11];
  Md[52][112] = Cd[0][13];
  Md[52][113] = Cd[0][16];
  Md[53][22] = Cd[0][19];
  Md[53][25] = Cd[0][9];
  Md[53][26] = Cd[0][3];
  Md[53][53] = Cd[0][0];
  Md[53][63] = Cd[0][15];
  Md[53][69] = Cd[0][6];
  Md[53][72] = Cd[0][12];
  Md[53][78] = Cd[0][1];
  Md[53][81] = Cd[0][4];
  Md[53][84] = Cd[0][10];
  Md[53][92] = Cd[0][18];
  Md[53][98] = Cd[0][8];
  Md[53][101] = Cd[0][14];
  Md[53][102] = Cd[0][17];
  Md[53][108] = Cd[0][2];
  Md[53][111] = Cd[0][5];
  Md[53][112] = Cd[0][7];
  Md[53][115] = Cd[0][11];
  Md[53][116] = Cd[0][13];
  Md[53][117] = Cd[0][16];
  Md[54][0] = Cd[1][19];
  Md[54][1] = Cd[1][9];
  Md[54][2] = Cd[1][15];
  Md[54][3] = Cd[1][18];
  Md[54][4] = Cd[1][3];
  Md[54][5] = Cd[1][6];
  Md[54][6] = Cd[1][8];
  Md[54][7] = Cd[1][12];
  Md[54][8] = Cd[1][14];
  Md[54][9] = Cd[1][17];
  Md[54][10] = Cd[1][1];
  Md[54][11] = Cd[1][2];
  Md[54][12] = Cd[1][4];
  Md[54][13] = Cd[1][5];
  Md[54][14] = Cd[1][7];
  Md[54][15] = Cd[1][11];
  Md[54][16] = Cd[1][13];
  Md[54][27] = Cd[1][0];
  Md[54][54] = Cd[1][10];
  Md[54][85] = Cd[1][16];
  Md[55][1] = Cd[1][19];
  Md[55][4] = Cd[1][9];
  Md[55][5] = Cd[1][15];
  Md[55][6] = Cd[1][18];
  Md[55][10] = Cd[1][6];
  Md[55][11] = Cd[1][8];
  Md[55][12] = Cd[1][12];
  Md[55][13] = Cd[1][14];
  Md[55][14] = Cd[1][17];
  Md[55][17] = Cd[1][4];
  Md[55][18] = Cd[1][5];
  Md[55][19] = Cd[1][7];
  Md[55][20] = Cd[1][11];
  Md[55][21] = Cd[1][13];
  Md[55][27] = Cd[1][3];
  Md[55][28] = Cd[1][0];
  Md[55][29] = Cd[1][1];
  Md[55][30] = Cd[1][2];
  Md[55][55] = Cd[1][10];
  Md[55][86] = Cd[1][16];
  Md[56][2] = Cd[1][19];
  Md[56][5] = Cd[1][9];
  Md[56][7] = Cd[1][15];
  Md[56][8] = Cd[1][18];
  Md[56][10] = Cd[1][3];
  Md[56][12] = Cd[1][6];
  Md[56][13] = Cd[1][8];
  Md[56][15] = Cd[1][14];
  Md[56][16] = Cd[1][17];
  Md[56][17] = Cd[1][1];
  Md[56][18] = Cd[1][2];
  Md[56][20] = Cd[1][5];
  Md[56][21] = Cd[1][7];
  Md[56][22] = Cd[1][13];
  Md[56][29] = Cd[1][0];
  Md[56][54] = Cd[1][12];
  Md[56][55] = Cd[1][4];
  Md[56][56] = Cd[1][10];
  Md[56][57] = Cd[1][11];
  Md[56][87] = Cd[1][16];
  Md[57][3] = Cd[1][19];
  Md[57][6] = Cd[1][9];
  Md[57][8] = Cd[1][15];
  Md[57][9] = Cd[1][18];
  Md[57][11] = Cd[1][3];
  Md[57][13] = Cd[1][6];
  Md[57][14] = Cd[1][8];
  Md[57][15] = Cd[1][12];
  Md[57][16] = Cd[1][14];
  Md[57][18] = Cd[1][1];
  Md[57][19] = Cd[1][2];
  Md[57][20] = Cd[1][4];
  Md[57][21] = Cd[1][5];
  Md[57][22] = Cd[1][11];
  Md[57][30] = Cd[1][0];
  Md[57][57] = Cd[1][10];
  Md[57][85] = Cd[1][17];
  Md[57][86] = Cd[1][7];
  Md[57][87] = Cd[1][13];
  Md[57][88] = Cd[1][16];
  Md[58][4] = Cd[1][19];
  Md[58][10] = Cd[1][15];
  Md[58][11] = Cd[1][18];
  Md[58][17] = Cd[1][12];
  Md[58][18] = Cd[1][14];
  Md[58][19] = Cd[1][17];
  Md[58][23] = Cd[1][11];
  Md[58][24] = Cd[1][13];
  Md[58][27] = Cd[1][9];
  Md[58][28] = Cd[1][3];
  Md[58][29] = Cd[1][6];
  Md[58][30] = Cd[1][8];
  Md[58][31] = Cd[1][0];
  Md[58][32] = Cd[1][1];
  Md[58][33] = Cd[1][2];
  Md[58][34] = Cd[1][4];
  Md[58][35] = Cd[1][5];
  Md[58][36] = Cd[1][7];
  Md[58][58] = Cd[1][10];
  Md[58][89] = Cd[1][16];
  Md[59][5] = Cd[1][19];
  Md[59][10] = Cd[1][9];
  Md[59][12] = Cd[1][15];
  Md[59][13] = Cd[1][18];
  Md[59][17] = Cd[1][6];
  Md[59][18] = Cd[1][8];
  Md[59][20] = Cd[1][14];
  Md[59][21] = Cd[1][17];
  Md[59][23] = Cd[1][5];
  Md[59][24] = Cd[1][7];
  Md[59][25] = Cd[1][13];
  Md[59][29] = Cd[1][3];
  Md[59][32] = Cd[1][0];
  Md[59][34] = Cd[1][1];
  Md[59][35] = Cd[1][2];
  Md[59][55] = Cd[1][12];
  Md[59][58] = Cd[1][4];
  Md[59][59] = Cd[1][10];
  Md[59][60] = Cd[1][11];
  Md[59][90] = Cd[1][16];
  Md[60][6] = Cd[1][19];
  Md[60][11] = Cd[1][9];
  Md[60][13] = Cd[1][15];
  Md[60][14] = Cd[1][18];
  Md[60][18] = Cd[1][6];
  Md[60][19] = Cd[1][8];
  Md[60][20] = Cd[1][12];
  Md[60][21] = Cd[1][14];
  Md[60][23] = Cd[1][4];
  Md[60][24] = Cd[1][5];
  Md[60][25] = Cd[1][11];
  Md[60][30] = Cd[1][3];
  Md[60][33] = Cd[1][0];
  Md[60][35] = Cd[1][1];
  Md[60][36] = Cd[1][2];
  Md[60][60] = Cd[1][10];
  Md[60][86] = Cd[1][17];
  Md[60][89] = Cd[1][7];
  Md[60][90] = Cd[1][13];
  Md[60][91] = Cd[1][16];
  Md[61][7] = Cd[1][19];
  Md[61][12] = Cd[1][9];
  Md[61][15] = Cd[1][18];
  Md[61][17] = Cd[1][3];
  Md[61][20] = Cd[1][8];
  Md[61][22] = Cd[1][17];
  Md[61][23] = Cd[1][2];
  Md[61][25] = Cd[1][7];
  Md[61][34] = Cd[1][0];
  Md[61][54] = Cd[1][15];
  Md[61][55] = Cd[1][6];
  Md[61][56] = Cd[1][12];
  Md[61][57] = Cd[1][14];
  Md[61][58] = Cd[1][1];
  Md[61][59] = Cd[1][4];
  Md[61][60] = Cd[1][5];
  Md[61][61] = Cd[1][10];
  Md[61][62] = Cd[1][11];
  Md[61][63] = Cd[1][13];
  Md[61][92] = Cd[1][16];
  Md[62][8] = Cd[1][19];
  Md[62][13] = Cd[1][9];
  Md[62][15] = Cd[1][15];
  Md[62][16] = Cd[1][18];
  Md[62][18] = Cd[1][3];
  Md[62][20] = Cd[1][6];
  Md[62][21] = Cd[1][8];
  Md[62][22] = Cd[1][14];
  Md[62][23] = Cd[1][1];
  Md[62][24] = Cd[1][2];
  Md[62][25] = Cd[1][5];
  Md[62][35] = Cd[1][0];
  Md[62][57] = Cd[1][12];
  Md[62][60] = Cd[1][4];
  Md[62][62] = Cd[1][10];
  Md[62][63] = Cd[1][11];
  Md[62][87] = Cd[1][17];
  Md[62][90] = Cd[1][7];
  Md[62][92] = Cd[1][13];
  Md[62][93] = Cd[1][16];
  Md[63][9] = Cd[1][19];
  Md[63][14] = Cd[1][9];
  Md[63][16] = Cd[1][15];
  Md[63][19] = Cd[1][3];
  Md[63][21] = Cd[1][6];
  Md[63][22] = Cd[1][12];
  Md[63][24] = Cd[1][1];
  Md[63][25] = Cd[1][4];
  Md[63][36] = Cd[1][0];
  Md[63][63] = Cd[1][10];
  Md[63][85] = Cd[1][18];
  Md[63][86] = Cd[1][8];
  Md[63][87] = Cd[1][14];
  Md[63][88] = Cd[1][17];
  Md[63][89] = Cd[1][2];
  Md[63][90] = Cd[1][5];
  Md[63][91] = Cd[1][7];
  Md[63][92] = Cd[1][11];
  Md[63][93] = Cd[1][13];
  Md[63][94] = Cd[1][16];
  Md[64][27] = Cd[1][19];
  Md[64][28] = Cd[1][9];
  Md[64][29] = Cd[1][15];
  Md[64][30] = Cd[1][18];
  Md[64][31] = Cd[1][3];
  Md[64][32] = Cd[1][6];
  Md[64][33] = Cd[1][8];
  Md[64][34] = Cd[1][12];
  Md[64][35] = Cd[1][14];
  Md[64][36] = Cd[1][17];
  Md[64][37] = Cd[1][0];
  Md[64][38] = Cd[1][1];
  Md[64][39] = Cd[1][2];
  Md[64][40] = Cd[1][4];
  Md[64][41] = Cd[1][5];
  Md[64][42] = Cd[1][7];
  Md[64][43] = Cd[1][11];
  Md[64][44] = Cd[1][13];
  Md[64][64] = Cd[1][10];
  Md[64][95] = Cd[1][16];
  Md[65][10] = Cd[1][19];
  Md[65][17] = Cd[1][15];
  Md[65][18] = Cd[1][18];
  Md[65][23] = Cd[1][14];
  Md[65][24] = Cd[1][17];
  Md[65][26] = Cd[1][13];
  Md[65][29] = Cd[1][9];
  Md[65][32] = Cd[1][3];
  Md[65][34] = Cd[1][6];
  Md[65][35] = Cd[1][8];
  Md[65][38] = Cd[1][0];
  Md[65][40] = Cd[1][1];
  Md[65][41] = Cd[1][2];
  Md[65][43] = Cd[1][5];
  Md[65][44] = Cd[1][7];
  Md[65][58] = Cd[1][12];
  Md[65][64] = Cd[1][4];
  Md[65][65] = Cd[1][10];
  Md[65][66] = Cd[1][11];
  Md[65][96] = Cd[1][16];
  Md[66][11] = Cd[1][19];
  Md[66][18] = Cd[1][15];
  Md[66][19] = Cd[1][18];
  Md[66][23] = Cd[1][12];
  Md[66][24] = Cd[1][14];
  Md[66][26] = Cd[1][11];
  Md[66][30] = Cd[1][9];
  Md[66][33] = Cd[1][3];
  Md[66][35] = Cd[1][6];
  Md[66][36] = Cd[1][8];
  Md[66][39] = Cd[1][0];
  Md[66][41] = Cd[1][1];
  Md[66][42] = Cd[1][2];
  Md[66][43] = Cd[1][4];
  Md[66][44] = Cd[1][5];
  Md[66][66] = Cd[1][10];
  Md[66][89] = Cd[1][17];
  Md[66][95] = Cd[1][7];
  Md[66][96] = Cd[1][13];
  Md[66][97] = Cd[1][16];
  Md[67][12] = Cd[1][19];
  Md[67][17] = Cd[1][9];
  Md[67][20] = Cd[1][18];
  Md[67][23] = Cd[1][8];
  Md[67][25] = Cd[1][17];
  Md[67][26] = Cd[1][7];
  Md[67][34] = Cd[1][3];
  Md[67][40] = Cd[1][0];
  Md[67][43] = Cd[1][2];
  Md[67][55] = Cd[1][15];
  Md[67][58] = Cd[1][6];
  Md[67][59] = Cd[1][12];
  Md[67][60] = Cd[1][14];
  Md[67][64] = Cd[1][1];
  Md[67][65] = Cd[1][4];
  Md[67][66] = Cd[1][5];
  Md[67][67] = Cd[1][10];
  Md[67][68] = Cd[1][11];
  Md[67][69] = Cd[1][13];
  Md[67][98] = Cd[1][16];
  Md[68][13] = Cd[1][19];
  Md[68][18] = Cd[1][9];
  Md[68][20] = Cd[1][15];
  Md[68][21] = Cd[1][18];
  Md[68][23] = Cd[1][6];
  Md[68][24] = Cd[1][8];
  Md[68][25] = Cd[1][14];
  Md[68][26] = Cd[1][5];
  Md[68][35] = Cd[1][3];
  Md[68][41] = Cd[1][0];
  Md[68][43] = Cd[1][1];
  Md[68][44] = Cd[1][2];
  Md[68][60] = Cd[1][12];
  Md[68][66] = Cd[1][4];
  Md[68][68] = Cd[1][10];
  Md[68][69] = Cd[1][11];
  Md[68][90] = Cd[1][17];
  Md[68][96] = Cd[1][7];
  Md[68][98] = Cd[1][13];
  Md[68][99] = Cd[1][16];
  Md[69][14] = Cd[1][19];
  Md[69][19] = Cd[1][9];
  Md[69][21] = Cd[1][15];
  Md[69][24] = Cd[1][6];
  Md[69][25] = Cd[1][12];
  Md[69][26] = Cd[1][4];
  Md[69][36] = Cd[1][3];
  Md[69][42] = Cd[1][0];
  Md[69][44] = Cd[1][1];
  Md[69][69] = Cd[1][10];
  Md[69][86] = Cd[1][18];
  Md[69][89] = Cd[1][8];
  Md[69][90] = Cd[1][14];
  Md[69][91] = Cd[1][17];
  Md[69][95] = Cd[1][2];
  Md[69][96] = Cd[1][5];
  Md[69][97] = Cd[1][7];
  Md[69][98] = Cd[1][11];
  Md[69][99] = Cd[1][13];
  Md[69][100] = Cd[1][16];
  Md[70][54] = Cd[1][19];
  Md[70][55] = Cd[1][9];
  Md[70][56] = Cd[1][15];
  Md[70][57] = Cd[1][18];
  Md[70][58] = Cd[1][3];
  Md[70][59] = Cd[1][6];
  Md[70][60] = Cd[1][8];
  Md[70][61] = Cd[1][12];
  Md[70][62] = Cd[1][14];
  Md[70][63] = Cd[1][17];
  Md[70][64] = Cd[1][0];
  Md[70][65] = Cd[1][1];
  Md[70][66] = Cd[1][2];
  Md[70][67] = Cd[1][4];
  Md[70][68] = Cd[1][5];
  Md[70][69] = Cd[1][7];
  Md[70][70] = Cd[1][10];
  Md[70][71] = Cd[1][11];
  Md[70][72] = Cd[1][13];
  Md[70][101] = Cd[1][16];
  Md[71][15] = Cd[1][19];
  Md[71][20] = Cd[1][9];
  Md[71][22] = Cd[1][18];
  Md[71][23] = Cd[1][3];
  Md[71][25] = Cd[1][8];
  Md[71][26] = Cd[1][2];
  Md[71][43] = Cd[1][0];
  Md[71][57] = Cd[1][15];
  Md[71][60] = Cd[1][6];
  Md[71][62] = Cd[1][12];
  Md[71][63] = Cd[1][14];
  Md[71][66] = Cd[1][1];
  Md[71][68] = Cd[1][4];
  Md[71][69] = Cd[1][5];
  Md[71][71] = Cd[1][10];
  Md[71][72] = Cd[1][11];
  Md[71][92] = Cd[1][17];
  Md[71][98] = Cd[1][7];
  Md[71][101] = Cd[1][13];
  Md[71][102] = Cd[1][16];
  Md[72][16] = Cd[1][19];
  Md[72][21] = Cd[1][9];
  Md[72][22] = Cd[1][15];
  Md[72][24] = Cd[1][3];
  Md[72][25] = Cd[1][6];
  Md[72][26] = Cd[1][1];
  Md[72][44] = Cd[1][0];
  Md[72][63] = Cd[1][12];
  Md[72][69] = Cd[1][4];
  Md[72][72] = Cd[1][10];
  Md[72][87] = Cd[1][18];
  Md[72][90] = Cd[1][8];
  Md[72][92] = Cd[1][14];
  Md[72][93] = Cd[1][17];
  Md[72][96] = Cd[1][2];
  Md[72][98] = Cd[1][5];
  Md[72][99] = Cd[1][7];
  Md[72][101] = Cd[1][11];
  Md[72][102] = Cd[1][13];
  Md[72][103] = Cd[1][16];
  Md[73][28] = Cd[1][19];
  Md[73][31] = Cd[1][9];
  Md[73][32] = Cd[1][15];
  Md[73][33] = Cd[1][18];
  Md[73][37] = Cd[1][3];
  Md[73][38] = Cd[1][6];
  Md[73][39] = Cd[1][8];
  Md[73][40] = Cd[1][12];
  Md[73][41] = Cd[1][14];
  Md[73][42] = Cd[1][17];
  Md[73][45] = Cd[1][0];
  Md[73][46] = Cd[1][1];
  Md[73][47] = Cd[1][2];
  Md[73][48] = Cd[1][4];
  Md[73][49] = Cd[1][5];
  Md[73][50] = Cd[1][7];
  Md[73][51] = Cd[1][11];
  Md[73][52] = Cd[1][13];
  Md[73][73] = Cd[1][10];
  Md[73][105] = Cd[1][16];
  Md[74][29] = Cd[1][19];
  Md[74][32] = Cd[1][9];
  Md[74][34] = Cd[1][15];
  Md[74][35] = Cd[1][18];
  Md[74][38] = Cd[1][3];
  Md[74][40] = Cd[1][6];
  Md[74][41] = Cd[1][8];
  Md[74][43] = Cd[1][14];
  Md[74][44] = Cd[1][17];
  Md[74][46] = Cd[1][0];
  Md[74][48] = Cd[1][1];
  Md[74][49] = Cd[1][2];
  Md[74][51] = Cd[1][5];
  Md[74][52] = Cd[1][7];
  Md[74][53] = Cd[1][13];
  Md[74][64] = Cd[1][12];
  Md[74][73] = Cd[1][4];
  Md[74][74] = Cd[1][10];
  Md[74][75] = Cd[1][11];
  Md[74][106] = Cd[1][16];
  Md[75][30] = Cd[1][19];
  Md[75][33] = Cd[1][9];
  Md[75][35] = Cd[1][15];
  Md[75][36] = Cd[1][18];
  Md[75][39] = Cd[1][3];
  Md[75][41] = Cd[1][6];
  Md[75][42] = Cd[1][8];
  Md[75][43] = Cd[1][12];
  Md[75][44] = Cd[1][14];
  Md[75][47] = Cd[1][0];
  Md[75][49] = Cd[1][1];
  Md[75][50] = Cd[1][2];
  Md[75][51] = Cd[1][4];
  Md[75][52] = Cd[1][5];
  Md[75][53] = Cd[1][11];
  Md[75][75] = Cd[1][10];
  Md[75][95] = Cd[1][17];
  Md[75][105] = Cd[1][7];
  Md[75][106] = Cd[1][13];
  Md[75][107] = Cd[1][16];
  Md[76][17] = Cd[1][19];
  Md[76][23] = Cd[1][18];
  Md[76][26] = Cd[1][17];
  Md[76][34] = Cd[1][9];
  Md[76][40] = Cd[1][3];
  Md[76][43] = Cd[1][8];
  Md[76][48] = Cd[1][0];
  Md[76][51] = Cd[1][2];
  Md[76][53] = Cd[1][7];
  Md[76][58] = Cd[1][15];
  Md[76][64] = Cd[1][6];
  Md[76][65] = Cd[1][12];
  Md[76][66] = Cd[1][14];
  Md[76][73] = Cd[1][1];
  Md[76][74] = Cd[1][4];
  Md[76][75] = Cd[1][5];
  Md[76][76] = Cd[1][10];
  Md[76][77] = Cd[1][11];
  Md[76][78] = Cd[1][13];
  Md[76][108] = Cd[1][16];
  Md[77][18] = Cd[1][19];
  Md[77][23] = Cd[1][15];
  Md[77][24] = Cd[1][18];
  Md[77][26] = Cd[1][14];
  Md[77][35] = Cd[1][9];
  Md[77][41] = Cd[1][3];
  Md[77][43] = Cd[1][6];
  Md[77][44] = Cd[1][8];
  Md[77][49] = Cd[1][0];
  Md[77][51] = Cd[1][1];
  Md[77][52] = Cd[1][2];
  Md[77][53] = Cd[1][5];
  Md[77][66] = Cd[1][12];
  Md[77][75] = Cd[1][4];
  Md[77][77] = Cd[1][10];
  Md[77][78] = Cd[1][11];
  Md[77][96] = Cd[1][17];
  Md[77][106] = Cd[1][7];
  Md[77][108] = Cd[1][13];
  Md[77][109] = Cd[1][16];
  Md[78][19] = Cd[1][19];
  Md[78][24] = Cd[1][15];
  Md[78][26] = Cd[1][12];
  Md[78][36] = Cd[1][9];
  Md[78][42] = Cd[1][3];
  Md[78][44] = Cd[1][6];
  Md[78][50] = Cd[1][0];
  Md[78][52] = Cd[1][1];
  Md[78][53] = Cd[1][4];
  Md[78][78] = Cd[1][10];
  Md[78][89] = Cd[1][18];
  Md[78][95] = Cd[1][8];
  Md[78][96] = Cd[1][14];
  Md[78][97] = Cd[1][17];
  Md[78][105] = Cd[1][2];
  Md[78][106] = Cd[1][5];
  Md[78][107] = Cd[1][7];
  Md[78][108] = Cd[1][11];
  Md[78][109] = Cd[1][13];
  Md[78][110] = Cd[1][16];
  Md[79][55] = Cd[1][19];
  Md[79][58] = Cd[1][9];
  Md[79][59] = Cd[1][15];
  Md[79][60] = Cd[1][18];
  Md[79][64] = Cd[1][3];
  Md[79][65] = Cd[1][6];
  Md[79][66] = Cd[1][8];
  Md[79][67] = Cd[1][12];
  Md[79][68] = Cd[1][14];
  Md[79][69] = Cd[1][17];
  Md[79][73] = Cd[1][0];
  Md[79][74] = Cd[1][1];
  Md[79][75] = Cd[1][2];
  Md[79][76] = Cd[1][4];
  Md[79][77] = Cd[1][5];
  Md[79][78] = Cd[1][7];
  Md[79][79] = Cd[1][10];
  Md[79][80] = Cd[1][11];
  Md[79][81] = Cd[1][13];
  Md[79][111] = Cd[1][16];
  Md[80][20] = Cd[1][19];
  Md[80][23] = Cd[1][9];
  Md[80][25] = Cd[1][18];
  Md[80][26] = Cd[1][8];
  Md[80][43] = Cd[1][3];
  Md[80][51] = Cd[1][0];
  Md[80][53] = Cd[1][2];
  Md[80][60] = Cd[1][15];
  Md[80][66] = Cd[1][6];
  Md[80][68] = Cd[1][12];
  Md[80][69] = Cd[1][14];
  Md[80][75] = Cd[1][1];
  Md[80][77] = Cd[1][4];
  Md[80][78] = Cd[1][5];
  Md[80][80] = Cd[1][10];
  Md[80][81] = Cd[1][11];
  Md[80][98] = Cd[1][17];
  Md[80][108] = Cd[1][7];
  Md[80][111] = Cd[1][13];
  Md[80][112] = Cd[1][16];
  Md[81][21] = Cd[1][19];
  Md[81][24] = Cd[1][9];
  Md[81][25] = Cd[1][15];
  Md[81][26] = Cd[1][6];
  Md[81][44] = Cd[1][3];
  Md[81][52] = Cd[1][0];
  Md[81][53] = Cd[1][1];
  Md[81][69] = Cd[1][12];
  Md[81][78] = Cd[1][4];
  Md[81][81] = Cd[1][10];
  Md[81][90] = Cd[1][18];
  Md[81][96] = Cd[1][8];
  Md[81][98] = Cd[1][14];
  Md[81][99] = Cd[1][17];
  Md[81][106] = Cd[1][2];
  Md[81][108] = Cd[1][5];
  Md[81][109] = Cd[1][7];
  Md[81][111] = Cd[1][11];
  Md[81][112] = Cd[1][13];
  Md[81][113] = Cd[1][16];
  Md[82][56] = Cd[1][19];
  Md[82][59] = Cd[1][9];
  Md[82][61] = Cd[1][15];
  Md[82][62] = Cd[1][18];
  Md[82][65] = Cd[1][3];
  Md[82][67] = Cd[1][6];
  Md[82][68] = Cd[1][8];
  Md[82][70] = Cd[1][12];
  Md[82][71] = Cd[1][14];
  Md[82][72] = Cd[1][17];
  Md[82][74] = Cd[1][0];
  Md[82][76] = Cd[1][1];
  Md[82][77] = Cd[1][2];
  Md[82][79] = Cd[1][4];
  Md[82][80] = Cd[1][5];
  Md[82][81] = Cd[1][7];
  Md[82][82] = Cd[1][10];
  Md[82][83] = Cd[1][11];
  Md[82][84] = Cd[1][13];
  Md[82][115] = Cd[1][16];
  Md[83][57] = Cd[1][19];
  Md[83][60] = Cd[1][9];
  Md[83][62] = Cd[1][15];
  Md[83][63] = Cd[1][18];
  Md[83][66] = Cd[1][3];
  Md[83][68] = Cd[1][6];
  Md[83][69] = Cd[1][8];
  Md[83][71] = Cd[1][12];
  Md[83][72] = Cd[1][14];
  Md[83][75] = Cd[1][0];
  Md[83][77] = Cd[1][1];
  Md[83][78] = Cd[1][2];
  Md[83][80] = Cd[1][4];
  Md[83][81] = Cd[1][5];
  Md[83][83] = Cd[1][10];
  Md[83][84] = Cd[1][11];
  Md[83][101] = Cd[1][17];
  Md[83][111] = Cd[1][7];
  Md[83][115] = Cd[1][13];
  Md[83][116] = Cd[1][16];
  Md[84][22] = Cd[1][19];
  Md[84][25] = Cd[1][9];
  Md[84][26] = Cd[1][3];
  Md[84][53] = Cd[1][0];
  Md[84][63] = Cd[1][15];
  Md[84][69] = Cd[1][6];
  Md[84][72] = Cd[1][12];
  Md[84][78] = Cd[1][1];
  Md[84][81] = Cd[1][4];
  Md[84][84] = Cd[1][10];
  Md[84][92] = Cd[1][18];
  Md[84][98] = Cd[1][8];
  Md[84][101] = Cd[1][14];
  Md[84][102] = Cd[1][17];
  Md[84][108] = Cd[1][2];
  Md[84][111] = Cd[1][5];
  Md[84][112] = Cd[1][7];
  Md[84][115] = Cd[1][11];
  Md[84][116] = Cd[1][13];
  Md[84][117] = Cd[1][16];
  Md[85][0] = Cd[2][19];
  Md[85][1] = Cd[2][9];
  Md[85][2] = Cd[2][15];
  Md[85][3] = Cd[2][18];
  Md[85][4] = Cd[2][3];
  Md[85][5] = Cd[2][6];
  Md[85][6] = Cd[2][8];
  Md[85][7] = Cd[2][12];
  Md[85][8] = Cd[2][14];
  Md[85][9] = Cd[2][17];
  Md[85][10] = Cd[2][1];
  Md[85][11] = Cd[2][2];
  Md[85][12] = Cd[2][4];
  Md[85][13] = Cd[2][5];
  Md[85][14] = Cd[2][7];
  Md[85][15] = Cd[2][11];
  Md[85][16] = Cd[2][13];
  Md[85][27] = Cd[2][0];
  Md[85][54] = Cd[2][10];
  Md[85][85] = Cd[2][16];
  Md[86][1] = Cd[2][19];
  Md[86][4] = Cd[2][9];
  Md[86][5] = Cd[2][15];
  Md[86][6] = Cd[2][18];
  Md[86][10] = Cd[2][6];
  Md[86][11] = Cd[2][8];
  Md[86][12] = Cd[2][12];
  Md[86][13] = Cd[2][14];
  Md[86][14] = Cd[2][17];
  Md[86][17] = Cd[2][4];
  Md[86][18] = Cd[2][5];
  Md[86][19] = Cd[2][7];
  Md[86][20] = Cd[2][11];
  Md[86][21] = Cd[2][13];
  Md[86][27] = Cd[2][3];
  Md[86][28] = Cd[2][0];
  Md[86][29] = Cd[2][1];
  Md[86][30] = Cd[2][2];
  Md[86][55] = Cd[2][10];
  Md[86][86] = Cd[2][16];
  Md[87][2] = Cd[2][19];
  Md[87][5] = Cd[2][9];
  Md[87][7] = Cd[2][15];
  Md[87][8] = Cd[2][18];
  Md[87][10] = Cd[2][3];
  Md[87][12] = Cd[2][6];
  Md[87][13] = Cd[2][8];
  Md[87][15] = Cd[2][14];
  Md[87][16] = Cd[2][17];
  Md[87][17] = Cd[2][1];
  Md[87][18] = Cd[2][2];
  Md[87][20] = Cd[2][5];
  Md[87][21] = Cd[2][7];
  Md[87][22] = Cd[2][13];
  Md[87][29] = Cd[2][0];
  Md[87][54] = Cd[2][12];
  Md[87][55] = Cd[2][4];
  Md[87][56] = Cd[2][10];
  Md[87][57] = Cd[2][11];
  Md[87][87] = Cd[2][16];
  Md[88][3] = Cd[2][19];
  Md[88][6] = Cd[2][9];
  Md[88][8] = Cd[2][15];
  Md[88][9] = Cd[2][18];
  Md[88][11] = Cd[2][3];
  Md[88][13] = Cd[2][6];
  Md[88][14] = Cd[2][8];
  Md[88][15] = Cd[2][12];
  Md[88][16] = Cd[2][14];
  Md[88][18] = Cd[2][1];
  Md[88][19] = Cd[2][2];
  Md[88][20] = Cd[2][4];
  Md[88][21] = Cd[2][5];
  Md[88][22] = Cd[2][11];
  Md[88][30] = Cd[2][0];
  Md[88][57] = Cd[2][10];
  Md[88][85] = Cd[2][17];
  Md[88][86] = Cd[2][7];
  Md[88][87] = Cd[2][13];
  Md[88][88] = Cd[2][16];
  Md[89][4] = Cd[2][19];
  Md[89][10] = Cd[2][15];
  Md[89][11] = Cd[2][18];
  Md[89][17] = Cd[2][12];
  Md[89][18] = Cd[2][14];
  Md[89][19] = Cd[2][17];
  Md[89][23] = Cd[2][11];
  Md[89][24] = Cd[2][13];
  Md[89][27] = Cd[2][9];
  Md[89][28] = Cd[2][3];
  Md[89][29] = Cd[2][6];
  Md[89][30] = Cd[2][8];
  Md[89][31] = Cd[2][0];
  Md[89][32] = Cd[2][1];
  Md[89][33] = Cd[2][2];
  Md[89][34] = Cd[2][4];
  Md[89][35] = Cd[2][5];
  Md[89][36] = Cd[2][7];
  Md[89][58] = Cd[2][10];
  Md[89][89] = Cd[2][16];
  Md[90][5] = Cd[2][19];
  Md[90][10] = Cd[2][9];
  Md[90][12] = Cd[2][15];
  Md[90][13] = Cd[2][18];
  Md[90][17] = Cd[2][6];
  Md[90][18] = Cd[2][8];
  Md[90][20] = Cd[2][14];
  Md[90][21] = Cd[2][17];
  Md[90][23] = Cd[2][5];
  Md[90][24] = Cd[2][7];
  Md[90][25] = Cd[2][13];
  Md[90][29] = Cd[2][3];
  Md[90][32] = Cd[2][0];
  Md[90][34] = Cd[2][1];
  Md[90][35] = Cd[2][2];
  Md[90][55] = Cd[2][12];
  Md[90][58] = Cd[2][4];
  Md[90][59] = Cd[2][10];
  Md[90][60] = Cd[2][11];
  Md[90][90] = Cd[2][16];
  Md[91][6] = Cd[2][19];
  Md[91][11] = Cd[2][9];
  Md[91][13] = Cd[2][15];
  Md[91][14] = Cd[2][18];
  Md[91][18] = Cd[2][6];
  Md[91][19] = Cd[2][8];
  Md[91][20] = Cd[2][12];
  Md[91][21] = Cd[2][14];
  Md[91][23] = Cd[2][4];
  Md[91][24] = Cd[2][5];
  Md[91][25] = Cd[2][11];
  Md[91][30] = Cd[2][3];
  Md[91][33] = Cd[2][0];
  Md[91][35] = Cd[2][1];
  Md[91][36] = Cd[2][2];
  Md[91][60] = Cd[2][10];
  Md[91][86] = Cd[2][17];
  Md[91][89] = Cd[2][7];
  Md[91][90] = Cd[2][13];
  Md[91][91] = Cd[2][16];
  Md[92][7] = Cd[2][19];
  Md[92][12] = Cd[2][9];
  Md[92][15] = Cd[2][18];
  Md[92][17] = Cd[2][3];
  Md[92][20] = Cd[2][8];
  Md[92][22] = Cd[2][17];
  Md[92][23] = Cd[2][2];
  Md[92][25] = Cd[2][7];
  Md[92][34] = Cd[2][0];
  Md[92][54] = Cd[2][15];
  Md[92][55] = Cd[2][6];
  Md[92][56] = Cd[2][12];
  Md[92][57] = Cd[2][14];
  Md[92][58] = Cd[2][1];
  Md[92][59] = Cd[2][4];
  Md[92][60] = Cd[2][5];
  Md[92][61] = Cd[2][10];
  Md[92][62] = Cd[2][11];
  Md[92][63] = Cd[2][13];
  Md[92][92] = Cd[2][16];
  Md[93][8] = Cd[2][19];
  Md[93][13] = Cd[2][9];
  Md[93][15] = Cd[2][15];
  Md[93][16] = Cd[2][18];
  Md[93][18] = Cd[2][3];
  Md[93][20] = Cd[2][6];
  Md[93][21] = Cd[2][8];
  Md[93][22] = Cd[2][14];
  Md[93][23] = Cd[2][1];
  Md[93][24] = Cd[2][2];
  Md[93][25] = Cd[2][5];
  Md[93][35] = Cd[2][0];
  Md[93][57] = Cd[2][12];
  Md[93][60] = Cd[2][4];
  Md[93][62] = Cd[2][10];
  Md[93][63] = Cd[2][11];
  Md[93][87] = Cd[2][17];
  Md[93][90] = Cd[2][7];
  Md[93][92] = Cd[2][13];
  Md[93][93] = Cd[2][16];
  Md[94][9] = Cd[2][19];
  Md[94][14] = Cd[2][9];
  Md[94][16] = Cd[2][15];
  Md[94][19] = Cd[2][3];
  Md[94][21] = Cd[2][6];
  Md[94][22] = Cd[2][12];
  Md[94][24] = Cd[2][1];
  Md[94][25] = Cd[2][4];
  Md[94][36] = Cd[2][0];
  Md[94][63] = Cd[2][10];
  Md[94][85] = Cd[2][18];
  Md[94][86] = Cd[2][8];
  Md[94][87] = Cd[2][14];
  Md[94][88] = Cd[2][17];
  Md[94][89] = Cd[2][2];
  Md[94][90] = Cd[2][5];
  Md[94][91] = Cd[2][7];
  Md[94][92] = Cd[2][11];
  Md[94][93] = Cd[2][13];
  Md[94][94] = Cd[2][16];
  Md[95][27] = Cd[2][19];
  Md[95][28] = Cd[2][9];
  Md[95][29] = Cd[2][15];
  Md[95][30] = Cd[2][18];
  Md[95][31] = Cd[2][3];
  Md[95][32] = Cd[2][6];
  Md[95][33] = Cd[2][8];
  Md[95][34] = Cd[2][12];
  Md[95][35] = Cd[2][14];
  Md[95][36] = Cd[2][17];
  Md[95][37] = Cd[2][0];
  Md[95][38] = Cd[2][1];
  Md[95][39] = Cd[2][2];
  Md[95][40] = Cd[2][4];
  Md[95][41] = Cd[2][5];
  Md[95][42] = Cd[2][7];
  Md[95][43] = Cd[2][11];
  Md[95][44] = Cd[2][13];
  Md[95][64] = Cd[2][10];
  Md[95][95] = Cd[2][16];
  Md[96][10] = Cd[2][19];
  Md[96][17] = Cd[2][15];
  Md[96][18] = Cd[2][18];
  Md[96][23] = Cd[2][14];
  Md[96][24] = Cd[2][17];
  Md[96][26] = Cd[2][13];
  Md[96][29] = Cd[2][9];
  Md[96][32] = Cd[2][3];
  Md[96][34] = Cd[2][6];
  Md[96][35] = Cd[2][8];
  Md[96][38] = Cd[2][0];
  Md[96][40] = Cd[2][1];
  Md[96][41] = Cd[2][2];
  Md[96][43] = Cd[2][5];
  Md[96][44] = Cd[2][7];
  Md[96][58] = Cd[2][12];
  Md[96][64] = Cd[2][4];
  Md[96][65] = Cd[2][10];
  Md[96][66] = Cd[2][11];
  Md[96][96] = Cd[2][16];
  Md[97][11] = Cd[2][19];
  Md[97][18] = Cd[2][15];
  Md[97][19] = Cd[2][18];
  Md[97][23] = Cd[2][12];
  Md[97][24] = Cd[2][14];
  Md[97][26] = Cd[2][11];
  Md[97][30] = Cd[2][9];
  Md[97][33] = Cd[2][3];
  Md[97][35] = Cd[2][6];
  Md[97][36] = Cd[2][8];
  Md[97][39] = Cd[2][0];
  Md[97][41] = Cd[2][1];
  Md[97][42] = Cd[2][2];
  Md[97][43] = Cd[2][4];
  Md[97][44] = Cd[2][5];
  Md[97][66] = Cd[2][10];
  Md[97][89] = Cd[2][17];
  Md[97][95] = Cd[2][7];
  Md[97][96] = Cd[2][13];
  Md[97][97] = Cd[2][16];
  Md[98][12] = Cd[2][19];
  Md[98][17] = Cd[2][9];
  Md[98][20] = Cd[2][18];
  Md[98][23] = Cd[2][8];
  Md[98][25] = Cd[2][17];
  Md[98][26] = Cd[2][7];
  Md[98][34] = Cd[2][3];
  Md[98][40] = Cd[2][0];
  Md[98][43] = Cd[2][2];
  Md[98][55] = Cd[2][15];
  Md[98][58] = Cd[2][6];
  Md[98][59] = Cd[2][12];
  Md[98][60] = Cd[2][14];
  Md[98][64] = Cd[2][1];
  Md[98][65] = Cd[2][4];
  Md[98][66] = Cd[2][5];
  Md[98][67] = Cd[2][10];
  Md[98][68] = Cd[2][11];
  Md[98][69] = Cd[2][13];
  Md[98][98] = Cd[2][16];
  Md[99][13] = Cd[2][19];
  Md[99][18] = Cd[2][9];
  Md[99][20] = Cd[2][15];
  Md[99][21] = Cd[2][18];
  Md[99][23] = Cd[2][6];
  Md[99][24] = Cd[2][8];
  Md[99][25] = Cd[2][14];
  Md[99][26] = Cd[2][5];
  Md[99][35] = Cd[2][3];
  Md[99][41] = Cd[2][0];
  Md[99][43] = Cd[2][1];
  Md[99][44] = Cd[2][2];
  Md[99][60] = Cd[2][12];
  Md[99][66] = Cd[2][4];
  Md[99][68] = Cd[2][10];
  Md[99][69] = Cd[2][11];
  Md[99][90] = Cd[2][17];
  Md[99][96] = Cd[2][7];
  Md[99][98] = Cd[2][13];
  Md[99][99] = Cd[2][16];
  Md[100][14] = Cd[2][19];
  Md[100][19] = Cd[2][9];
  Md[100][21] = Cd[2][15];
  Md[100][24] = Cd[2][6];
  Md[100][25] = Cd[2][12];
  Md[100][26] = Cd[2][4];
  Md[100][36] = Cd[2][3];
  Md[100][42] = Cd[2][0];
  Md[100][44] = Cd[2][1];
  Md[100][69] = Cd[2][10];
  Md[100][86] = Cd[2][18];
  Md[100][89] = Cd[2][8];
  Md[100][90] = Cd[2][14];
  Md[100][91] = Cd[2][17];
  Md[100][95] = Cd[2][2];
  Md[100][96] = Cd[2][5];
  Md[100][97] = Cd[2][7];
  Md[100][98] = Cd[2][11];
  Md[100][99] = Cd[2][13];
  Md[100][100] = Cd[2][16];
  Md[101][54] = Cd[2][19];
  Md[101][55] = Cd[2][9];
  Md[101][56] = Cd[2][15];
  Md[101][57] = Cd[2][18];
  Md[101][58] = Cd[2][3];
  Md[101][59] = Cd[2][6];
  Md[101][60] = Cd[2][8];
  Md[101][61] = Cd[2][12];
  Md[101][62] = Cd[2][14];
  Md[101][63] = Cd[2][17];
  Md[101][64] = Cd[2][0];
  Md[101][65] = Cd[2][1];
  Md[101][66] = Cd[2][2];
  Md[101][67] = Cd[2][4];
  Md[101][68] = Cd[2][5];
  Md[101][69] = Cd[2][7];
  Md[101][70] = Cd[2][10];
  Md[101][71] = Cd[2][11];
  Md[101][72] = Cd[2][13];
  Md[101][101] = Cd[2][16];
  Md[102][15] = Cd[2][19];
  Md[102][20] = Cd[2][9];
  Md[102][22] = Cd[2][18];
  Md[102][23] = Cd[2][3];
  Md[102][25] = Cd[2][8];
  Md[102][26] = Cd[2][2];
  Md[102][43] = Cd[2][0];
  Md[102][57] = Cd[2][15];
  Md[102][60] = Cd[2][6];
  Md[102][62] = Cd[2][12];
  Md[102][63] = Cd[2][14];
  Md[102][66] = Cd[2][1];
  Md[102][68] = Cd[2][4];
  Md[102][69] = Cd[2][5];
  Md[102][71] = Cd[2][10];
  Md[102][72] = Cd[2][11];
  Md[102][92] = Cd[2][17];
  Md[102][98] = Cd[2][7];
  Md[102][101] = Cd[2][13];
  Md[102][102] = Cd[2][16];
  Md[103][16] = Cd[2][19];
  Md[103][21] = Cd[2][9];
  Md[103][22] = Cd[2][15];
  Md[103][24] = Cd[2][3];
  Md[103][25] = Cd[2][6];
  Md[103][26] = Cd[2][1];
  Md[103][44] = Cd[2][0];
  Md[103][63] = Cd[2][12];
  Md[103][69] = Cd[2][4];
  Md[103][72] = Cd[2][10];
  Md[103][87] = Cd[2][18];
  Md[103][90] = Cd[2][8];
  Md[103][92] = Cd[2][14];
  Md[103][93] = Cd[2][17];
  Md[103][96] = Cd[2][2];
  Md[103][98] = Cd[2][5];
  Md[103][99] = Cd[2][7];
  Md[103][101] = Cd[2][11];
  Md[103][102] = Cd[2][13];
  Md[103][103] = Cd[2][16];
  Md[104][85] = Cd[2][19];
  Md[104][86] = Cd[2][9];
  Md[104][87] = Cd[2][15];
  Md[104][88] = Cd[2][18];
  Md[104][89] = Cd[2][3];
  Md[104][90] = Cd[2][6];
  Md[104][91] = Cd[2][8];
  Md[104][92] = Cd[2][12];
  Md[104][93] = Cd[2][14];
  Md[104][94] = Cd[2][17];
  Md[104][95] = Cd[2][0];
  Md[104][96] = Cd[2][1];
  Md[104][97] = Cd[2][2];
  Md[104][98] = Cd[2][4];
  Md[104][99] = Cd[2][5];
  Md[104][100] = Cd[2][7];
  Md[104][101] = Cd[2][10];
  Md[104][102] = Cd[2][11];
  Md[104][103] = Cd[2][13];
  Md[104][104] = Cd[2][16];
  Md[105][28] = Cd[2][19];
  Md[105][31] = Cd[2][9];
  Md[105][32] = Cd[2][15];
  Md[105][33] = Cd[2][18];
  Md[105][37] = Cd[2][3];
  Md[105][38] = Cd[2][6];
  Md[105][39] = Cd[2][8];
  Md[105][40] = Cd[2][12];
  Md[105][41] = Cd[2][14];
  Md[105][42] = Cd[2][17];
  Md[105][45] = Cd[2][0];
  Md[105][46] = Cd[2][1];
  Md[105][47] = Cd[2][2];
  Md[105][48] = Cd[2][4];
  Md[105][49] = Cd[2][5];
  Md[105][50] = Cd[2][7];
  Md[105][51] = Cd[2][11];
  Md[105][52] = Cd[2][13];
  Md[105][73] = Cd[2][10];
  Md[105][105] = Cd[2][16];
  Md[106][29] = Cd[2][19];
  Md[106][32] = Cd[2][9];
  Md[106][34] = Cd[2][15];
  Md[106][35] = Cd[2][18];
  Md[106][38] = Cd[2][3];
  Md[106][40] = Cd[2][6];
  Md[106][41] = Cd[2][8];
  Md[106][43] = Cd[2][14];
  Md[106][44] = Cd[2][17];
  Md[106][46] = Cd[2][0];
  Md[106][48] = Cd[2][1];
  Md[106][49] = Cd[2][2];
  Md[106][51] = Cd[2][5];
  Md[106][52] = Cd[2][7];
  Md[106][53] = Cd[2][13];
  Md[106][64] = Cd[2][12];
  Md[106][73] = Cd[2][4];
  Md[106][74] = Cd[2][10];
  Md[106][75] = Cd[2][11];
  Md[106][106] = Cd[2][16];
  Md[107][30] = Cd[2][19];
  Md[107][33] = Cd[2][9];
  Md[107][35] = Cd[2][15];
  Md[107][36] = Cd[2][18];
  Md[107][39] = Cd[2][3];
  Md[107][41] = Cd[2][6];
  Md[107][42] = Cd[2][8];
  Md[107][43] = Cd[2][12];
  Md[107][44] = Cd[2][14];
  Md[107][47] = Cd[2][0];
  Md[107][49] = Cd[2][1];
  Md[107][50] = Cd[2][2];
  Md[107][51] = Cd[2][4];
  Md[107][52] = Cd[2][5];
  Md[107][53] = Cd[2][11];
  Md[107][75] = Cd[2][10];
  Md[107][95] = Cd[2][17];
  Md[107][105] = Cd[2][7];
  Md[107][106] = Cd[2][13];
  Md[107][107] = Cd[2][16];
  Md[108][17] = Cd[2][19];
  Md[108][23] = Cd[2][18];
  Md[108][26] = Cd[2][17];
  Md[108][34] = Cd[2][9];
  Md[108][40] = Cd[2][3];
  Md[108][43] = Cd[2][8];
  Md[108][48] = Cd[2][0];
  Md[108][51] = Cd[2][2];
  Md[108][53] = Cd[2][7];
  Md[108][58] = Cd[2][15];
  Md[108][64] = Cd[2][6];
  Md[108][65] = Cd[2][12];
  Md[108][66] = Cd[2][14];
  Md[108][73] = Cd[2][1];
  Md[108][74] = Cd[2][4];
  Md[108][75] = Cd[2][5];
  Md[108][76] = Cd[2][10];
  Md[108][77] = Cd[2][11];
  Md[108][78] = Cd[2][13];
  Md[108][108] = Cd[2][16];
  Md[109][18] = Cd[2][19];
  Md[109][23] = Cd[2][15];
  Md[109][24] = Cd[2][18];
  Md[109][26] = Cd[2][14];
  Md[109][35] = Cd[2][9];
  Md[109][41] = Cd[2][3];
  Md[109][43] = Cd[2][6];
  Md[109][44] = Cd[2][8];
  Md[109][49] = Cd[2][0];
  Md[109][51] = Cd[2][1];
  Md[109][52] = Cd[2][2];
  Md[109][53] = Cd[2][5];
  Md[109][66] = Cd[2][12];
  Md[109][75] = Cd[2][4];
  Md[109][77] = Cd[2][10];
  Md[109][78] = Cd[2][11];
  Md[109][96] = Cd[2][17];
  Md[109][106] = Cd[2][7];
  Md[109][108] = Cd[2][13];
  Md[109][109] = Cd[2][16];
  Md[110][19] = Cd[2][19];
  Md[110][24] = Cd[2][15];
  Md[110][26] = Cd[2][12];
  Md[110][36] = Cd[2][9];
  Md[110][42] = Cd[2][3];
  Md[110][44] = Cd[2][6];
  Md[110][50] = Cd[2][0];
  Md[110][52] = Cd[2][1];
  Md[110][53] = Cd[2][4];
  Md[110][78] = Cd[2][10];
  Md[110][89] = Cd[2][18];
  Md[110][95] = Cd[2][8];
  Md[110][96] = Cd[2][14];
  Md[110][97] = Cd[2][17];
  Md[110][105] = Cd[2][2];
  Md[110][106] = Cd[2][5];
  Md[110][107] = Cd[2][7];
  Md[110][108] = Cd[2][11];
  Md[110][109] = Cd[2][13];
  Md[110][110] = Cd[2][16];
  Md[111][55] = Cd[2][19];
  Md[111][58] = Cd[2][9];
  Md[111][59] = Cd[2][15];
  Md[111][60] = Cd[2][18];
  Md[111][64] = Cd[2][3];
  Md[111][65] = Cd[2][6];
  Md[111][66] = Cd[2][8];
  Md[111][67] = Cd[2][12];
  Md[111][68] = Cd[2][14];
  Md[111][69] = Cd[2][17];
  Md[111][73] = Cd[2][0];
  Md[111][74] = Cd[2][1];
  Md[111][75] = Cd[2][2];
  Md[111][76] = Cd[2][4];
  Md[111][77] = Cd[2][5];
  Md[111][78] = Cd[2][7];
  Md[111][79] = Cd[2][10];
  Md[111][80] = Cd[2][11];
  Md[111][81] = Cd[2][13];
  Md[111][111] = Cd[2][16];
  Md[112][20] = Cd[2][19];
  Md[112][23] = Cd[2][9];
  Md[112][25] = Cd[2][18];
  Md[112][26] = Cd[2][8];
  Md[112][43] = Cd[2][3];
  Md[112][51] = Cd[2][0];
  Md[112][53] = Cd[2][2];
  Md[112][60] = Cd[2][15];
  Md[112][66] = Cd[2][6];
  Md[112][68] = Cd[2][12];
  Md[112][69] = Cd[2][14];
  Md[112][75] = Cd[2][1];
  Md[112][77] = Cd[2][4];
  Md[112][78] = Cd[2][5];
  Md[112][80] = Cd[2][10];
  Md[112][81] = Cd[2][11];
  Md[112][98] = Cd[2][17];
  Md[112][108] = Cd[2][7];
  Md[112][111] = Cd[2][13];
  Md[112][112] = Cd[2][16];
  Md[113][21] = Cd[2][19];
  Md[113][24] = Cd[2][9];
  Md[113][25] = Cd[2][15];
  Md[113][26] = Cd[2][6];
  Md[113][44] = Cd[2][3];
  Md[113][52] = Cd[2][0];
  Md[113][53] = Cd[2][1];
  Md[113][69] = Cd[2][12];
  Md[113][78] = Cd[2][4];
  Md[113][81] = Cd[2][10];
  Md[113][90] = Cd[2][18];
  Md[113][96] = Cd[2][8];
  Md[113][98] = Cd[2][14];
  Md[113][99] = Cd[2][17];
  Md[113][106] = Cd[2][2];
  Md[113][108] = Cd[2][5];
  Md[113][109] = Cd[2][7];
  Md[113][111] = Cd[2][11];
  Md[113][112] = Cd[2][13];
  Md[113][113] = Cd[2][16];
  Md[114][86] = Cd[2][19];
  Md[114][89] = Cd[2][9];
  Md[114][90] = Cd[2][15];
  Md[114][91] = Cd[2][18];
  Md[114][95] = Cd[2][3];
  Md[114][96] = Cd[2][6];
  Md[114][97] = Cd[2][8];
  Md[114][98] = Cd[2][12];
  Md[114][99] = Cd[2][14];
  Md[114][100] = Cd[2][17];
  Md[114][105] = Cd[2][0];
  Md[114][106] = Cd[2][1];
  Md[114][107] = Cd[2][2];
  Md[114][108] = Cd[2][4];
  Md[114][109] = Cd[2][5];
  Md[114][110] = Cd[2][7];
  Md[114][111] = Cd[2][10];
  Md[114][112] = Cd[2][11];
  Md[114][113] = Cd[2][13];
  Md[114][114] = Cd[2][16];
  Md[115][56] = Cd[2][19];
  Md[115][59] = Cd[2][9];
  Md[115][61] = Cd[2][15];
  Md[115][62] = Cd[2][18];
  Md[115][65] = Cd[2][3];
  Md[115][67] = Cd[2][6];
  Md[115][68] = Cd[2][8];
  Md[115][70] = Cd[2][12];
  Md[115][71] = Cd[2][14];
  Md[115][72] = Cd[2][17];
  Md[115][74] = Cd[2][0];
  Md[115][76] = Cd[2][1];
  Md[115][77] = Cd[2][2];
  Md[115][79] = Cd[2][4];
  Md[115][80] = Cd[2][5];
  Md[115][81] = Cd[2][7];
  Md[115][82] = Cd[2][10];
  Md[115][83] = Cd[2][11];
  Md[115][84] = Cd[2][13];
  Md[115][115] = Cd[2][16];
  Md[116][57] = Cd[2][19];
  Md[116][60] = Cd[2][9];
  Md[116][62] = Cd[2][15];
  Md[116][63] = Cd[2][18];
  Md[116][66] = Cd[2][3];
  Md[116][68] = Cd[2][6];
  Md[116][69] = Cd[2][8];
  Md[116][71] = Cd[2][12];
  Md[116][72] = Cd[2][14];
  Md[116][75] = Cd[2][0];
  Md[116][77] = Cd[2][1];
  Md[116][78] = Cd[2][2];
  Md[116][80] = Cd[2][4];
  Md[116][81] = Cd[2][5];
  Md[116][83] = Cd[2][10];
  Md[116][84] = Cd[2][11];
  Md[116][101] = Cd[2][17];
  Md[116][111] = Cd[2][7];
  Md[116][115] = Cd[2][13];
  Md[116][116] = Cd[2][16];
  Md[117][22] = Cd[2][19];
  Md[117][25] = Cd[2][9];
  Md[117][26] = Cd[2][3];
  Md[117][53] = Cd[2][0];
  Md[117][63] = Cd[2][15];
  Md[117][69] = Cd[2][6];
  Md[117][72] = Cd[2][12];
  Md[117][78] = Cd[2][1];
  Md[117][81] = Cd[2][4];
  Md[117][84] = Cd[2][10];
  Md[117][92] = Cd[2][18];
  Md[117][98] = Cd[2][8];
  Md[117][101] = Cd[2][14];
  Md[117][102] = Cd[2][17];
  Md[117][108] = Cd[2][2];
  Md[117][111] = Cd[2][5];
  Md[117][112] = Cd[2][7];
  Md[117][115] = Cd[2][11];
  Md[117][116] = Cd[2][13];
  Md[117][117] = Cd[2][16];
  Md[118][87] = Cd[2][19];
  Md[118][90] = Cd[2][9];
  Md[118][92] = Cd[2][15];
  Md[118][93] = Cd[2][18];
  Md[118][96] = Cd[2][3];
  Md[118][98] = Cd[2][6];
  Md[118][99] = Cd[2][8];
  Md[118][101] = Cd[2][12];
  Md[118][102] = Cd[2][14];
  Md[118][103] = Cd[2][17];
  Md[118][106] = Cd[2][0];
  Md[118][108] = Cd[2][1];
  Md[118][109] = Cd[2][2];
  Md[118][111] = Cd[2][4];
  Md[118][112] = Cd[2][5];
  Md[118][113] = Cd[2][7];
  Md[118][115] = Cd[2][10];
  Md[118][116] = Cd[2][11];
  Md[118][117] = Cd[2][13];
  Md[118][118] = Cd[2][16];
  Md[119][88] = Cd[2][19];
  Md[119][91] = Cd[2][9];
  Md[119][93] = Cd[2][15];
  Md[119][94] = Cd[2][18];
  Md[119][97] = Cd[2][3];
  Md[119][99] = Cd[2][6];
  Md[119][100] = Cd[2][8];
  Md[119][102] = Cd[2][12];
  Md[119][103] = Cd[2][14];
  Md[119][104] = Cd[2][17];
  Md[119][107] = Cd[2][0];
  Md[119][109] = Cd[2][1];
  Md[119][110] = Cd[2][2];
  Md[119][112] = Cd[2][4];
  Md[119][113] = Cd[2][5];
  Md[119][114] = Cd[2][7];
  Md[119][116] = Cd[2][10];
  Md[119][117] = Cd[2][11];
  Md[119][118] = Cd[2][13];
  Md[119][119] = Cd[2][16];

function_terminate:

   return error;
}

Rox_ErrorCode rox_extract_real_solutions_macaulay(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double M)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double Q = NULL;

   Rox_Array2D_Double A = NULL;
   Rox_Array2D_Double B = NULL;
   Rox_Array2D_Double C = NULL;
   Rox_Array2D_Double D = NULL;
   Rox_Array2D_Double E = NULL;
   Rox_Array2D_Double Di = NULL;

   Rox_Array2D_Double re = NULL;
   Rox_MatSO3 Rek = NULL;

   // Compute the n real eigenvalues d and eigenvectors V
   // of the Q matrix of size m x m
   Rox_ObjSet_Array2D_Double V = NULL; // size V will be m x n
   Rox_DynVec_Double e = NULL; // size e will be 1 x n

   if (!Re || !M ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Extract submatrices

   error = rox_array2d_double_new(&Q , 27, 27);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Di, 93, 93);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&E , 27, 93);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &A, M,  0,  0, 27, 27 );            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &B, M,  0, 27, 27, 93 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &C, M, 27,  0, 93, 27 );            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &D, M, 27, 27, 93, 93 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svdinverse(Di, D);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(E, B, Di);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(Q, E, C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_substract(Q, A, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new(&e, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_new(&V, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_real_eigenvalues_eigenvectors(e, V, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ////////////////////////////////////////////////////////////////

   Rox_Uint nb_used = 0;
   error = rox_dynvec_double_get_used(&nb_used, e);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&re, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Array2D_Double * cd = NULL;
   error = rox_objset_array2d_double_get_data_pointer ( &cd, V );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k=0; k <nb_used; k++)
   {
      Rox_Double v1 = 0;
      Rox_Double v2 = 0;
      Rox_Double v3 = 0;
      Rox_Double v4 = 0;
      Rox_Double v7 = 0;
      Rox_Double v9 = 0;
      Rox_Double xn = 0;
      Rox_Double yn = 0;
      Rox_Double zn = 0;

      error = rox_array2d_double_get_value(&v1, cd[k], 1, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&v2, cd[k], 2, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&v3, cd[k], 3, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&v4, cd[k], 4, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&v7, cd[k], 7, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&v9, cd[k], 9, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (fabs(v1) > 10e-8)
      {
         xn = v4/v1;
      }
      else
      {
         xn = 0;
      }

      if (fabs(v2) > 10e-8)
      {
         yn = v7/v2;
      }
      else
      {
         yn = 0;
      }

      if (fabs(v3) > 10e-8)
      {
         zn = v9/v3;
      }
      else
      {
         zn = 0;
      }

      error = rox_array2d_double_set_value(re, 0, 0, xn);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_set_value(re, 1, 0, yn);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_set_value(re, 2, 0, zn);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matso3_new(&Rek);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_matso3_from_vector_gibbs(Rek, re);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // rox_matrix_print(Rek);

      error = rox_objset_array2d_double_append(Re, Rek);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:

   rox_array2d_double_del(&Q);
   rox_array2d_double_del(&A);
   rox_array2d_double_del(&B);
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&D);
   rox_array2d_double_del(&E);
   rox_array2d_double_del(&Di);

   rox_dynvec_double_del(&e);
   rox_objset_array2d_double_del(&V);

   rox_array2d_double_del(&re);

   return error;
}

Rox_ErrorCode rox_compute_valid_solutions(Rox_ObjSet_Array2D_Double Te, Rox_ObjSet_Array2D_Double ze, Rox_DynVec_Double ce, const Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double P, const Rox_Array2D_Double W, const Rox_Point2D_Double q, const Rox_Point3D_Double m, const Rox_Uint nbp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double vR = NULL;
   Rox_Array2D_Double R = NULL;
   Rox_Array2D_Double t = NULL;
   Rox_Array2D_Double g = NULL;

   Rox_Uint max_real_solutions = 0;
   Rox_Uint good_solution = 0;
   Rox_Uint count_sols = 0;

   if (!Te || !ze || !ce || !Re || !P || !W || !q || !m) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_objset_array2d_double_get_used(&max_real_solutions, Re);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_new(&vR, 9, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_new(&R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&t, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&g, nbp, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Array2D_Double * Red = NULL;
   error = rox_objset_array2d_double_get_data_pointer ( &Red, Re );
   ROX_ERROR_CHECK_TERMINATE ( error );

   count_sols = 0;
   for (Rox_Uint k = 0; k < max_real_solutions; k++)
   {
      // Assign rotation R = Re(:,:,k);
      error = rox_array2d_double_copy(R, Red[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // vR = [R(1,1:3)'; R(2,1:3)'; R(3,1:3)'];
      error = rox_vector_col_copy(vR, R);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute translation t = P * vR;
      error = rox_array2d_double_mulmatmat(t, P, vR);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute normalized depths g = W * vR;
      error = rox_array2d_double_mulmatmat(g, W, vR);
      ROX_ERROR_CHECK_TERMINATE ( error );

      Rox_Double ** gd = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer ( &gd, g );
      ROX_ERROR_CHECK_TERMINATE ( error );

      good_solution = 1;
      for (Rox_Uint i = 0; i < nbp; i++)
      {
         // The g must be positive
         if (gd[i][0] < 0.0)
         {
            good_solution = 0;
            break;
         }
      }

      if (good_solution == 1)
      {
         Rox_Double cost = 0;
         Rox_MatSE3 T = NULL;
         Rox_Array2D_Double z = NULL;

         error = rox_matse3_new(&T);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_array2d_double_new(&z, 1, nbp);
         ROX_ERROR_CHECK_TERMINATE ( error );

         Rox_Double ** zd = NULL;
         error = rox_array2d_double_get_data_pointer_to_pointer( &zd, z);
         ROX_ERROR_CHECK_TERMINATE ( error );

         for (Rox_Uint i = 0; i < nbp; i++)
         {
            Rox_Double x = q[i].u;
            Rox_Double y = q[i].v;
            Rox_Double nq = sqrt(x*x+y*y+1.0);
            zd[0][i] = gd[i][0] / nq;
         }

         error = rox_matse3_set_matso3_r3(T, R, t);
         ROX_ERROR_CHECK_TERMINATE ( error );

         count_sols = count_sols + 1;
         //T(:,:,count_sols) = [R, t; 0, 0, 0, 1];
         //z(count_sols, :) = gamma.*normq;

         // compute cost final_solutions
         error = rox_cost_function(&cost, T, z, q, m, nbp);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_double_append(Te, T);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_double_append(ze, z);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_dynvec_double_append(ce, &cost);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

   error = rox_sort_solutions_ascending_cost(Te, ze, ce);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matrix_del(&vR);
   rox_matrix_del(&g);
   rox_matrix_del(&t);
   rox_matrix_del(&R);

   return error;
}


Rox_ErrorCode rox_sort_solutions_ascending_cost(Rox_ObjSet_Array2D_Double Te, Rox_ObjSet_Array2D_Double ze, Rox_DynVec_Double ce)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!Te || !ze || !ce) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   /////////////////////////////////////////////////////////////////////////////////
   // Sort in ascending order
   Rox_Array2D_Double cost_ascending = NULL;

   Rox_ObjSet_Array2D_Double Te_sort = NULL;
   Rox_ObjSet_Array2D_Double ze_sort = NULL;

   Rox_Uint nsol = 0;
   error = rox_dynvec_double_get_used(&nsol, ce);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&cost_ascending, 1, nsol);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double * ce_data = NULL;
   error = rox_dynvec_double_get_data_pointer ( &ce_data, ce );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** cost_ascending_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &cost_ascending_data, cost_ascending );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k=0; k<nsol; k++)
   {
      cost_ascending_data[0][k] = ce_data[k];
   }

   // Order solution by increasing cost function
   error = rox_array2d_double_asort(cost_ascending);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_new(&Te_sort, nsol);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_new(&ze_sort, nsol);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k=0; k<nsol; k++)
   {
      for (Rox_Uint i=0; i<nsol; i++)
      {
         if (fabs(ce_data[i]-cost_ascending_data[0][k]) < 1e-15)
         {
            Rox_Array2D_Double Te_copy = NULL;
            Rox_Array2D_Double ze_copy = NULL;

            error = rox_array2d_double_new_copy(&Te_copy, Te->data[i]);
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_array2d_double_new_copy(&ze_copy, ze->data[i]);
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_objset_array2d_double_append(Te_sort, Te_copy);
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_objset_array2d_double_append(ze_sort, ze_copy);
            ROX_ERROR_CHECK_TERMINATE ( error );
         }
     }
   }

   for (Rox_Uint k=0; k<nsol; k++)
   {
      error = rox_array2d_double_copy(Te->data[k], Te_sort->data[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_copy(ze->data[k], ze_sort->data[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // error = rox_objset_array2d_double_clone(Te, Te_sort);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   //error = rox_objset_array2d_double_clone(ze, ze_sort);
   //ROX_ERROR_CHECK_TERMINATE ( error );


function_terminate:
   rox_objset_array2d_double_del(&Te_sort);
   rox_objset_array2d_double_del(&ze_sort);
   rox_array2d_double_del(&cost_ascending);
   return error;
}

Rox_ErrorCode rox_cost_function(Rox_Double * cost, Rox_MatSE3 T, Rox_Array2D_Double z, Rox_Point2D_Double q, const Rox_Point3D_Double m, const Rox_Uint nbp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cost || !T || !z || !q || !m ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // R = T(1:3,1:3);
   // t = T(1:3,4:4);
   Rox_Double ** Td = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Td, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** zd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &zd, z);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *cost = 0.0;
   for (Rox_Uint k = 0; k < nbp; k++)
   {
      // *cost += norm( z(1,k) * q(:,k) - R * m(1:3,k) - t)^2;

      Rox_Double x = q[k].u;
      Rox_Double y = q[k].v;

      Rox_Double Xm = m[k].X;
      Rox_Double Ym = m[k].Y;
      Rox_Double Zm = m[k].Z;

      Rox_Double Xc = Td[0][0]*Xm+Td[0][1]*Ym+Td[0][2]*Zm+Td[0][3];
      Rox_Double Xi = zd[0][k]*x;

      Rox_Double Yc = Td[1][0]*Xm+Td[1][1]*Ym+Td[1][2]*Zm+Td[1][3];
      Rox_Double Yi = zd[0][k]*y;

      Rox_Double Zc = Td[2][0]*Xm+Td[2][1]*Ym+Td[2][2]*Zm+Td[2][3];
      Rox_Double Zi = zd[0][k]*1;

      *cost += (Xi-Xc)*(Xi-Xc) + (Yi-Yc)*(Yi-Yc) + (Zi-Zc)*(Zi-Zc);
   }

function_terminate:

   return error;
}
