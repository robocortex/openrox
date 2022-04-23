//==============================================================================
//
//    OPENROX   : File matso3_from_vectors.c
//
//    Contents  : Implementation of matso3_from_vectors module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "matso3_from_vectors.h"

#include <baseproc/maths/maths_macros.h>

#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/maths/linalg/matrix.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/solve/svd_solve.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/solve/symm3x3solve.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <baseproc/array/robust/tukey.h>
#include <generated/dynvec_point3d_double_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_matso3_from_vectors_cayley(Rox_MatSO3 R, Rox_DynVec_Point3D_Double ref, Rox_DynVec_Point3D_Double cur);

Rox_ErrorCode rox_matso3_from_vectors_quaternion(Rox_MatSO3 R, Rox_DynVec_Point3D_Double ref, Rox_DynVec_Point3D_Double cur);

Rox_ErrorCode rox_matso3_from_vectors(Rox_MatSO3 R, const Rox_DynVec_Point3D_Double ref, const Rox_DynVec_Point3D_Double cur)
{
   // The cayley parametrization does not work for rotation angles of pi (around any axis)
   // It is better to use the quaternion parametrization
   // return rox_matso3_from_vectors_cayley(R, ref, cur);
   return rox_matso3_from_vectors_quaternion(R, ref, cur);
}

Rox_ErrorCode rox_matso3_from_vectors_cayley(Rox_MatSO3 R, const Rox_DynVec_Point3D_Double ref, const Rox_DynVec_Point3D_Double cur)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double S = NULL, A = NULL, Ak = NULL, b = NULL, bk = NULL, bt = NULL, x= NULL;
   Rox_Array2D_Double sc = NULL, sr = NULL;
   Rox_Array2D_Double d = NULL, w = NULL, wb1 = NULL, wb2 = NULL;
   Rox_Double * dw = NULL;

   if ((ref->used == 0) || (ref->used != cur->used))
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Be sure that is a matso3 matrix
   error = rox_matso3_check_size ( R );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_fillunit(R);
   ROX_ERROR_CHECK_TERMINATE(error)

   // New matrices for solving the linear system
   error = rox_array2d_double_new(&S, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&A, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&Ak, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&sc, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&sr, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&b, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&bk, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&bt, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&x, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&d, ref->used, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&wb1, ref->used, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&wb2, ref->used, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new(&w, ref->used, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Compute distances, put in appropriate function rox_point3d_double_distance
   for (Rox_Uint k = 0; k < ref->used; k++)
   {
      double dx = ref->data[k].X - cur->data[k].X;
      double dy = ref->data[k].Y - cur->data[k].Y;
      double dz = ref->data[k].Z - cur->data[k].Z;
      double dist = sqrt(dx*dx+dy*dy+dz*dz);
      rox_array2d_double_set_value(d, k, 0, dist);
   }

   error = rox_array2d_double_tukey(w, wb1, wb2, d);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_get_data_pointer( &dw, w );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(A, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(b, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k = 0; k < ref->used; k++)
   {
      //dw[k]=1.0;
      rox_array2d_double_set_value(sr, 0, 0, ref->data[k].X);
      rox_array2d_double_set_value(sr, 1, 0, ref->data[k].Y);
      rox_array2d_double_set_value(sr, 2, 0, ref->data[k].Z);

      rox_array2d_double_set_value(sc, 0, 0, cur->data[k].X);
      rox_array2d_double_set_value(sc, 1, 0, cur->data[k].Y);
      rox_array2d_double_set_value(sc, 2, 0, cur->data[k].Z);

      // Use bk as a temporary vector to add sr and sc
      rox_array2d_double_add(bk, sr, sc);
      rox_transformtools_skew_from_vector(Ak, bk);

      // True use of bk as the sustraction of sr and sc
      rox_array2d_double_substract(bk, sr, sc);
      rox_array2d_double_scale(Ak, Ak, dw[k]);
      rox_array2d_double_scale(bk, bk, dw[k]);

      rox_array2d_double_mulmattransmat(S, Ak, Ak);
      rox_array2d_double_add(A, A, S);

      rox_array2d_double_mulmattransmat(bt, Ak, bk);
      rox_array2d_double_add(b, b, bt);
   }

   error = rox_array2d_double_symm3x3_solve(x, A, b);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matso3_from_vector_gibbs(R, x);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   ROX_ERROR_CHECK(rox_array2d_double_del(&S));
   ROX_ERROR_CHECK(rox_array2d_double_del(&A));
   ROX_ERROR_CHECK(rox_array2d_double_del(&Ak));
   ROX_ERROR_CHECK(rox_array2d_double_del(&b));
   ROX_ERROR_CHECK(rox_array2d_double_del(&bk));
   ROX_ERROR_CHECK(rox_array2d_double_del(&bt));
   ROX_ERROR_CHECK(rox_array2d_double_del(&x));
   ROX_ERROR_CHECK(rox_array2d_double_del(&sc));
   ROX_ERROR_CHECK(rox_array2d_double_del(&sr));
   ROX_ERROR_CHECK(rox_array2d_double_del(&d));
   ROX_ERROR_CHECK(rox_array2d_double_del(&w));
   ROX_ERROR_CHECK(rox_array2d_double_del(&wb1));
   ROX_ERROR_CHECK(rox_array2d_double_del(&wb2));

   return error;
}

// See matlab implementation
Rox_ErrorCode rox_matso3_from_vectors_quaternion ( Rox_MatSO3 R, Rox_DynVec_Point3D_Double ref, Rox_DynVec_Point3D_Double cur)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double Q = NULL, U = NULL, S = NULL, V = NULL;
   Rox_Array2D_Double A = NULL, Ak = NULL, As = NULL, b = NULL, bk = NULL;
   // Rox_Array2D_Double d = NULL, w = NULL, wb1 = NULL, wb2 = NULL;
   // Rox_Double * dw = NULL;
   Rox_Double traceA;

   if ((ref->used == 0) || (ref->used != cur->used))
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Be sure that is a matso3 matrix
   error = rox_array2d_double_check_size ( R, 3, 3 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // New matrices for solving the linear system
   error = rox_array2d_double_new(&U, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&S, 4, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&V, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&Q, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&As, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&A, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Ak, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&b, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&bk, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(Q, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(A, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(b, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Ak_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Ak_data, Ak);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** bk_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &bk_data, bk);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build the 4 x 4 symmetrix matrix
   for (Rox_Uint k = 0; k < ref->used; k++)
   {
      // dw[k] = 1.0;

      Rox_Double rx = ref->data[k].X;
      Rox_Double ry = ref->data[k].Y;
      Rox_Double rz = ref->data[k].Z;

      Rox_Double cx = cur->data[k].X;
      Rox_Double cy = cur->data[k].Y;
      Rox_Double cz = cur->data[k].Z;

      // Ak = vr[k] * vc[k]'
      Ak_data[0][0] = cx*rx; Ak_data[0][1] = cy*rx; Ak_data[0][2] = cz*rx;
      Ak_data[1][0] = cx*ry, Ak_data[1][1] = cy*ry, Ak_data[1][2] = cz*ry;
      Ak_data[2][0] = cx*rz, Ak_data[2][1] = cy*rz, Ak_data[2][2] = cz*rz;

      // bk = skew(vr[k]) * vc[k]
      bk_data[0][0] = cz*ry - cy*rz;
      bk_data[1][0] = cx*rz - cz*rx;
      bk_data[2][0] = cy*rx - cx*ry;

      // Cumulate
      error = rox_array2d_double_add(A, A, Ak);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add(b, b, bk);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_matrix_trace(&traceA, A); // traceA = trace(A);
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_matrix_add_matmattrans(As, A, A); // As = A + A'
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Q_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Q_data, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** As_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &As_data, As);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** b_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &b_data, b);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Build the matrix Q =  [trace(A), b'; b, (A'+A)-trace(A)*eye(3)];
   Q_data[0][0] = traceA;        Q_data[0][1] = b_data[0][0];           Q_data[0][2] = b_data[0][1];           Q_data[0][3] = b_data[0][2];
   Q_data[1][0] = b_data[0][0];  Q_data[1][1] = As_data[0][0]-traceA;   Q_data[1][2] = As_data[0][1];          Q_data[1][3] = As_data[0][2];
   Q_data[2][0] = b_data[0][1];  Q_data[2][1] = As_data[1][0];          Q_data[2][2] = As_data[1][1]-traceA;   Q_data[2][3] = As_data[1][2];
   Q_data[3][0] = b_data[0][2];  Q_data[3][1] = As_data[2][0];          Q_data[3][2] = As_data[2][1];          Q_data[3][3] = As_data[2][2]-traceA;

   // Decompose the Q matrix with SVD
   error = rox_array2d_double_svd(U, S, V, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // be sure that V is sorted
   error = rox_array2d_double_svd_sort_SV(S, V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the best quaternion that is the first column of V (corresponding to the higher singular value)
   Rox_Double ** V_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &V_data, V );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the rotation matrix from quaternion q = [V_data[0][0], V_data[1][0], V_data[2][0], V_data[3][0]];
   error = rox_transformtools_rotationmatrix_from_quaternion(R, V_data[0][0], V_data[1][0], V_data[2][0], V_data[3][0]);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   ROX_ERROR_CHECK(rox_array2d_double_del(&U));
   ROX_ERROR_CHECK(rox_array2d_double_del(&S));
   ROX_ERROR_CHECK(rox_array2d_double_del(&V));
   ROX_ERROR_CHECK(rox_array2d_double_del(&Q));
   ROX_ERROR_CHECK(rox_array2d_double_del(&A));
   ROX_ERROR_CHECK(rox_array2d_double_del(&Ak));
   ROX_ERROR_CHECK(rox_array2d_double_del(&As));
   ROX_ERROR_CHECK(rox_array2d_double_del(&b));
   ROX_ERROR_CHECK(rox_array2d_double_del(&bk));

   return error;
}
