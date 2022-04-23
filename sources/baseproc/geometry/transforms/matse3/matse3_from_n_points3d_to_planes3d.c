//==============================================================================
//
//    OPENROX   : File matse3_from_n_points3d_to_planes3d.c
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

#include "matse3_from_n_points3d_to_planes3d.h"

#include <math.h>
#include <float.h>

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
#include <baseproc/array/multiply/mulmatmattrans.h>

#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>

#include <baseproc/array/eigenv/real_eigenvalues_eigenvectors.h>

#include <baseproc/array/inverse/mat3x3inv.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/inverse/inverse_lu.h>

#include <baseproc/array/median/median.h>

#include <baseproc/array/norm/norm2sq.h>

#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillzero.h>
#include <baseproc/array/fill/fillunit.h>

#include <baseproc/geometry/transforms/transform_tools.h>

#include <baseproc/geometry/plane/plane_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <baseproc/maths/linalg/matso3.h>

#include <inout/numeric/array2d_save.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

#include <system/time/timer.h>

// Internal functions
Rox_ErrorCode rox_sort_solutions_ascending_cost_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Te, Rox_DynVec_Double ce);

Rox_ErrorCode rox_solve_gibbs_parametrization_numeric_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double Ar, const Rox_Array2D_Double br);
Rox_ErrorCode rox_solve_quaternion_parametrization_numeric_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double Ar, const Rox_Array2D_Double br);

Rox_ErrorCode rox_compute_coefficients_gibbs_parametrization_points3d_to_planes3d(Rox_Array2D_Double C, const Rox_Array2D_Double Ar, const Rox_Array2D_Double br);
Rox_ErrorCode rox_compute_coefficients_quaternion_parametrization_points3d_to_planes3d(Rox_Array2D_Double C, const Rox_Array2D_Double Ar, const Rox_Array2D_Double br);

Rox_ErrorCode rox_solve_gibbs_equations_system_macaulay_resultant_numeric_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double C);
Rox_ErrorCode rox_solve_quaternion_equations_system_macaulay_resultant_numeric_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double C);

Rox_ErrorCode rox_matrix_macaulay_gibbs_points3d_to_planes3d(Rox_Array2D_Double M, const Rox_Array2D_Double u, const Rox_Array2D_Double C);
Rox_ErrorCode rox_matrix_macaulay_quaternion_points3d_to_planes3d(Rox_Array2D_Double M, const Rox_Array2D_Double u, const Rox_Array2D_Double C);

Rox_ErrorCode rox_extract_real_gibbs_solutions_macaulay_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double M);
Rox_ErrorCode rox_extract_real_quaternion_solutions_macaulay_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double M);

Rox_ErrorCode rox_compute_normalized_data_points3d_to_planes3d(Rox_Array2D_Double At, Rox_Array2D_Double bt, Rox_Array2D_Double Ar, Rox_Array2D_Double br, Rox_Array2D_Double cr, const Rox_Plane3D_Double p, const Rox_Point3D_Double m, const Rox_Sint n);
Rox_ErrorCode rox_compute_valid_solutions_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Te, Rox_DynVec_Double ce, const Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double At, const Rox_Array2D_Double bt, const Rox_Array2D_Double Ar, const Rox_Array2D_Double br, const Rox_Array2D_Double cr, const Rox_Plane3D_Double p, const Rox_Point3D_Double m, const Rox_Uint nbp);

Rox_ErrorCode rox_check_twofold_solutions ( Rox_Sint * new_solution, Rox_ObjSet_Array2D_Double re, Rox_Array2D_Double rek);

Rox_ErrorCode rox_array2d_double_normalize_rows(Rox_Array2D_Double C);

// Exported functions
Rox_ErrorCode rox_matse3_from_n_points3d_to_planes3d_double(Rox_ObjSet_Array2D_Double T, const Rox_Plane3D_Double p, const Rox_Point3D_Double m, const Rox_Sint n)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nb_used = 0;

   Rox_Array2D_Double Ar = NULL;
   Rox_Array2D_Double br = NULL;
   Rox_Array2D_Double cr = NULL;
   Rox_Array2D_Double At = NULL;
   Rox_Array2D_Double bt = NULL;
   Rox_ObjSet_Array2D_Double R = NULL;
   Rox_DynVec_Double cost = NULL;
   
   if (!T || !m || !p) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&Ar, 9, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&br, 9, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&cr, 1, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&At, 3, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&bt, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_compute_normalized_data_points3d_to_planes3d(At, bt, Ar, br, cr, p, m, n);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create an object set for rotation
   error = rox_objset_array2d_double_new(&R, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create a dynvec for cost function
   error = rox_dynvec_double_new(&cost, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //error = rox_solve_gibbs_parametrization_numeric_points3d_to_planes3d(R, Ar, br);
   //ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_solve_quaternion_parametrization_numeric_points3d_to_planes3d(R, Ar, br);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_get_used(&nb_used, R);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_compute_valid_solutions_points3d_to_planes3d(T, cost, R, At, bt, Ar, br, cr, p, m, n);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_get_used(&nb_used, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&At);
   rox_array2d_double_del(&bt);
   rox_array2d_double_del(&Ar);
   rox_array2d_double_del(&br);
   rox_array2d_double_del(&cr);

   rox_objset_array2d_double_del(&R);
   rox_dynvec_double_del(&cost);

   return error;
}


// Compute At and bt such that t = At * rv + bt
// Compute Ar, br and cr such that rv' * Ar * rv + br' * rv + cr = 0

Rox_ErrorCode rox_compute_normalized_data_points3d_to_planes3d(Rox_Array2D_Double At, Rox_Array2D_Double bt, Rox_Array2D_Double Ar, Rox_Array2D_Double br, Rox_Array2D_Double cr, const Rox_Plane3D_Double p, const Rox_Point3D_Double m, const Rox_Sint n)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double Att = NULL;
   Rox_Array2D_Double Atti = NULL;
   Rox_Array2D_Double At_k = NULL;
   Rox_Array2D_Double bt_k = NULL;
   Rox_Array2D_Double Ar_k = NULL;
   Rox_Array2D_Double br_k = NULL;
   Rox_Array2D_Double N = NULL;
   Rox_Array2D_Double s = NULL;
   Rox_Array2D_Double v = NULL;
   Rox_Array2D_Double h = NULL;

   if (!At || !bt || !Ar || !br || !cr || !m || !p) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&Att, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero(Att);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Attd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Attd, Att);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new( &N, 3, n );            
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** Nd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Nd, N);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute Att
   for (Rox_Sint k = 0; k < n; k++)
   {
      Rox_Double nx = p[k].a;
      Rox_Double ny = p[k].b;
      Rox_Double nz = p[k].c;

      Nd[0][k] = nx;
      Nd[1][k] = ny;
      Nd[2][k] = nz;

      Attd[0][0] += nx*nx; Attd[0][1] += nx*ny; Attd[0][2] += nx*nz;
      Attd[1][0] += nx*ny; Attd[1][1] += ny*ny; Attd[1][2] += ny*nz;
      Attd[2][0] += nx*nz; Attd[2][1] += ny*nz; Attd[2][2] += nz*nz;
   }

   // Compute Atti = inv(Att)
   error = rox_array2d_double_new(&Atti, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mat3x3_inverse(Atti, Att);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&v, n, n);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&s, 3, n);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmat(s, Atti, N);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // Compute V

   error = rox_array2d_double_new(&At_k, 3, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&bt_k, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** At_kd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &At_kd, At_k);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** bt_kd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &bt_kd, bt_k);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** vd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &vd, v);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** sd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &sd, s);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero(At);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero(bt);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute At and bt such that t = At * r + bt
   for (Rox_Sint k = 0; k < n; k++)
   {
      Rox_Double X = m[k].X;
      Rox_Double Y = m[k].Y;
      Rox_Double Z = m[k].Z;      
      
      Rox_Double nx = p[k].a;
      Rox_Double ny = p[k].b;
      Rox_Double nz = p[k].c;

      vd[k][0] = nx * X; vd[k][1] = nx * Y; vd[k][2] = nx * Z;
      vd[k][3] = ny * X; vd[k][4] = ny * Y; vd[k][5] = ny * Z;
      vd[k][6] = nz * X; vd[k][7] = nz * Y; vd[k][8] = nz * Z;

      At_kd[0][0] = sd[0][k]*vd[k][0]; At_kd[0][1] = sd[0][k]*vd[k][1]; At_kd[0][2] = sd[0][k]*vd[k][2]; At_kd[0][3] = sd[0][k]*vd[k][3]; At_kd[0][4] = sd[0][k]*vd[k][4]; At_kd[0][5] = sd[0][k]*vd[k][5]; At_kd[0][6] = sd[0][k]*vd[k][6]; At_kd[0][7] = sd[0][k]*vd[k][7]; At_kd[0][8] = sd[0][k]*vd[k][8];
      At_kd[1][0] = sd[1][k]*vd[k][0]; At_kd[1][1] = sd[1][k]*vd[k][1]; At_kd[1][2] = sd[1][k]*vd[k][2]; At_kd[1][3] = sd[1][k]*vd[k][3]; At_kd[1][4] = sd[1][k]*vd[k][4]; At_kd[1][5] = sd[1][k]*vd[k][5]; At_kd[1][6] = sd[1][k]*vd[k][6]; At_kd[1][7] = sd[1][k]*vd[k][7]; At_kd[1][8] = sd[1][k]*vd[k][8];
      At_kd[2][0] = sd[2][k]*vd[k][0]; At_kd[2][1] = sd[2][k]*vd[k][1]; At_kd[2][2] = sd[2][k]*vd[k][2]; At_kd[2][3] = sd[2][k]*vd[k][3]; At_kd[2][4] = sd[2][k]*vd[k][4]; At_kd[2][5] = sd[2][k]*vd[k][5]; At_kd[2][6] = sd[2][k]*vd[k][6]; At_kd[2][7] = sd[2][k]*vd[k][7]; At_kd[2][8] = sd[2][k]*vd[k][8];

      bt_kd[0][0] = p[k].d * sd[0][k]; 
      bt_kd[1][0] = p[k].d * sd[1][k]; 
      bt_kd[2][0] = p[k].d * sd[2][k]; 

      error = rox_array2d_double_substract(At, At, At_k);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_substract(bt, bt, bt_k);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Compute matrix Ar such that cost function = r' * Ar * r + br' * r + cr
   error = rox_array2d_double_new(&Ar_k, 9, 9);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&br_k, 9, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Ar_kd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Ar_kd, Ar_k);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** br_kd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &br_kd, br_k);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero(Ar);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero(br);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillzero(cr);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&h, 9, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** hd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &hd, h);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Atd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Atd, At);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** btd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &btd, bt);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint k=0; k<n; k++)
   {
      Rox_Double nx = p[k].a;
      Rox_Double ny = p[k].b;
      Rox_Double nz = p[k].c;
      Rox_Double  d = p[k].d;

      Rox_Double gd = d + nx * btd[0][0] + ny * btd[1][0] + nz * btd[2][0];

      hd[0][0] = vd[k][0] + Atd[0][0] * nx + Atd[1][0] * ny + Atd[2][0] * nz;
      hd[1][0] = vd[k][1] + Atd[0][1] * nx + Atd[1][1] * ny + Atd[2][1] * nz;
      hd[2][0] = vd[k][2] + Atd[0][2] * nx + Atd[1][2] * ny + Atd[2][2] * nz;
      hd[3][0] = vd[k][3] + Atd[0][3] * nx + Atd[1][3] * ny + Atd[2][3] * nz;
      hd[4][0] = vd[k][4] + Atd[0][4] * nx + Atd[1][4] * ny + Atd[2][4] * nz;
      hd[5][0] = vd[k][5] + Atd[0][5] * nx + Atd[1][5] * ny + Atd[2][5] * nz;
      hd[6][0] = vd[k][6] + Atd[0][6] * nx + Atd[1][6] * ny + Atd[2][6] * nz;
      hd[7][0] = vd[k][7] + Atd[0][7] * nx + Atd[1][7] * ny + Atd[2][7] * nz;
      hd[8][0] = vd[k][8] + Atd[0][8] * nx + Atd[1][8] * ny + Atd[2][8] * nz;

      error = rox_array2d_double_mulmatmattrans(Ar_k, h, h);
      ROX_ERROR_CHECK_TERMINATE ( error );

      br_kd[0][0] = 2 * gd * hd[0][0];
      br_kd[1][0] = 2 * gd * hd[1][0];
      br_kd[2][0] = 2 * gd * hd[2][0];
      br_kd[3][0] = 2 * gd * hd[3][0];
      br_kd[4][0] = 2 * gd * hd[4][0];
      br_kd[5][0] = 2 * gd * hd[5][0];
      br_kd[6][0] = 2 * gd * hd[6][0];
      br_kd[7][0] = 2 * gd * hd[7][0];
      br_kd[8][0] = 2 * gd * hd[8][0];

      error = rox_array2d_double_add(Ar, Ar, Ar_k);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add(br, br, br_k);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&Att );
   rox_array2d_double_del(&Atti);
   rox_array2d_double_del(&At_k);
   rox_array2d_double_del(&bt_k);
   rox_array2d_double_del(&Ar_k);
   rox_array2d_double_del(&br_k);
   rox_array2d_double_del(&N   );
   rox_array2d_double_del(&s   );
   rox_array2d_double_del(&v   );
   rox_array2d_double_del(&h   );

   return error;
}

Rox_ErrorCode rox_solve_quaternion_parametrization_numeric_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double Ar, const Rox_Array2D_Double br)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double C = NULL;

   if (!Ar || !br || !Re ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&C, 5, 35);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_compute_coefficients_quaternion_parametrization_points3d_to_planes3d(C, Ar, br);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_normalize_rows(C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_solve_quaternion_equations_system_macaulay_resultant_numeric_points3d_to_planes3d(Re, C);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&C);

   return error;
}

Rox_ErrorCode rox_compute_coefficients_quaternion_parametrization_points3d_to_planes3d(Rox_Array2D_Double C, const Rox_Array2D_Double Ar, const Rox_Array2D_Double br)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!Ar || !br || !C ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** Cd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Cd, C );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Ad = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Ad, Ar );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** bd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &bd, br );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero(C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // row-wise
   Cd[0][4] = 1; Cd[0][18] = 1; Cd[0][27] = 1; Cd[0][32] = 1; Cd[0][34] = -1;
   Cd[1][5] = 4*Ad[5][5] - 4*Ad[5][7] - 4*Ad[7][5] + 4*Ad[7][7]; 
   Cd[1][6] = 4*Ad[2][7] - 4*Ad[5][2] - 4*Ad[2][5] + 4*Ad[7][2] + 4*Ad[5][6] + 4*Ad[6][5] - 4*Ad[6][7] - 4*Ad[7][6]; 
   Cd[1][7] = 4*Ad[1][5] + 4*Ad[5][1] - 4*Ad[1][7] - 4*Ad[3][5] - 4*Ad[5][3] - 4*Ad[7][1] + 4*Ad[3][7] + 4*Ad[7][3]; 
   Cd[1][9] = 4*Ad[2][2] - 4*Ad[2][6] - 4*Ad[6][2] + 4*Ad[6][6]; 
   Cd[1][10] = 4*Ad[2][3] - 4*Ad[2][1] - 4*Ad[1][2] + 4*Ad[3][2] + 4*Ad[1][6] + 4*Ad[6][1] - 4*Ad[3][6] - 4*Ad[6][3]; 
   Cd[1][12] = 4*Ad[1][1] - 4*Ad[1][3] - 4*Ad[3][1] + 4*Ad[3][3]; 
   Cd[1][15] = 2*Ad[4][5] + 2*Ad[5][4] - 2*Ad[4][7] - 2*Ad[7][4] + 2*Ad[5][8] + 2*Ad[8][5] - 2*Ad[7][8] - 2*Ad[8][7]; 
   Cd[1][16] = 2*Ad[1][7] - 2*Ad[2][4] - 2*Ad[4][2] - 2*Ad[5][1] - 2*Ad[1][5] - 2*Ad[3][5] - 2*Ad[5][3] + 2*Ad[7][1] - 2*Ad[2][8] + 2*Ad[3][7] + 2*Ad[4][6] + 2*Ad[6][4] + 2*Ad[7][3] - 2*Ad[8][2] + 2*Ad[6][8] + 2*Ad[8][6]; 
   Cd[1][17] = 2*Ad[1][4] + 2*Ad[4][1] - 2*Ad[2][5] - 2*Ad[3][4] - 2*Ad[4][3] - 2*Ad[5][2] + 2*Ad[1][8] + 2*Ad[2][7] + 2*Ad[7][2] + 2*Ad[8][1] - 2*Ad[3][8] - 2*Ad[5][6] - 2*Ad[6][5] - 2*Ad[8][3] + 2*Ad[6][7] + 2*Ad[7][6]; 
   Cd[1][19] = 2*Ad[1][2] + 2*Ad[2][1] + 2*Ad[0][5] + 2*Ad[2][3] + 2*Ad[3][2] + 2*Ad[5][0] - 2*Ad[0][7] - 2*Ad[1][6] - 2*Ad[6][1] - 2*Ad[7][0] - 2*Ad[3][6] - 2*Ad[6][3] + 2*Ad[5][8] + 2*Ad[8][5] - 2*Ad[7][8] - 2*Ad[8][7]; 
   Cd[1][20] = 4*Ad[2][2] - 4*Ad[1][1] + 4*Ad[3][3] - 4*Ad[5][5] - 4*Ad[6][6] + 4*Ad[7][7]; 
   Cd[1][22] = 2*Ad[0][5] - 2*Ad[2][1] - 2*Ad[1][2] + 2*Ad[2][3] + 2*Ad[3][2] + 2*Ad[5][0] - 2*Ad[0][7] - 2*Ad[1][6] - 2*Ad[6][1] - 2*Ad[7][0] + 2*Ad[3][6] + 2*Ad[4][5] + 2*Ad[5][4] + 2*Ad[6][3] - 2*Ad[4][7] - 2*Ad[7][4]; 
   Cd[1][24] = Ad[0][7] - Ad[5][0] - Ad[0][5] + Ad[7][0] - Ad[4][5] - Ad[5][4] + Ad[4][7] + Ad[7][4] - Ad[5][8] - Ad[8][5] + Ad[7][8] + Ad[8][7] - bd[5][0] + bd[7][0]; 
   Cd[1][25] = 2*Ad[0][6] - 2*Ad[2][0] - 2*Ad[0][2] + 2*Ad[6][0] - 2*Ad[2][8] - 2*Ad[8][2] + 2*Ad[6][8] + 2*Ad[8][6]; 
   Cd[1][26] = 2*Ad[0][1] + 2*Ad[1][0] - 2*Ad[0][3] - 2*Ad[3][0] + 2*Ad[2][5] + 2*Ad[5][2] + 2*Ad[1][8] + 2*Ad[2][7] + 2*Ad[7][2] + 2*Ad[8][1] - 2*Ad[3][8] - 2*Ad[5][6] - 2*Ad[6][5] - 2*Ad[8][3] - 2*Ad[6][7] - 2*Ad[7][6]; 
   Cd[1][28] = 2*Ad[0][6] - 2*Ad[2][0] - 2*Ad[0][2] - 2*Ad[1][5] - 2*Ad[2][4] - 2*Ad[4][2] - 2*Ad[5][1] + 2*Ad[6][0] - 2*Ad[1][7] + 2*Ad[3][5] + 2*Ad[5][3] - 2*Ad[7][1] + 2*Ad[3][7] + 2*Ad[4][6] + 2*Ad[6][4] + 2*Ad[7][3]; 
   Cd[1][30] = Ad[0][2] + Ad[2][0] - Ad[0][6] + Ad[2][4] + Ad[4][2] - Ad[6][0] + Ad[2][8] - Ad[4][6] - Ad[6][4] + Ad[8][2] - Ad[6][8] - Ad[8][6] + bd[2][0] - bd[6][0]; 
   Cd[1][31] = 2*Ad[0][1] + 2*Ad[1][0] - 2*Ad[0][3] - 2*Ad[3][0] + 2*Ad[1][4] + 2*Ad[4][1] - 2*Ad[3][4] - 2*Ad[4][3]; 
   Cd[1][33] = Ad[0][3] - Ad[1][0] - Ad[0][1] + Ad[3][0] - Ad[1][4] - Ad[4][1] + Ad[3][4] + Ad[4][3] - Ad[1][8] - Ad[8][1] + Ad[3][8] + Ad[8][3] - bd[1][0] + bd[3][0]; 
   
   Cd[2][1] = 4*Ad[5][5] - 4*Ad[5][7] - 4*Ad[7][5] + 4*Ad[7][7]; 
   Cd[2][2] = 2*Ad[2][7] - 2*Ad[5][2] - 2*Ad[2][5] + 2*Ad[7][2] + 2*Ad[5][6] + 2*Ad[6][5] - 2*Ad[6][7] - 2*Ad[7][6]; 
   Cd[2][3] = 2*Ad[1][5] + 2*Ad[5][1] - 2*Ad[1][7] - 2*Ad[3][5] - 2*Ad[5][3] - 2*Ad[7][1] + 2*Ad[3][7] + 2*Ad[7][3]; 
   Cd[2][5] = 6*Ad[4][5] + 6*Ad[5][4] - 6*Ad[4][7] - 6*Ad[7][4] + 6*Ad[5][8] + 6*Ad[8][5] - 6*Ad[7][8] - 6*Ad[8][7]; 
   Cd[2][6] = 4*Ad[1][7] - 4*Ad[2][4] - 4*Ad[4][2] - 4*Ad[5][1] - 4*Ad[1][5] - 4*Ad[3][5] - 4*Ad[5][3] + 4*Ad[7][1] - 4*Ad[2][8] + 4*Ad[3][7] + 4*Ad[4][6] + 4*Ad[6][4] + 4*Ad[7][3] - 4*Ad[8][2] + 4*Ad[6][8] + 4*Ad[8][6]; 
   Cd[2][7] = 4*Ad[1][4] + 4*Ad[4][1] - 4*Ad[2][5] - 4*Ad[3][4] - 4*Ad[4][3] - 4*Ad[5][2] + 4*Ad[1][8] + 4*Ad[2][7] + 4*Ad[7][2] + 4*Ad[8][1] - 4*Ad[3][8] - 4*Ad[5][6] - 4*Ad[6][5] - 4*Ad[8][3] + 4*Ad[6][7] + 4*Ad[7][6]; 
   Cd[2][9] = 2*Ad[1][2] + 2*Ad[2][1] + 2*Ad[0][5] + 2*Ad[2][3] + 2*Ad[3][2] + 2*Ad[5][0] - 2*Ad[0][7] - 2*Ad[1][6] - 2*Ad[6][1] - 2*Ad[7][0] - 2*Ad[3][6] - 2*Ad[6][3] + 2*Ad[5][8] + 2*Ad[8][5] - 2*Ad[7][8] - 2*Ad[8][7]; 
   Cd[2][10] = 4*Ad[2][2] - 4*Ad[1][1] + 4*Ad[3][3] - 4*Ad[5][5] - 4*Ad[6][6] + 4*Ad[7][7]; 
   Cd[2][12] = 2*Ad[0][5] - 2*Ad[2][1] - 2*Ad[1][2] + 2*Ad[2][3] + 2*Ad[3][2] + 2*Ad[5][0] - 2*Ad[0][7] - 2*Ad[1][6] - 2*Ad[6][1] - 2*Ad[7][0] + 2*Ad[3][6] + 2*Ad[4][5] + 2*Ad[5][4] + 2*Ad[6][3] - 2*Ad[4][7] - 2*Ad[7][4]; 
   Cd[2][14] = Ad[0][7] - Ad[5][0] - Ad[0][5] + Ad[7][0] - Ad[4][5] - Ad[5][4] + Ad[4][7] + Ad[7][4] - Ad[5][8] - Ad[8][5] + Ad[7][8] + Ad[8][7] - bd[5][0] + bd[7][0]; 
   Cd[2][15] = 8*Ad[4][4] + 8*Ad[4][8] + 8*Ad[8][4] + 8*Ad[8][8]; 
   Cd[2][16] = - 6*Ad[1][4] - 6*Ad[4][1] - 6*Ad[3][4] - 6*Ad[4][3] - 6*Ad[1][8] - 6*Ad[8][1] - 6*Ad[3][8] - 6*Ad[8][3]; 
   Cd[2][17] = - 6*Ad[2][4] - 6*Ad[4][2] - 6*Ad[2][8] - 6*Ad[4][6] - 6*Ad[6][4] - 6*Ad[8][2] - 6*Ad[6][8] - 6*Ad[8][6]; 
   Cd[2][19] = 4*Ad[1][1] + 4*Ad[0][4] + 4*Ad[1][3] + 4*Ad[3][1] + 4*Ad[4][0] + 4*Ad[3][3] + 4*Ad[0][8] + 4*Ad[8][0] + 4*Ad[4][8] + 4*Ad[8][4] + 8*Ad[8][8]; 
   Cd[2][20] = 4*Ad[1][2] + 4*Ad[2][1] + 4*Ad[2][3] + 4*Ad[3][2] + 4*Ad[1][6] + 4*Ad[6][1] + 4*Ad[3][6] - 4*Ad[4][5] - 4*Ad[5][4] + 4*Ad[6][3] - 4*Ad[4][7] - 4*Ad[7][4] - 4*Ad[5][8] - 4*Ad[8][5] - 4*Ad[7][8] - 4*Ad[8][7]; 
   Cd[2][22] = 4*Ad[0][4] + 4*Ad[2][2] + 4*Ad[4][0] + 4*Ad[0][8] + 4*Ad[2][6] + 8*Ad[4][4] + 4*Ad[6][2] + 4*Ad[8][0] + 4*Ad[4][8] + 4*Ad[6][6] + 4*Ad[8][4]; 
   Cd[2][24] = - 2*Ad[0][4] - 2*Ad[4][0] - 2*Ad[0][8] - 4*Ad[4][4] - 2*Ad[8][0] - 4*Ad[4][8] - 4*Ad[8][4] - 4*Ad[8][8] - 2*bd[4][0] - 2*bd[8][0]; 
   Cd[2][25] = - 2*Ad[0][1] - 2*Ad[1][0] - 2*Ad[0][3] - 2*Ad[3][0] - 2*Ad[1][8] - 2*Ad[8][1] - 2*Ad[3][8] - 2*Ad[8][3]; 
   Cd[2][26] = 2*Ad[1][5] - 2*Ad[2][0] - 2*Ad[0][6] - 2*Ad[0][2] + 2*Ad[5][1] - 2*Ad[6][0] + 2*Ad[1][7] + 2*Ad[3][5] + 2*Ad[5][3] + 2*Ad[7][1] - 2*Ad[2][8] + 2*Ad[3][7] + 2*Ad[7][3] - 2*Ad[8][2] - 2*Ad[6][8] - 2*Ad[8][6]; 
   Cd[2][28] = 2*Ad[2][5] - 2*Ad[1][0] - 2*Ad[0][3] - 2*Ad[3][0] - 2*Ad[1][4] - 2*Ad[4][1] - 2*Ad[0][1] - 2*Ad[3][4] - 2*Ad[4][3] + 2*Ad[5][2] + 2*Ad[2][7] + 2*Ad[7][2] + 2*Ad[5][6] + 2*Ad[6][5] + 2*Ad[6][7] + 2*Ad[7][6]; 
   Cd[2][30] = Ad[0][1] + Ad[1][0] + Ad[0][3] + Ad[3][0] + Ad[1][4] + Ad[4][1] + Ad[3][4] + Ad[4][3] + Ad[1][8] + Ad[8][1] + Ad[3][8] + Ad[8][3] + bd[1][0] + bd[3][0]; 
   Cd[2][31] = - 2*Ad[0][2] - 2*Ad[2][0] - 2*Ad[0][6] - 2*Ad[2][4] - 2*Ad[4][2] - 2*Ad[6][0] - 2*Ad[4][6] - 2*Ad[6][4]; 
   Cd[2][33] = Ad[0][2] + Ad[2][0] + Ad[0][6] + Ad[2][4] + Ad[4][2] + Ad[6][0] + Ad[2][8] + Ad[4][6] + Ad[6][4] + Ad[8][2] + Ad[6][8] + Ad[8][6] + bd[2][0] + bd[6][0]; 
   
   Cd[3][1] = 2*Ad[2][7] - 2*Ad[5][2] - 2*Ad[2][5] + 2*Ad[7][2] + 2*Ad[5][6] + 2*Ad[6][5] - 2*Ad[6][7] - 2*Ad[7][6]; 
   Cd[3][2] = 4*Ad[2][2] - 4*Ad[2][6] - 4*Ad[6][2] + 4*Ad[6][6]; 
   Cd[3][3] = 2*Ad[2][3] - 2*Ad[2][1] - 2*Ad[1][2] + 2*Ad[3][2] + 2*Ad[1][6] + 2*Ad[6][1] - 2*Ad[3][6] - 2*Ad[6][3]; 
   Cd[3][5] = 2*Ad[1][7] - 2*Ad[2][4] - 2*Ad[4][2] - 2*Ad[5][1] - 2*Ad[1][5] - 2*Ad[3][5] - 2*Ad[5][3] + 2*Ad[7][1] - 2*Ad[2][8] + 2*Ad[3][7] + 2*Ad[4][6] + 2*Ad[6][4] + 2*Ad[7][3] - 2*Ad[8][2] + 2*Ad[6][8] + 2*Ad[8][6]; 
   Cd[3][6] = 4*Ad[1][2] + 4*Ad[2][1] + 4*Ad[0][5] + 4*Ad[2][3] + 4*Ad[3][2] + 4*Ad[5][0] - 4*Ad[0][7] - 4*Ad[1][6] - 4*Ad[6][1] - 4*Ad[7][0] - 4*Ad[3][6] - 4*Ad[6][3] + 4*Ad[5][8] + 4*Ad[8][5] - 4*Ad[7][8] - 4*Ad[8][7]; 
   Cd[3][7] = 4*Ad[2][2] - 4*Ad[1][1] + 4*Ad[3][3] - 4*Ad[5][5] - 4*Ad[6][6] + 4*Ad[7][7]; 
   Cd[3][9] = 6*Ad[0][6] - 6*Ad[2][0] - 6*Ad[0][2] + 6*Ad[6][0] - 6*Ad[2][8] - 6*Ad[8][2] + 6*Ad[6][8] + 6*Ad[8][6]; 
   Cd[3][10] = 4*Ad[0][1] + 4*Ad[1][0] - 4*Ad[0][3] - 4*Ad[3][0] + 4*Ad[2][5] + 4*Ad[5][2] + 4*Ad[1][8] + 4*Ad[2][7] + 4*Ad[7][2] + 4*Ad[8][1] - 4*Ad[3][8] - 4*Ad[5][6] - 4*Ad[6][5] - 4*Ad[8][3] - 4*Ad[6][7] - 4*Ad[7][6]; 
   Cd[3][12] = 2*Ad[0][6] - 2*Ad[2][0] - 2*Ad[0][2] - 2*Ad[1][5] - 2*Ad[2][4] - 2*Ad[4][2] - 2*Ad[5][1] + 2*Ad[6][0] - 2*Ad[1][7] + 2*Ad[3][5] + 2*Ad[5][3] - 2*Ad[7][1] + 2*Ad[3][7] + 2*Ad[4][6] + 2*Ad[6][4] + 2*Ad[7][3]; 
   Cd[3][14] = Ad[0][2] + Ad[2][0] - Ad[0][6] + Ad[2][4] + Ad[4][2] - Ad[6][0] + Ad[2][8] - Ad[4][6] - Ad[6][4] + Ad[8][2] - Ad[6][8] - Ad[8][6] + bd[2][0] - bd[6][0]; 
   Cd[3][15] = - 2*Ad[1][4] - 2*Ad[4][1] - 2*Ad[3][4] - 2*Ad[4][3] - 2*Ad[1][8] - 2*Ad[8][1] - 2*Ad[3][8] - 2*Ad[8][3]; 
   Cd[3][16] = 4*Ad[1][1] + 4*Ad[0][4] + 4*Ad[1][3] + 4*Ad[3][1] + 4*Ad[4][0] + 4*Ad[3][3] + 4*Ad[0][8] + 4*Ad[8][0] + 4*Ad[4][8] + 4*Ad[8][4] + 8*Ad[8][8]; 
   Cd[3][17] = 2*Ad[1][2] + 2*Ad[2][1] + 2*Ad[2][3] + 2*Ad[3][2] + 2*Ad[1][6] + 2*Ad[6][1] + 2*Ad[3][6] - 2*Ad[4][5] - 2*Ad[5][4] + 2*Ad[6][3] - 2*Ad[4][7] - 2*Ad[7][4] - 2*Ad[5][8] - 2*Ad[8][5] - 2*Ad[7][8] - 2*Ad[8][7]; 
   Cd[3][19] = - 6*Ad[0][1] - 6*Ad[1][0] - 6*Ad[0][3] - 6*Ad[3][0] - 6*Ad[1][8] - 6*Ad[8][1] - 6*Ad[3][8] - 6*Ad[8][3]; 
   Cd[3][20] = 4*Ad[1][5] - 4*Ad[2][0] - 4*Ad[0][6] - 4*Ad[0][2] + 4*Ad[5][1] - 4*Ad[6][0] + 4*Ad[1][7] + 4*Ad[3][5] + 4*Ad[5][3] + 4*Ad[7][1] - 4*Ad[2][8] + 4*Ad[3][7] + 4*Ad[7][3] - 4*Ad[8][2] - 4*Ad[6][8] - 4*Ad[8][6]; 
   Cd[3][22] = 2*Ad[2][5] - 2*Ad[1][0] - 2*Ad[0][3] - 2*Ad[3][0] - 2*Ad[1][4] - 2*Ad[4][1] - 2*Ad[0][1] - 2*Ad[3][4] - 2*Ad[4][3] + 2*Ad[5][2] + 2*Ad[2][7] + 2*Ad[7][2] + 2*Ad[5][6] + 2*Ad[6][5] + 2*Ad[6][7] + 2*Ad[7][6]; 
   Cd[3][24] = Ad[0][1] + Ad[1][0] + Ad[0][3] + Ad[3][0] + Ad[1][4] + Ad[4][1] + Ad[3][4] + Ad[4][3] + Ad[1][8] + Ad[8][1] + Ad[3][8] + Ad[8][3] + bd[1][0] + bd[3][0]; 
   Cd[3][25] = 8*Ad[0][0] + 8*Ad[0][8] + 8*Ad[8][0] + 8*Ad[8][8]; 
   Cd[3][26] = - 6*Ad[0][5] - 6*Ad[5][0] - 6*Ad[0][7] - 6*Ad[7][0] - 6*Ad[5][8] - 6*Ad[8][5] - 6*Ad[7][8] - 6*Ad[8][7]; 
   Cd[3][28] = 8*Ad[0][0] + 4*Ad[0][4] + 4*Ad[4][0] + 4*Ad[0][8] + 4*Ad[8][0] + 4*Ad[5][5] + 4*Ad[4][8] + 4*Ad[5][7] + 4*Ad[7][5] + 4*Ad[8][4] + 4*Ad[7][7]; 
   Cd[3][30] = - 4*Ad[0][0] - 2*Ad[0][4] - 2*Ad[4][0] - 4*Ad[0][8] - 4*Ad[8][0] - 2*Ad[4][8] - 2*Ad[8][4] - 4*Ad[8][8] - 2*bd[8][0] - 2*bd[0][0]; 
   Cd[3][31] = - 2*Ad[0][5] - 2*Ad[5][0] - 2*Ad[0][7] - 2*Ad[7][0] - 2*Ad[4][5] - 2*Ad[5][4] - 2*Ad[4][7] - 2*Ad[7][4]; 
   Cd[3][33] = Ad[0][5] + Ad[5][0] + Ad[0][7] + Ad[7][0] + Ad[4][5] + Ad[5][4] + Ad[4][7] + Ad[7][4] + Ad[5][8] + Ad[8][5] + Ad[7][8] + Ad[8][7] + bd[5][0] + bd[7][0]; 
   
   Cd[4][1] = 2*Ad[1][5] + 2*Ad[5][1] - 2*Ad[1][7] - 2*Ad[3][5] - 2*Ad[5][3] - 2*Ad[7][1] + 2*Ad[3][7] + 2*Ad[7][3]; 
   Cd[4][2] = 2*Ad[2][3] - 2*Ad[2][1] - 2*Ad[1][2] + 2*Ad[3][2] + 2*Ad[1][6] + 2*Ad[6][1] - 2*Ad[3][6] - 2*Ad[6][3]; 
   Cd[4][3] = 4*Ad[1][1] - 4*Ad[1][3] - 4*Ad[3][1] + 4*Ad[3][3]; 
   Cd[4][5] = 2*Ad[1][4] + 2*Ad[4][1] - 2*Ad[2][5] - 2*Ad[3][4] - 2*Ad[4][3] - 2*Ad[5][2] + 2*Ad[1][8] + 2*Ad[2][7] + 2*Ad[7][2] + 2*Ad[8][1] - 2*Ad[3][8] - 2*Ad[5][6] - 2*Ad[6][5] - 2*Ad[8][3] + 2*Ad[6][7] + 2*Ad[7][6]; 
   Cd[4][6] = 4*Ad[2][2] - 4*Ad[1][1] + 4*Ad[3][3] - 4*Ad[5][5] - 4*Ad[6][6] + 4*Ad[7][7]; 
   Cd[4][7] = 4*Ad[0][5] - 4*Ad[2][1] - 4*Ad[1][2] + 4*Ad[2][3] + 4*Ad[3][2] + 4*Ad[5][0] - 4*Ad[0][7] - 4*Ad[1][6] - 4*Ad[6][1] - 4*Ad[7][0] + 4*Ad[3][6] + 4*Ad[4][5] + 4*Ad[5][4] + 4*Ad[6][3] - 4*Ad[4][7] - 4*Ad[7][4]; 
   Cd[4][9] = 2*Ad[0][1] + 2*Ad[1][0] - 2*Ad[0][3] - 2*Ad[3][0] + 2*Ad[2][5] + 2*Ad[5][2] + 2*Ad[1][8] + 2*Ad[2][7] + 2*Ad[7][2] + 2*Ad[8][1] - 2*Ad[3][8] - 2*Ad[5][6] - 2*Ad[6][5] - 2*Ad[8][3] - 2*Ad[6][7] - 2*Ad[7][6]; 
   Cd[4][10] = 4*Ad[0][6] - 4*Ad[2][0] - 4*Ad[0][2] - 4*Ad[1][5] - 4*Ad[2][4] - 4*Ad[4][2] - 4*Ad[5][1] + 4*Ad[6][0] - 4*Ad[1][7] + 4*Ad[3][5] + 4*Ad[5][3] - 4*Ad[7][1] + 4*Ad[3][7] + 4*Ad[4][6] + 4*Ad[6][4] + 4*Ad[7][3]; 
   Cd[4][12] = 6*Ad[0][1] + 6*Ad[1][0] - 6*Ad[0][3] - 6*Ad[3][0] + 6*Ad[1][4] + 6*Ad[4][1] - 6*Ad[3][4] - 6*Ad[4][3]; 
   Cd[4][14] = Ad[0][3] - Ad[1][0] - Ad[0][1] + Ad[3][0] - Ad[1][4] - Ad[4][1] + Ad[3][4] + Ad[4][3] - Ad[1][8] - Ad[8][1] + Ad[3][8] + Ad[8][3] - bd[1][0] + bd[3][0]; 
   Cd[4][15] = - 2*Ad[2][4] - 2*Ad[4][2] - 2*Ad[2][8] - 2*Ad[4][6] - 2*Ad[6][4] - 2*Ad[8][2] - 2*Ad[6][8] - 2*Ad[8][6]; 
   Cd[4][16] = 2*Ad[1][2] + 2*Ad[2][1] + 2*Ad[2][3] + 2*Ad[3][2] + 2*Ad[1][6] + 2*Ad[6][1] + 2*Ad[3][6] - 2*Ad[4][5] - 2*Ad[5][4] + 2*Ad[6][3] - 2*Ad[4][7] - 2*Ad[7][4] - 2*Ad[5][8] - 2*Ad[8][5] - 2*Ad[7][8] - 2*Ad[8][7]; 
   Cd[4][17] = 4*Ad[0][4] + 4*Ad[2][2] + 4*Ad[4][0] + 4*Ad[0][8] + 4*Ad[2][6] + 8*Ad[4][4] + 4*Ad[6][2] + 4*Ad[8][0] + 4*Ad[4][8] + 4*Ad[6][6] + 4*Ad[8][4]; 
   Cd[4][19] = 2*Ad[1][5] - 2*Ad[2][0] - 2*Ad[0][6] - 2*Ad[0][2] + 2*Ad[5][1] - 2*Ad[6][0] + 2*Ad[1][7] + 2*Ad[3][5] + 2*Ad[5][3] + 2*Ad[7][1] - 2*Ad[2][8] + 2*Ad[3][7] + 2*Ad[7][3] - 2*Ad[8][2] - 2*Ad[6][8] - 2*Ad[8][6]; 
   Cd[4][20] = 4*Ad[2][5] - 4*Ad[1][0] - 4*Ad[0][3] - 4*Ad[3][0] - 4*Ad[1][4] - 4*Ad[4][1] - 4*Ad[0][1] - 4*Ad[3][4] - 4*Ad[4][3] + 4*Ad[5][2] + 4*Ad[2][7] + 4*Ad[7][2] + 4*Ad[5][6] + 4*Ad[6][5] + 4*Ad[6][7] + 4*Ad[7][6]; 
   Cd[4][22] = - 6*Ad[0][2] - 6*Ad[2][0] - 6*Ad[0][6] - 6*Ad[2][4] - 6*Ad[4][2] - 6*Ad[6][0] - 6*Ad[4][6] - 6*Ad[6][4]; 
   Cd[4][24] = Ad[0][2] + Ad[2][0] + Ad[0][6] + Ad[2][4] + Ad[4][2] + Ad[6][0] + Ad[2][8] + Ad[4][6] + Ad[6][4] + Ad[8][2] + Ad[6][8] + Ad[8][6] + bd[2][0] + bd[6][0]; 
   Cd[4][25] = - 2*Ad[0][5] - 2*Ad[5][0] - 2*Ad[0][7] - 2*Ad[7][0] - 2*Ad[5][8] - 2*Ad[8][5] - 2*Ad[7][8] - 2*Ad[8][7]; 
   Cd[4][26] = 8*Ad[0][0] + 4*Ad[0][4] + 4*Ad[4][0] + 4*Ad[0][8] + 4*Ad[8][0] + 4*Ad[5][5] + 4*Ad[4][8] + 4*Ad[5][7] + 4*Ad[7][5] + 4*Ad[8][4] + 4*Ad[7][7]; 
   Cd[4][28] = - 6*Ad[0][5] - 6*Ad[5][0] - 6*Ad[0][7] - 6*Ad[7][0] - 6*Ad[4][5] - 6*Ad[5][4] - 6*Ad[4][7] - 6*Ad[7][4]; 
   Cd[4][30] = Ad[0][5] + Ad[5][0] + Ad[0][7] + Ad[7][0] + Ad[4][5] + Ad[5][4] + Ad[4][7] + Ad[7][4] + Ad[5][8] + Ad[8][5] + Ad[7][8] + Ad[8][7] + bd[5][0] + bd[7][0];
   Cd[4][31] = 8*Ad[0][0] + 8*Ad[0][4] + 8*Ad[4][0] + 8*Ad[4][4]; 
   Cd[4][33] = - 4*Ad[0][0] - 4*Ad[0][4] - 4*Ad[4][0] - 2*Ad[0][8] - 4*Ad[4][4] - 2*Ad[8][0] - 2*Ad[4][8] - 2*Ad[8][4] - 2*bd[4][0] - 2*bd[0][0]; 

function_terminate:

   return error;
}

Rox_ErrorCode rox_solve_quaternion_equations_system_macaulay_resultant_numeric_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double C)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double u = NULL;
   Rox_Array2D_Double M = NULL;

   if (!Re || !C ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&u, 1, 5);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&M, 495, 495);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_macaulay_quaternion_points3d_to_planes3d(M, u, C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_extract_real_quaternion_solutions_macaulay_points3d_to_planes3d(Re, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&M);
   rox_array2d_double_del(&u);

   return error;
}


Rox_ErrorCode rox_matrix_macaulay_quaternion_points3d_to_planes3d(Rox_Array2D_Double M, const Rox_Array2D_Double u, const Rox_Array2D_Double C)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!M || !C || !u ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double ** ud = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &ud, u );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Cd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Cd, C );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Md = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Md, M );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillzero(M);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Temporarily fixed here, should be entered by user
   ud[0][0] = 10.0; ud[0][1] = 0.1; ud[0][2] = 0.1; ud[0][3] = 0.1; ud[0][4] = 0.1;

   Md[0][0] = ud[0][0]; Md[0][1] = ud[0][1]; Md[0][2] = ud[0][2]; Md[0][3] = ud[0][3]; Md[0][4] = ud[0][4]; 
   Md[1][1] = ud[0][0]; Md[1][5] = ud[0][2]; Md[1][6] = ud[0][3]; Md[1][7] = ud[0][4]; Md[1][54] = ud[0][1]; 
   Md[2][2] = ud[0][0]; Md[2][5] = ud[0][1]; Md[2][8] = ud[0][2]; Md[2][9] = ud[0][3]; Md[2][10] = ud[0][4]; 
   Md[3][3] = ud[0][0]; Md[3][6] = ud[0][1]; Md[3][9] = ud[0][2]; Md[3][11] = ud[0][3]; Md[3][12] = ud[0][4]; 
   Md[4][4] = ud[0][0]; Md[4][7] = ud[0][1]; Md[4][10] = ud[0][2]; Md[4][12] = ud[0][3]; Md[4][13] = ud[0][4]; 
   Md[5][5] = ud[0][0]; Md[5][14] = ud[0][2]; Md[5][15] = ud[0][3]; Md[5][16] = ud[0][4]; Md[5][56] = ud[0][1]; 
   Md[6][6] = ud[0][0]; Md[6][15] = ud[0][2]; Md[6][17] = ud[0][3]; Md[6][18] = ud[0][4]; Md[6][57] = ud[0][1]; 
   Md[7][7] = ud[0][0]; Md[7][16] = ud[0][2]; Md[7][18] = ud[0][3]; Md[7][19] = ud[0][4]; Md[7][58] = ud[0][1]; 
   Md[8][8] = ud[0][0]; Md[8][14] = ud[0][1]; Md[8][20] = ud[0][3]; Md[8][21] = ud[0][4]; Md[8][162] = ud[0][2]; 
   Md[9][9] = ud[0][0]; Md[9][15] = ud[0][1]; Md[9][20] = ud[0][2]; Md[9][22] = ud[0][3]; Md[9][23] = ud[0][4]; 
   Md[10][10] = ud[0][0]; Md[10][16] = ud[0][1]; Md[10][21] = ud[0][2]; Md[10][23] = ud[0][3]; Md[10][24] = ud[0][4]; 
   Md[11][11] = ud[0][0]; Md[11][17] = ud[0][1]; Md[11][22] = ud[0][2]; Md[11][25] = ud[0][4]; Md[11][258] = ud[0][3]; 
   Md[12][12] = ud[0][0]; Md[12][18] = ud[0][1]; Md[12][23] = ud[0][2]; Md[12][25] = ud[0][3]; Md[12][26] = ud[0][4]; 
   Md[13][13] = ud[0][0]; Md[13][19] = ud[0][1]; Md[13][24] = ud[0][2]; Md[13][26] = ud[0][3]; Md[13][369] = ud[0][4]; 
   Md[14][14] = ud[0][0]; Md[14][27] = ud[0][3]; Md[14][28] = ud[0][4]; Md[14][63] = ud[0][1]; Md[14][163] = ud[0][2]; 
   Md[15][15] = ud[0][0]; Md[15][27] = ud[0][2]; Md[15][29] = ud[0][3]; Md[15][30] = ud[0][4]; Md[15][64] = ud[0][1]; 
   Md[16][16] = ud[0][0]; Md[16][28] = ud[0][2]; Md[16][30] = ud[0][3]; Md[16][31] = ud[0][4]; Md[16][65] = ud[0][1]; 
   Md[17][17] = ud[0][0]; Md[17][29] = ud[0][2]; Md[17][32] = ud[0][4]; Md[17][66] = ud[0][1]; Md[17][259] = ud[0][3]; 
   Md[18][18] = ud[0][0]; Md[18][30] = ud[0][2]; Md[18][32] = ud[0][3]; Md[18][33] = ud[0][4]; Md[18][67] = ud[0][1]; 
   Md[19][19] = ud[0][0]; Md[19][31] = ud[0][2]; Md[19][33] = ud[0][3]; Md[19][68] = ud[0][1]; Md[19][370] = ud[0][4]; 
   Md[20][20] = ud[0][0]; Md[20][27] = ud[0][1]; Md[20][34] = ud[0][3]; Md[20][35] = ud[0][4]; Md[20][165] = ud[0][2]; 
   Md[21][21] = ud[0][0]; Md[21][28] = ud[0][1]; Md[21][35] = ud[0][3]; Md[21][36] = ud[0][4]; Md[21][166] = ud[0][2]; 
   Md[22][22] = ud[0][0]; Md[22][29] = ud[0][1]; Md[22][34] = ud[0][2]; Md[22][37] = ud[0][4]; Md[22][260] = ud[0][3]; 
   Md[23][23] = ud[0][0]; Md[23][30] = ud[0][1]; Md[23][35] = ud[0][2]; Md[23][37] = ud[0][3]; Md[23][38] = ud[0][4]; 
   Md[24][24] = ud[0][0]; Md[24][31] = ud[0][1]; Md[24][36] = ud[0][2]; Md[24][38] = ud[0][3]; Md[24][371] = ud[0][4]; 
   Md[25][25] = ud[0][0]; Md[25][32] = ud[0][1]; Md[25][37] = ud[0][2]; Md[25][39] = ud[0][4]; Md[25][262] = ud[0][3]; 
   Md[26][26] = ud[0][0]; Md[26][33] = ud[0][1]; Md[26][38] = ud[0][2]; Md[26][39] = ud[0][3]; Md[26][372] = ud[0][4]; 
   Md[27][27] = ud[0][0]; Md[27][40] = ud[0][3]; Md[27][41] = ud[0][4]; Md[27][79] = ud[0][1]; Md[27][169] = ud[0][2]; 
   Md[28][28] = ud[0][0]; Md[28][41] = ud[0][3]; Md[28][42] = ud[0][4]; Md[28][80] = ud[0][1]; Md[28][170] = ud[0][2]; 
   Md[29][29] = ud[0][0]; Md[29][40] = ud[0][2]; Md[29][43] = ud[0][4]; Md[29][81] = ud[0][1]; Md[29][264] = ud[0][3]; 
   Md[30][30] = ud[0][0]; Md[30][41] = ud[0][2]; Md[30][43] = ud[0][3]; Md[30][44] = ud[0][4]; Md[30][82] = ud[0][1]; 
   Md[31][31] = ud[0][0]; Md[31][42] = ud[0][2]; Md[31][44] = ud[0][3]; Md[31][83] = ud[0][1]; Md[31][375] = ud[0][4]; 
   Md[32][32] = ud[0][0]; Md[32][43] = ud[0][2]; Md[32][45] = ud[0][4]; Md[32][84] = ud[0][1]; Md[32][266] = ud[0][3]; 
   Md[33][33] = ud[0][0]; Md[33][44] = ud[0][2]; Md[33][45] = ud[0][3]; Md[33][85] = ud[0][1]; Md[33][376] = ud[0][4]; 
   Md[34][34] = ud[0][0]; Md[34][40] = ud[0][1]; Md[34][46] = ud[0][4]; Md[34][174] = ud[0][2]; Md[34][267] = ud[0][3]; 
   Md[35][35] = ud[0][0]; Md[35][41] = ud[0][1]; Md[35][46] = ud[0][3]; Md[35][47] = ud[0][4]; Md[35][175] = ud[0][2]; 
   Md[36][36] = ud[0][0]; Md[36][42] = ud[0][1]; Md[36][47] = ud[0][3]; Md[36][176] = ud[0][2]; Md[36][378] = ud[0][4]; 
   Md[37][37] = ud[0][0]; Md[37][43] = ud[0][1]; Md[37][46] = ud[0][2]; Md[37][48] = ud[0][4]; Md[37][269] = ud[0][3]; 
   Md[38][38] = ud[0][0]; Md[38][44] = ud[0][1]; Md[38][47] = ud[0][2]; Md[38][48] = ud[0][3]; Md[38][379] = ud[0][4]; 
   Md[39][39] = ud[0][0]; Md[39][45] = ud[0][1]; Md[39][48] = ud[0][2]; Md[39][272] = ud[0][3]; Md[39][381] = ud[0][4]; 
   Md[40][40] = ud[0][0]; Md[40][49] = ud[0][4]; Md[40][103] = ud[0][1]; Md[40][184] = ud[0][2]; Md[40][277] = ud[0][3]; 
   Md[41][41] = ud[0][0]; Md[41][49] = ud[0][3]; Md[41][50] = ud[0][4]; Md[41][104] = ud[0][1]; Md[41][185] = ud[0][2]; 
   Md[42][42] = ud[0][0]; Md[42][50] = ud[0][3]; Md[42][105] = ud[0][1]; Md[42][186] = ud[0][2]; Md[42][388] = ud[0][4]; 
   Md[43][43] = ud[0][0]; Md[43][49] = ud[0][2]; Md[43][51] = ud[0][4]; Md[43][106] = ud[0][1]; Md[43][279] = ud[0][3]; 
   Md[44][44] = ud[0][0]; Md[44][50] = ud[0][2]; Md[44][51] = ud[0][3]; Md[44][107] = ud[0][1]; Md[44][389] = ud[0][4]; 
   Md[45][45] = ud[0][0]; Md[45][51] = ud[0][2]; Md[45][108] = ud[0][1]; Md[45][282] = ud[0][3]; Md[45][391] = ud[0][4]; 
   Md[46][46] = ud[0][0]; Md[46][49] = ud[0][1]; Md[46][52] = ud[0][4]; Md[46][193] = ud[0][2]; Md[46][285] = ud[0][3]; 
   Md[47][47] = ud[0][0]; Md[47][50] = ud[0][1]; Md[47][52] = ud[0][3]; Md[47][194] = ud[0][2]; Md[47][395] = ud[0][4]; 
   Md[48][48] = ud[0][0]; Md[48][51] = ud[0][1]; Md[48][52] = ud[0][2]; Md[48][288] = ud[0][3]; Md[48][397] = ud[0][4]; 
   Md[49][49] = ud[0][0]; Md[49][53] = ud[0][4]; Md[49][132] = ud[0][1]; Md[49][211] = ud[0][2]; Md[49][304] = ud[0][3]; 
   Md[50][50] = ud[0][0]; Md[50][53] = ud[0][3]; Md[50][133] = ud[0][1]; Md[50][212] = ud[0][2]; Md[50][415] = ud[0][4]; 
   Md[51][51] = ud[0][0]; Md[51][53] = ud[0][2]; Md[51][134] = ud[0][1]; Md[51][307] = ud[0][3]; Md[51][417] = ud[0][4]; 
   Md[52][52] = ud[0][0]; Md[52][53] = ud[0][1]; Md[52][221] = ud[0][2]; Md[52][316] = ud[0][3]; Md[52][427] = ud[0][4]; 
   Md[53][53] = ud[0][0]; Md[53][161] = ud[0][1]; Md[53][248] = ud[0][2]; Md[53][347] = ud[0][3]; Md[53][462] = ud[0][4]; 
   Md[54][0] = -1; Md[54][8] = 1; Md[54][11] = 1; Md[54][13] = 1; Md[54][54] = 1; 
   Md[55][1] = -1; Md[55][14] = 1; Md[55][17] = 1; Md[55][19] = 1; Md[55][55] = 1; 
   Md[56][2] = -1; Md[56][22] = 1; Md[56][24] = 1; Md[56][56] = 1; Md[56][162] = 1; 
   Md[57][3] = -1; Md[57][20] = 1; Md[57][26] = 1; Md[57][57] = 1; Md[57][258] = 1; 
   Md[58][4] = -1; Md[58][21] = 1; Md[58][25] = 1; Md[58][58] = 1; Md[58][369] = 1; 
   Md[59][54] = -1; Md[59][59] = 1; Md[59][63] = 1; Md[59][66] = 1; Md[59][68] = 1; 
   Md[60][5] = -1; Md[60][29] = 1; Md[60][31] = 1; Md[60][60] = 1; Md[60][163] = 1; 
   Md[61][6] = -1; Md[61][27] = 1; Md[61][33] = 1; Md[61][61] = 1; Md[61][259] = 1; 
   Md[62][7] = -1; Md[62][28] = 1; Md[62][32] = 1; Md[62][62] = 1; Md[62][370] = 1; 
   Md[63][8] = -1; Md[63][34] = 1; Md[63][36] = 1; Md[63][63] = 1; Md[63][164] = 1; 
   Md[64][9] = -1; Md[64][38] = 1; Md[64][64] = 1; Md[64][165] = 1; Md[64][260] = 1; 
   Md[65][10] = -1; Md[65][37] = 1; Md[65][65] = 1; Md[65][166] = 1; Md[65][371] = 1; 
   Md[66][11] = -1; Md[66][34] = 1; Md[66][39] = 1; Md[66][66] = 1; Md[66][261] = 1; 
   Md[67][12] = -1; Md[67][35] = 1; Md[67][67] = 1; Md[67][262] = 1; Md[67][372] = 1; 
   Md[68][13] = -1; Md[68][36] = 1; Md[68][39] = 1; Md[68][68] = 1; Md[68][373] = 1; 
   Md[69][55] = -1; Md[69][69] = 1; Md[69][73] = 1; Md[69][76] = 1; Md[69][78] = 1; 
   Md[70][56] = -1; Md[70][70] = 1; Md[70][81] = 1; Md[70][83] = 1; Md[70][167] = 1; 
   Md[71][57] = -1; Md[71][71] = 1; Md[71][79] = 1; Md[71][85] = 1; Md[71][263] = 1; 
   Md[72][58] = -1; Md[72][72] = 1; Md[72][80] = 1; Md[72][84] = 1; Md[72][374] = 1; 
   Md[73][14] = -1; Md[73][40] = 1; Md[73][42] = 1; Md[73][73] = 1; Md[73][168] = 1; 
   Md[74][15] = -1; Md[74][44] = 1; Md[74][74] = 1; Md[74][169] = 1; Md[74][264] = 1; 
   Md[75][16] = -1; Md[75][43] = 1; Md[75][75] = 1; Md[75][170] = 1; Md[75][375] = 1; 
   Md[76][17] = -1; Md[76][40] = 1; Md[76][45] = 1; Md[76][76] = 1; Md[76][265] = 1; 
   Md[77][18] = -1; Md[77][41] = 1; Md[77][77] = 1; Md[77][266] = 1; Md[77][376] = 1; 
   Md[78][19] = -1; Md[78][42] = 1; Md[78][45] = 1; Md[78][78] = 1; Md[78][377] = 1; 
   Md[79][20] = -1; Md[79][47] = 1; Md[79][79] = 1; Md[79][172] = 1; Md[79][267] = 1; 
   Md[80][21] = -1; Md[80][46] = 1; Md[80][80] = 1; Md[80][173] = 1; Md[80][378] = 1; 
   Md[81][22] = -1; Md[81][48] = 1; Md[81][81] = 1; Md[81][174] = 1; Md[81][268] = 1; 
   Md[82][23] = -1; Md[82][82] = 1; Md[82][175] = 1; Md[82][269] = 1; Md[82][379] = 1; 
   Md[83][24] = -1; Md[83][48] = 1; Md[83][83] = 1; Md[83][176] = 1; Md[83][380] = 1; 
   Md[84][25] = -1; Md[84][46] = 1; Md[84][84] = 1; Md[84][271] = 1; Md[84][381] = 1; 
   Md[85][26] = -1; Md[85][47] = 1; Md[85][85] = 1; Md[85][272] = 1; Md[85][382] = 1; 
   Md[86][59] = -1; Md[86][86] = 1; Md[86][90] = 1; Md[86][93] = 1; Md[86][95] = 1; 
   Md[87][60] = -1; Md[87][87] = 1; Md[87][98] = 1; Md[87][100] = 1; Md[87][177] = 1; 
   Md[88][61] = -1; Md[88][88] = 1; Md[88][96] = 1; Md[88][102] = 1; Md[88][273] = 1; 
   Md[89][62] = -1; Md[89][89] = 1; Md[89][97] = 1; Md[89][101] = 1; Md[89][384] = 1; 
   Md[90][63] = -1; Md[90][90] = 1; Md[90][103] = 1; Md[90][105] = 1; Md[90][178] = 1; 
   Md[91][64] = -1; Md[91][91] = 1; Md[91][107] = 1; Md[91][179] = 1; Md[91][274] = 1; 
   Md[92][65] = -1; Md[92][92] = 1; Md[92][106] = 1; Md[92][180] = 1; Md[92][385] = 1; 
   Md[93][66] = -1; Md[93][93] = 1; Md[93][103] = 1; Md[93][108] = 1; Md[93][275] = 1; 
   Md[94][67] = -1; Md[94][94] = 1; Md[94][104] = 1; Md[94][276] = 1; Md[94][386] = 1; 
   Md[95][68] = -1; Md[95][95] = 1; Md[95][105] = 1; Md[95][108] = 1; Md[95][387] = 1; 
   Md[96][27] = -1; Md[96][50] = 1; Md[96][96] = 1; Md[96][182] = 1; Md[96][277] = 1; 
   Md[97][28] = -1; Md[97][49] = 1; Md[97][97] = 1; Md[97][183] = 1; Md[97][388] = 1; 
   Md[98][29] = -1; Md[98][51] = 1; Md[98][98] = 1; Md[98][184] = 1; Md[98][278] = 1; 
   Md[99][30] = -1; Md[99][99] = 1; Md[99][185] = 1; Md[99][279] = 1; Md[99][389] = 1; 
   Md[100][31] = -1; Md[100][51] = 1; Md[100][100] = 1; Md[100][186] = 1; Md[100][390] = 1; 
   Md[101][32] = -1; Md[101][49] = 1; Md[101][101] = 1; Md[101][281] = 1; Md[101][391] = 1; 
   Md[102][33] = -1; Md[102][50] = 1; Md[102][102] = 1; Md[102][282] = 1; Md[102][392] = 1; 
   Md[103][34] = -1; Md[103][52] = 1; Md[103][103] = 1; Md[103][190] = 1; Md[103][284] = 1; 
   Md[104][35] = -1; Md[104][104] = 1; Md[104][191] = 1; Md[104][285] = 1; Md[104][395] = 1; 
   Md[105][36] = -1; Md[105][52] = 1; Md[105][105] = 1; Md[105][192] = 1; Md[105][396] = 1; 
   Md[106][37] = -1; Md[106][106] = 1; Md[106][193] = 1; Md[106][287] = 1; Md[106][397] = 1; 
   Md[107][38] = -1; Md[107][107] = 1; Md[107][194] = 1; Md[107][288] = 1; Md[107][398] = 1; 
   Md[108][39] = -1; Md[108][52] = 1; Md[108][108] = 1; Md[108][291] = 1; Md[108][401] = 1; 
   Md[109][69] = -1; Md[109][109] = 1; Md[109][113] = 1; Md[109][116] = 1; Md[109][118] = 1; 
   Md[110][70] = -1; Md[110][110] = 1; Md[110][121] = 1; Md[110][123] = 1; Md[110][195] = 1; 
   Md[111][71] = -1; Md[111][111] = 1; Md[111][119] = 1; Md[111][125] = 1; Md[111][292] = 1; 
   Md[112][72] = -1; Md[112][112] = 1; Md[112][120] = 1; Md[112][124] = 1; Md[112][404] = 1; 
   Md[113][73] = -1; Md[113][113] = 1; Md[113][126] = 1; Md[113][128] = 1; Md[113][196] = 1; 
   Md[114][74] = -1; Md[114][114] = 1; Md[114][130] = 1; Md[114][197] = 1; Md[114][293] = 1; 
   Md[115][75] = -1; Md[115][115] = 1; Md[115][129] = 1; Md[115][198] = 1; Md[115][405] = 1; 
   Md[116][76] = -1; Md[116][116] = 1; Md[116][126] = 1; Md[116][131] = 1; Md[116][294] = 1; 
   Md[117][77] = -1; Md[117][117] = 1; Md[117][127] = 1; Md[117][295] = 1; Md[117][406] = 1; 
   Md[118][78] = -1; Md[118][118] = 1; Md[118][128] = 1; Md[118][131] = 1; Md[118][407] = 1; 
   Md[119][79] = -1; Md[119][119] = 1; Md[119][133] = 1; Md[119][200] = 1; Md[119][296] = 1; 
   Md[120][80] = -1; Md[120][120] = 1; Md[120][132] = 1; Md[120][201] = 1; Md[120][408] = 1; 
   Md[121][81] = -1; Md[121][121] = 1; Md[121][134] = 1; Md[121][202] = 1; Md[121][297] = 1; 
   Md[122][82] = -1; Md[122][122] = 1; Md[122][203] = 1; Md[122][298] = 1; Md[122][409] = 1; 
   Md[123][83] = -1; Md[123][123] = 1; Md[123][134] = 1; Md[123][204] = 1; Md[123][410] = 1; 
   Md[124][84] = -1; Md[124][124] = 1; Md[124][132] = 1; Md[124][300] = 1; Md[124][411] = 1; 
   Md[125][85] = -1; Md[125][125] = 1; Md[125][133] = 1; Md[125][301] = 1; Md[125][412] = 1; 
   Md[126][40] = -1; Md[126][53] = 1; Md[126][126] = 1; Md[126][208] = 1; Md[126][303] = 1; 
   Md[127][41] = -1; Md[127][127] = 1; Md[127][209] = 1; Md[127][304] = 1; Md[127][415] = 1; 
   Md[128][42] = -1; Md[128][53] = 1; Md[128][128] = 1; Md[128][210] = 1; Md[128][416] = 1; 
   Md[129][43] = -1; Md[129][129] = 1; Md[129][211] = 1; Md[129][306] = 1; Md[129][417] = 1; 
   Md[130][44] = -1; Md[130][130] = 1; Md[130][212] = 1; Md[130][307] = 1; Md[130][418] = 1; 
   Md[131][45] = -1; Md[131][53] = 1; Md[131][131] = 1; Md[131][310] = 1; Md[131][421] = 1; 
   Md[132][46] = -1; Md[132][132] = 1; Md[132][219] = 1; Md[132][315] = 1; Md[132][427] = 1; 
   Md[133][47] = -1; Md[133][133] = 1; Md[133][220] = 1; Md[133][316] = 1; Md[133][428] = 1; 
   Md[134][48] = -1; Md[134][134] = 1; Md[134][221] = 1; Md[134][319] = 1; Md[134][431] = 1; 
   Md[135][86] = -1; Md[135][135] = 1; Md[135][139] = 1; Md[135][142] = 1; Md[135][144] = 1; 
   Md[136][87] = -1; Md[136][136] = 1; Md[136][147] = 1; Md[136][149] = 1; Md[136][222] = 1; 
   Md[137][88] = -1; Md[137][137] = 1; Md[137][145] = 1; Md[137][151] = 1; Md[137][323] = 1; 
   Md[138][89] = -1; Md[138][138] = 1; Md[138][146] = 1; Md[138][150] = 1; Md[138][439] = 1; 
   Md[139][90] = -1; Md[139][139] = 1; Md[139][152] = 1; Md[139][154] = 1; Md[139][223] = 1; 
   Md[140][91] = -1; Md[140][140] = 1; Md[140][156] = 1; Md[140][224] = 1; Md[140][324] = 1; 
   Md[141][92] = -1; Md[141][141] = 1; Md[141][155] = 1; Md[141][225] = 1; Md[141][440] = 1; 
   Md[142][93] = -1; Md[142][142] = 1; Md[142][152] = 1; Md[142][157] = 1; Md[142][325] = 1; 
   Md[143][94] = -1; Md[143][143] = 1; Md[143][153] = 1; Md[143][326] = 1; Md[143][441] = 1; 
   Md[144][95] = -1; Md[144][144] = 1; Md[144][154] = 1; Md[144][157] = 1; Md[144][442] = 1; 
   Md[145][96] = -1; Md[145][145] = 1; Md[145][159] = 1; Md[145][227] = 1; Md[145][327] = 1; 
   Md[146][97] = -1; Md[146][146] = 1; Md[146][158] = 1; Md[146][228] = 1; Md[146][443] = 1; 
   Md[147][98] = -1; Md[147][147] = 1; Md[147][160] = 1; Md[147][229] = 1; Md[147][328] = 1; 
   Md[148][99] = -1; Md[148][148] = 1; Md[148][230] = 1; Md[148][329] = 1; Md[148][444] = 1; 
   Md[149][100] = -1; Md[149][149] = 1; Md[149][160] = 1; Md[149][231] = 1; Md[149][445] = 1; 
   Md[150][101] = -1; Md[150][150] = 1; Md[150][158] = 1; Md[150][331] = 1; Md[150][446] = 1; 
   Md[151][102] = -1; Md[151][151] = 1; Md[151][159] = 1; Md[151][332] = 1; Md[151][447] = 1; 
   Md[152][103] = -1; Md[152][152] = 1; Md[152][161] = 1; Md[152][235] = 1; Md[152][334] = 1; 
   Md[153][104] = -1; Md[153][153] = 1; Md[153][236] = 1; Md[153][335] = 1; Md[153][450] = 1; 
   Md[154][105] = -1; Md[154][154] = 1; Md[154][161] = 1; Md[154][237] = 1; Md[154][451] = 1; 
   Md[155][106] = -1; Md[155][155] = 1; Md[155][238] = 1; Md[155][337] = 1; Md[155][452] = 1; 
   Md[156][107] = -1; Md[156][156] = 1; Md[156][239] = 1; Md[156][338] = 1; Md[156][453] = 1; 
   Md[157][108] = -1; Md[157][157] = 1; Md[157][161] = 1; Md[157][341] = 1; Md[157][456] = 1; 
   Md[158][49] = -1; Md[158][158] = 1; Md[158][246] = 1; Md[158][346] = 1; Md[158][462] = 1; 
   Md[159][50] = -1; Md[159][159] = 1; Md[159][247] = 1; Md[159][347] = 1; Md[159][463] = 1; 
   Md[160][51] = -1; Md[160][160] = 1; Md[160][248] = 1; Md[160][350] = 1; Md[160][466] = 1; 
   Md[161][52] = -1; Md[161][161] = 1; Md[161][257] = 1; Md[161][362] = 1; Md[161][481] = 1; 
   Md[162][2] = Cd[1][24]; Md[162][3] = Cd[1][30]; Md[162][4] = Cd[1][33]; Md[162][14] = Cd[1][5]; Md[162][15] = Cd[1][6]; Md[162][16] = Cd[1][7]; Md[162][17] = Cd[1][9]; Md[162][18] = Cd[1][10]; Md[162][19] = Cd[1][12]; Md[162][20] = Cd[1][16]; Md[162][21] = Cd[1][17]; Md[162][22] = Cd[1][19]; Md[162][23] = Cd[1][20]; Md[162][24] = Cd[1][22]; Md[162][25] = Cd[1][26]; Md[162][26] = Cd[1][28]; Md[162][162] = Cd[1][15]; Md[162][258] = Cd[1][25]; Md[162][369] = Cd[1][31]; 
   Md[163][5] = Cd[1][24]; Md[163][6] = Cd[1][30]; Md[163][7] = Cd[1][33]; Md[163][27] = Cd[1][16]; Md[163][28] = Cd[1][17]; Md[163][29] = Cd[1][19]; Md[163][30] = Cd[1][20]; Md[163][31] = Cd[1][22]; Md[163][32] = Cd[1][26]; Md[163][33] = Cd[1][28]; Md[163][63] = Cd[1][5]; Md[163][64] = Cd[1][6]; Md[163][65] = Cd[1][7]; Md[163][66] = Cd[1][9]; Md[163][67] = Cd[1][10]; Md[163][68] = Cd[1][12]; Md[163][163] = Cd[1][15]; Md[163][259] = Cd[1][25]; Md[163][370] = Cd[1][31]; 
   Md[164][8] = Cd[1][24]; Md[164][9] = Cd[1][30]; Md[164][10] = Cd[1][33]; Md[164][27] = Cd[1][6]; Md[164][28] = Cd[1][7]; Md[164][29] = Cd[1][9]; Md[164][30] = Cd[1][10]; Md[164][31] = Cd[1][12]; Md[164][34] = Cd[1][19]; Md[164][35] = Cd[1][20]; Md[164][36] = Cd[1][22]; Md[164][37] = Cd[1][26]; Md[164][38] = Cd[1][28]; Md[164][163] = Cd[1][5]; Md[164][164] = Cd[1][15]; Md[164][165] = Cd[1][16]; Md[164][166] = Cd[1][17]; Md[164][260] = Cd[1][25]; Md[164][371] = Cd[1][31]; 
   Md[165][9] = Cd[1][24]; Md[165][11] = Cd[1][30]; Md[165][12] = Cd[1][33]; Md[165][27] = Cd[1][5]; Md[165][29] = Cd[1][6]; Md[165][30] = Cd[1][7]; Md[165][32] = Cd[1][10]; Md[165][33] = Cd[1][12]; Md[165][34] = Cd[1][16]; Md[165][35] = Cd[1][17]; Md[165][37] = Cd[1][20]; Md[165][38] = Cd[1][22]; Md[165][39] = Cd[1][28]; Md[165][165] = Cd[1][15]; Md[165][259] = Cd[1][9]; Md[165][260] = Cd[1][19]; Md[165][261] = Cd[1][25]; Md[165][262] = Cd[1][26]; Md[165][372] = Cd[1][31]; 
   Md[166][10] = Cd[1][24]; Md[166][12] = Cd[1][30]; Md[166][13] = Cd[1][33]; Md[166][28] = Cd[1][5]; Md[166][30] = Cd[1][6]; Md[166][31] = Cd[1][7]; Md[166][32] = Cd[1][9]; Md[166][33] = Cd[1][10]; Md[166][35] = Cd[1][16]; Md[166][36] = Cd[1][17]; Md[166][37] = Cd[1][19]; Md[166][38] = Cd[1][20]; Md[166][39] = Cd[1][26]; Md[166][166] = Cd[1][15]; Md[166][262] = Cd[1][25]; Md[166][370] = Cd[1][12]; Md[166][371] = Cd[1][22]; Md[166][372] = Cd[1][28]; Md[166][373] = Cd[1][31]; 
   Md[167][56] = Cd[1][24]; Md[167][57] = Cd[1][30]; Md[167][58] = Cd[1][33]; Md[167][73] = Cd[1][5]; Md[167][74] = Cd[1][6]; Md[167][75] = Cd[1][7]; Md[167][76] = Cd[1][9]; Md[167][77] = Cd[1][10]; Md[167][78] = Cd[1][12]; Md[167][79] = Cd[1][16]; Md[167][80] = Cd[1][17]; Md[167][81] = Cd[1][19]; Md[167][82] = Cd[1][20]; Md[167][83] = Cd[1][22]; Md[167][84] = Cd[1][26]; Md[167][85] = Cd[1][28]; Md[167][167] = Cd[1][15]; Md[167][263] = Cd[1][25]; Md[167][374] = Cd[1][31]; 
   Md[168][14] = Cd[1][24]; Md[168][15] = Cd[1][30]; Md[168][16] = Cd[1][33]; Md[168][40] = Cd[1][19]; Md[168][41] = Cd[1][20]; Md[168][42] = Cd[1][22]; Md[168][43] = Cd[1][26]; Md[168][44] = Cd[1][28]; Md[168][79] = Cd[1][6]; Md[168][80] = Cd[1][7]; Md[168][81] = Cd[1][9]; Md[168][82] = Cd[1][10]; Md[168][83] = Cd[1][12]; Md[168][167] = Cd[1][5]; Md[168][168] = Cd[1][15]; Md[168][169] = Cd[1][16]; Md[168][170] = Cd[1][17]; Md[168][264] = Cd[1][25]; Md[168][375] = Cd[1][31]; 
   Md[169][15] = Cd[1][24]; Md[169][17] = Cd[1][30]; Md[169][18] = Cd[1][33]; Md[169][40] = Cd[1][16]; Md[169][41] = Cd[1][17]; Md[169][43] = Cd[1][20]; Md[169][44] = Cd[1][22]; Md[169][45] = Cd[1][28]; Md[169][79] = Cd[1][5]; Md[169][81] = Cd[1][6]; Md[169][82] = Cd[1][7]; Md[169][84] = Cd[1][10]; Md[169][85] = Cd[1][12]; Md[169][169] = Cd[1][15]; Md[169][263] = Cd[1][9]; Md[169][264] = Cd[1][19]; Md[169][265] = Cd[1][25]; Md[169][266] = Cd[1][26]; Md[169][376] = Cd[1][31]; 
   Md[170][16] = Cd[1][24]; Md[170][18] = Cd[1][30]; Md[170][19] = Cd[1][33]; Md[170][41] = Cd[1][16]; Md[170][42] = Cd[1][17]; Md[170][43] = Cd[1][19]; Md[170][44] = Cd[1][20]; Md[170][45] = Cd[1][26]; Md[170][80] = Cd[1][5]; Md[170][82] = Cd[1][6]; Md[170][83] = Cd[1][7]; Md[170][84] = Cd[1][9]; Md[170][85] = Cd[1][10]; Md[170][170] = Cd[1][15]; Md[170][266] = Cd[1][25]; Md[170][374] = Cd[1][12]; Md[170][375] = Cd[1][22]; Md[170][376] = Cd[1][28]; Md[170][377] = Cd[1][31]; 
   Md[171][20] = Cd[1][30]; Md[171][21] = Cd[1][33]; Md[171][40] = Cd[1][9]; Md[171][41] = Cd[1][10]; Md[171][42] = Cd[1][12]; Md[171][46] = Cd[1][26]; Md[171][47] = Cd[1][28]; Md[171][162] = Cd[1][24]; Md[171][168] = Cd[1][5]; Md[171][169] = Cd[1][6]; Md[171][170] = Cd[1][7]; Md[171][171] = Cd[1][15]; Md[171][172] = Cd[1][16]; Md[171][173] = Cd[1][17]; Md[171][174] = Cd[1][19]; Md[171][175] = Cd[1][20]; Md[171][176] = Cd[1][22]; Md[171][267] = Cd[1][25]; Md[171][378] = Cd[1][31]; 
   Md[172][20] = Cd[1][24]; Md[172][22] = Cd[1][30]; Md[172][23] = Cd[1][33]; Md[172][40] = Cd[1][6]; Md[172][41] = Cd[1][7]; Md[172][43] = Cd[1][10]; Md[172][44] = Cd[1][12]; Md[172][46] = Cd[1][20]; Md[172][47] = Cd[1][22]; Md[172][48] = Cd[1][28]; Md[172][169] = Cd[1][5]; Md[172][172] = Cd[1][15]; Md[172][174] = Cd[1][16]; Md[172][175] = Cd[1][17]; Md[172][264] = Cd[1][9]; Md[172][267] = Cd[1][19]; Md[172][268] = Cd[1][25]; Md[172][269] = Cd[1][26]; Md[172][379] = Cd[1][31]; 
   Md[173][21] = Cd[1][24]; Md[173][23] = Cd[1][30]; Md[173][24] = Cd[1][33]; Md[173][41] = Cd[1][6]; Md[173][42] = Cd[1][7]; Md[173][43] = Cd[1][9]; Md[173][44] = Cd[1][10]; Md[173][46] = Cd[1][19]; Md[173][47] = Cd[1][20]; Md[173][48] = Cd[1][26]; Md[173][170] = Cd[1][5]; Md[173][173] = Cd[1][15]; Md[173][175] = Cd[1][16]; Md[173][176] = Cd[1][17]; Md[173][269] = Cd[1][25]; Md[173][375] = Cd[1][12]; Md[173][378] = Cd[1][22]; Md[173][379] = Cd[1][28]; Md[173][380] = Cd[1][31]; 
   Md[174][22] = Cd[1][24]; Md[174][25] = Cd[1][33]; Md[174][40] = Cd[1][5]; Md[174][43] = Cd[1][7]; Md[174][45] = Cd[1][12]; Md[174][46] = Cd[1][17]; Md[174][48] = Cd[1][22]; Md[174][174] = Cd[1][15]; Md[174][258] = Cd[1][30]; Md[174][264] = Cd[1][6]; Md[174][265] = Cd[1][9]; Md[174][266] = Cd[1][10]; Md[174][267] = Cd[1][16]; Md[174][268] = Cd[1][19]; Md[174][269] = Cd[1][20]; Md[174][270] = Cd[1][25]; Md[174][271] = Cd[1][26]; Md[174][272] = Cd[1][28]; Md[174][381] = Cd[1][31]; 
   Md[175][23] = Cd[1][24]; Md[175][25] = Cd[1][30]; Md[175][26] = Cd[1][33]; Md[175][41] = Cd[1][5]; Md[175][43] = Cd[1][6]; Md[175][44] = Cd[1][7]; Md[175][45] = Cd[1][10]; Md[175][46] = Cd[1][16]; Md[175][47] = Cd[1][17]; Md[175][48] = Cd[1][20]; Md[175][175] = Cd[1][15]; Md[175][266] = Cd[1][9]; Md[175][269] = Cd[1][19]; Md[175][271] = Cd[1][25]; Md[175][272] = Cd[1][26]; Md[175][376] = Cd[1][12]; Md[175][379] = Cd[1][22]; Md[175][381] = Cd[1][28]; Md[175][382] = Cd[1][31]; 
   Md[176][24] = Cd[1][24]; Md[176][26] = Cd[1][30]; Md[176][42] = Cd[1][5]; Md[176][44] = Cd[1][6]; Md[176][45] = Cd[1][9]; Md[176][47] = Cd[1][16]; Md[176][48] = Cd[1][19]; Md[176][176] = Cd[1][15]; Md[176][272] = Cd[1][25]; Md[176][369] = Cd[1][33]; Md[176][375] = Cd[1][7]; Md[176][376] = Cd[1][10]; Md[176][377] = Cd[1][12]; Md[176][378] = Cd[1][17]; Md[176][379] = Cd[1][20]; Md[176][380] = Cd[1][22]; Md[176][381] = Cd[1][26]; Md[176][382] = Cd[1][28]; Md[176][383] = Cd[1][31]; 
   Md[177][60] = Cd[1][24]; Md[177][61] = Cd[1][30]; Md[177][62] = Cd[1][33]; Md[177][90] = Cd[1][5]; Md[177][91] = Cd[1][6]; Md[177][92] = Cd[1][7]; Md[177][93] = Cd[1][9]; Md[177][94] = Cd[1][10]; Md[177][95] = Cd[1][12]; Md[177][96] = Cd[1][16]; Md[177][97] = Cd[1][17]; Md[177][98] = Cd[1][19]; Md[177][99] = Cd[1][20]; Md[177][100] = Cd[1][22]; Md[177][101] = Cd[1][26]; Md[177][102] = Cd[1][28]; Md[177][177] = Cd[1][15]; Md[177][273] = Cd[1][25]; Md[177][384] = Cd[1][31]; 
   Md[178][63] = Cd[1][24]; Md[178][64] = Cd[1][30]; Md[178][65] = Cd[1][33]; Md[178][96] = Cd[1][6]; Md[178][97] = Cd[1][7]; Md[178][98] = Cd[1][9]; Md[178][99] = Cd[1][10]; Md[178][100] = Cd[1][12]; Md[178][103] = Cd[1][19]; Md[178][104] = Cd[1][20]; Md[178][105] = Cd[1][22]; Md[178][106] = Cd[1][26]; Md[178][107] = Cd[1][28]; Md[178][177] = Cd[1][5]; Md[178][178] = Cd[1][15]; Md[178][179] = Cd[1][16]; Md[178][180] = Cd[1][17]; Md[178][274] = Cd[1][25]; Md[178][385] = Cd[1][31]; 
   Md[179][64] = Cd[1][24]; Md[179][66] = Cd[1][30]; Md[179][67] = Cd[1][33]; Md[179][96] = Cd[1][5]; Md[179][98] = Cd[1][6]; Md[179][99] = Cd[1][7]; Md[179][101] = Cd[1][10]; Md[179][102] = Cd[1][12]; Md[179][103] = Cd[1][16]; Md[179][104] = Cd[1][17]; Md[179][106] = Cd[1][20]; Md[179][107] = Cd[1][22]; Md[179][108] = Cd[1][28]; Md[179][179] = Cd[1][15]; Md[179][273] = Cd[1][9]; Md[179][274] = Cd[1][19]; Md[179][275] = Cd[1][25]; Md[179][276] = Cd[1][26]; Md[179][386] = Cd[1][31]; 
   Md[180][65] = Cd[1][24]; Md[180][67] = Cd[1][30]; Md[180][68] = Cd[1][33]; Md[180][97] = Cd[1][5]; Md[180][99] = Cd[1][6]; Md[180][100] = Cd[1][7]; Md[180][101] = Cd[1][9]; Md[180][102] = Cd[1][10]; Md[180][104] = Cd[1][16]; Md[180][105] = Cd[1][17]; Md[180][106] = Cd[1][19]; Md[180][107] = Cd[1][20]; Md[180][108] = Cd[1][26]; Md[180][180] = Cd[1][15]; Md[180][276] = Cd[1][25]; Md[180][384] = Cd[1][12]; Md[180][385] = Cd[1][22]; Md[180][386] = Cd[1][28]; Md[180][387] = Cd[1][31]; 
   Md[181][27] = Cd[1][30]; Md[181][28] = Cd[1][33]; Md[181][49] = Cd[1][26]; Md[181][50] = Cd[1][28]; Md[181][103] = Cd[1][9]; Md[181][104] = Cd[1][10]; Md[181][105] = Cd[1][12]; Md[181][163] = Cd[1][24]; Md[181][178] = Cd[1][5]; Md[181][179] = Cd[1][6]; Md[181][180] = Cd[1][7]; Md[181][181] = Cd[1][15]; Md[181][182] = Cd[1][16]; Md[181][183] = Cd[1][17]; Md[181][184] = Cd[1][19]; Md[181][185] = Cd[1][20]; Md[181][186] = Cd[1][22]; Md[181][277] = Cd[1][25]; Md[181][388] = Cd[1][31]; 
   Md[182][27] = Cd[1][24]; Md[182][29] = Cd[1][30]; Md[182][30] = Cd[1][33]; Md[182][49] = Cd[1][20]; Md[182][50] = Cd[1][22]; Md[182][51] = Cd[1][28]; Md[182][103] = Cd[1][6]; Md[182][104] = Cd[1][7]; Md[182][106] = Cd[1][10]; Md[182][107] = Cd[1][12]; Md[182][179] = Cd[1][5]; Md[182][182] = Cd[1][15]; Md[182][184] = Cd[1][16]; Md[182][185] = Cd[1][17]; Md[182][274] = Cd[1][9]; Md[182][277] = Cd[1][19]; Md[182][278] = Cd[1][25]; Md[182][279] = Cd[1][26]; Md[182][389] = Cd[1][31]; 
   Md[183][28] = Cd[1][24]; Md[183][30] = Cd[1][30]; Md[183][31] = Cd[1][33]; Md[183][49] = Cd[1][19]; Md[183][50] = Cd[1][20]; Md[183][51] = Cd[1][26]; Md[183][104] = Cd[1][6]; Md[183][105] = Cd[1][7]; Md[183][106] = Cd[1][9]; Md[183][107] = Cd[1][10]; Md[183][180] = Cd[1][5]; Md[183][183] = Cd[1][15]; Md[183][185] = Cd[1][16]; Md[183][186] = Cd[1][17]; Md[183][279] = Cd[1][25]; Md[183][385] = Cd[1][12]; Md[183][388] = Cd[1][22]; Md[183][389] = Cd[1][28]; Md[183][390] = Cd[1][31]; 
   Md[184][29] = Cd[1][24]; Md[184][32] = Cd[1][33]; Md[184][49] = Cd[1][17]; Md[184][51] = Cd[1][22]; Md[184][103] = Cd[1][5]; Md[184][106] = Cd[1][7]; Md[184][108] = Cd[1][12]; Md[184][184] = Cd[1][15]; Md[184][259] = Cd[1][30]; Md[184][274] = Cd[1][6]; Md[184][275] = Cd[1][9]; Md[184][276] = Cd[1][10]; Md[184][277] = Cd[1][16]; Md[184][278] = Cd[1][19]; Md[184][279] = Cd[1][20]; Md[184][280] = Cd[1][25]; Md[184][281] = Cd[1][26]; Md[184][282] = Cd[1][28]; Md[184][391] = Cd[1][31]; 
   Md[185][30] = Cd[1][24]; Md[185][32] = Cd[1][30]; Md[185][33] = Cd[1][33]; Md[185][49] = Cd[1][16]; Md[185][50] = Cd[1][17]; Md[185][51] = Cd[1][20]; Md[185][104] = Cd[1][5]; Md[185][106] = Cd[1][6]; Md[185][107] = Cd[1][7]; Md[185][108] = Cd[1][10]; Md[185][185] = Cd[1][15]; Md[185][276] = Cd[1][9]; Md[185][279] = Cd[1][19]; Md[185][281] = Cd[1][25]; Md[185][282] = Cd[1][26]; Md[185][386] = Cd[1][12]; Md[185][389] = Cd[1][22]; Md[185][391] = Cd[1][28]; Md[185][392] = Cd[1][31]; 
   Md[186][31] = Cd[1][24]; Md[186][33] = Cd[1][30]; Md[186][50] = Cd[1][16]; Md[186][51] = Cd[1][19]; Md[186][105] = Cd[1][5]; Md[186][107] = Cd[1][6]; Md[186][108] = Cd[1][9]; Md[186][186] = Cd[1][15]; Md[186][282] = Cd[1][25]; Md[186][370] = Cd[1][33]; Md[186][385] = Cd[1][7]; Md[186][386] = Cd[1][10]; Md[186][387] = Cd[1][12]; Md[186][388] = Cd[1][17]; Md[186][389] = Cd[1][20]; Md[186][390] = Cd[1][22]; Md[186][391] = Cd[1][26]; Md[186][392] = Cd[1][28]; Md[186][393] = Cd[1][31]; 
   Md[187][164] = Cd[1][24]; Md[187][165] = Cd[1][30]; Md[187][166] = Cd[1][33]; Md[187][181] = Cd[1][5]; Md[187][182] = Cd[1][6]; Md[187][183] = Cd[1][7]; Md[187][184] = Cd[1][9]; Md[187][185] = Cd[1][10]; Md[187][186] = Cd[1][12]; Md[187][187] = Cd[1][15]; Md[187][188] = Cd[1][16]; Md[187][189] = Cd[1][17]; Md[187][190] = Cd[1][19]; Md[187][191] = Cd[1][20]; Md[187][192] = Cd[1][22]; Md[187][193] = Cd[1][26]; Md[187][194] = Cd[1][28]; Md[187][283] = Cd[1][25]; Md[187][394] = Cd[1][31]; 
   Md[188][34] = Cd[1][30]; Md[188][35] = Cd[1][33]; Md[188][49] = Cd[1][10]; Md[188][50] = Cd[1][12]; Md[188][52] = Cd[1][28]; Md[188][165] = Cd[1][24]; Md[188][182] = Cd[1][5]; Md[188][184] = Cd[1][6]; Md[188][185] = Cd[1][7]; Md[188][188] = Cd[1][15]; Md[188][190] = Cd[1][16]; Md[188][191] = Cd[1][17]; Md[188][193] = Cd[1][20]; Md[188][194] = Cd[1][22]; Md[188][277] = Cd[1][9]; Md[188][283] = Cd[1][19]; Md[188][284] = Cd[1][25]; Md[188][285] = Cd[1][26]; Md[188][395] = Cd[1][31]; 
   Md[189][35] = Cd[1][30]; Md[189][36] = Cd[1][33]; Md[189][49] = Cd[1][9]; Md[189][50] = Cd[1][10]; Md[189][52] = Cd[1][26]; Md[189][166] = Cd[1][24]; Md[189][183] = Cd[1][5]; Md[189][185] = Cd[1][6]; Md[189][186] = Cd[1][7]; Md[189][189] = Cd[1][15]; Md[189][191] = Cd[1][16]; Md[189][192] = Cd[1][17]; Md[189][193] = Cd[1][19]; Md[189][194] = Cd[1][20]; Md[189][285] = Cd[1][25]; Md[189][388] = Cd[1][12]; Md[189][394] = Cd[1][22]; Md[189][395] = Cd[1][28]; Md[189][396] = Cd[1][31]; 
   Md[190][34] = Cd[1][24]; Md[190][37] = Cd[1][33]; Md[190][49] = Cd[1][7]; Md[190][51] = Cd[1][12]; Md[190][52] = Cd[1][22]; Md[190][184] = Cd[1][5]; Md[190][190] = Cd[1][15]; Md[190][193] = Cd[1][17]; Md[190][260] = Cd[1][30]; Md[190][277] = Cd[1][6]; Md[190][278] = Cd[1][9]; Md[190][279] = Cd[1][10]; Md[190][283] = Cd[1][16]; Md[190][284] = Cd[1][19]; Md[190][285] = Cd[1][20]; Md[190][286] = Cd[1][25]; Md[190][287] = Cd[1][26]; Md[190][288] = Cd[1][28]; Md[190][397] = Cd[1][31]; 
   Md[191][35] = Cd[1][24]; Md[191][37] = Cd[1][30]; Md[191][38] = Cd[1][33]; Md[191][49] = Cd[1][6]; Md[191][50] = Cd[1][7]; Md[191][51] = Cd[1][10]; Md[191][52] = Cd[1][20]; Md[191][185] = Cd[1][5]; Md[191][191] = Cd[1][15]; Md[191][193] = Cd[1][16]; Md[191][194] = Cd[1][17]; Md[191][279] = Cd[1][9]; Md[191][285] = Cd[1][19]; Md[191][287] = Cd[1][25]; Md[191][288] = Cd[1][26]; Md[191][389] = Cd[1][12]; Md[191][395] = Cd[1][22]; Md[191][397] = Cd[1][28]; Md[191][398] = Cd[1][31]; 
   Md[192][36] = Cd[1][24]; Md[192][38] = Cd[1][30]; Md[192][50] = Cd[1][6]; Md[192][51] = Cd[1][9]; Md[192][52] = Cd[1][19]; Md[192][186] = Cd[1][5]; Md[192][192] = Cd[1][15]; Md[192][194] = Cd[1][16]; Md[192][288] = Cd[1][25]; Md[192][371] = Cd[1][33]; Md[192][388] = Cd[1][7]; Md[192][389] = Cd[1][10]; Md[192][390] = Cd[1][12]; Md[192][394] = Cd[1][17]; Md[192][395] = Cd[1][20]; Md[192][396] = Cd[1][22]; Md[192][397] = Cd[1][26]; Md[192][398] = Cd[1][28]; Md[192][399] = Cd[1][31]; 
   Md[193][37] = Cd[1][24]; Md[193][39] = Cd[1][33]; Md[193][49] = Cd[1][5]; Md[193][51] = Cd[1][7]; Md[193][52] = Cd[1][17]; Md[193][193] = Cd[1][15]; Md[193][262] = Cd[1][30]; Md[193][279] = Cd[1][6]; Md[193][281] = Cd[1][9]; Md[193][282] = Cd[1][10]; Md[193][285] = Cd[1][16]; Md[193][287] = Cd[1][19]; Md[193][288] = Cd[1][20]; Md[193][290] = Cd[1][25]; Md[193][291] = Cd[1][26]; Md[193][391] = Cd[1][12]; Md[193][397] = Cd[1][22]; Md[193][400] = Cd[1][28]; Md[193][401] = Cd[1][31]; 
   Md[194][38] = Cd[1][24]; Md[194][39] = Cd[1][30]; Md[194][50] = Cd[1][5]; Md[194][51] = Cd[1][6]; Md[194][52] = Cd[1][16]; Md[194][194] = Cd[1][15]; Md[194][282] = Cd[1][9]; Md[194][288] = Cd[1][19]; Md[194][291] = Cd[1][25]; Md[194][372] = Cd[1][33]; Md[194][389] = Cd[1][7]; Md[194][391] = Cd[1][10]; Md[194][392] = Cd[1][12]; Md[194][395] = Cd[1][17]; Md[194][397] = Cd[1][20]; Md[194][398] = Cd[1][22]; Md[194][400] = Cd[1][26]; Md[194][401] = Cd[1][28]; Md[194][402] = Cd[1][31]; 
   Md[195][70] = Cd[1][24]; Md[195][71] = Cd[1][30]; Md[195][72] = Cd[1][33]; Md[195][113] = Cd[1][5]; Md[195][114] = Cd[1][6]; Md[195][115] = Cd[1][7]; Md[195][116] = Cd[1][9]; Md[195][117] = Cd[1][10]; Md[195][118] = Cd[1][12]; Md[195][119] = Cd[1][16]; Md[195][120] = Cd[1][17]; Md[195][121] = Cd[1][19]; Md[195][122] = Cd[1][20]; Md[195][123] = Cd[1][22]; Md[195][124] = Cd[1][26]; Md[195][125] = Cd[1][28]; Md[195][195] = Cd[1][15]; Md[195][292] = Cd[1][25]; Md[195][404] = Cd[1][31]; 
   Md[196][73] = Cd[1][24]; Md[196][74] = Cd[1][30]; Md[196][75] = Cd[1][33]; Md[196][119] = Cd[1][6]; Md[196][120] = Cd[1][7]; Md[196][121] = Cd[1][9]; Md[196][122] = Cd[1][10]; Md[196][123] = Cd[1][12]; Md[196][126] = Cd[1][19]; Md[196][127] = Cd[1][20]; Md[196][128] = Cd[1][22]; Md[196][129] = Cd[1][26]; Md[196][130] = Cd[1][28]; Md[196][195] = Cd[1][5]; Md[196][196] = Cd[1][15]; Md[196][197] = Cd[1][16]; Md[196][198] = Cd[1][17]; Md[196][293] = Cd[1][25]; Md[196][405] = Cd[1][31]; 
   Md[197][74] = Cd[1][24]; Md[197][76] = Cd[1][30]; Md[197][77] = Cd[1][33]; Md[197][119] = Cd[1][5]; Md[197][121] = Cd[1][6]; Md[197][122] = Cd[1][7]; Md[197][124] = Cd[1][10]; Md[197][125] = Cd[1][12]; Md[197][126] = Cd[1][16]; Md[197][127] = Cd[1][17]; Md[197][129] = Cd[1][20]; Md[197][130] = Cd[1][22]; Md[197][131] = Cd[1][28]; Md[197][197] = Cd[1][15]; Md[197][292] = Cd[1][9]; Md[197][293] = Cd[1][19]; Md[197][294] = Cd[1][25]; Md[197][295] = Cd[1][26]; Md[197][406] = Cd[1][31]; 
   Md[198][75] = Cd[1][24]; Md[198][77] = Cd[1][30]; Md[198][78] = Cd[1][33]; Md[198][120] = Cd[1][5]; Md[198][122] = Cd[1][6]; Md[198][123] = Cd[1][7]; Md[198][124] = Cd[1][9]; Md[198][125] = Cd[1][10]; Md[198][127] = Cd[1][16]; Md[198][128] = Cd[1][17]; Md[198][129] = Cd[1][19]; Md[198][130] = Cd[1][20]; Md[198][131] = Cd[1][26]; Md[198][198] = Cd[1][15]; Md[198][295] = Cd[1][25]; Md[198][404] = Cd[1][12]; Md[198][405] = Cd[1][22]; Md[198][406] = Cd[1][28]; Md[198][407] = Cd[1][31]; 
   Md[199][79] = Cd[1][30]; Md[199][80] = Cd[1][33]; Md[199][126] = Cd[1][9]; Md[199][127] = Cd[1][10]; Md[199][128] = Cd[1][12]; Md[199][132] = Cd[1][26]; Md[199][133] = Cd[1][28]; Md[199][167] = Cd[1][24]; Md[199][196] = Cd[1][5]; Md[199][197] = Cd[1][6]; Md[199][198] = Cd[1][7]; Md[199][199] = Cd[1][15]; Md[199][200] = Cd[1][16]; Md[199][201] = Cd[1][17]; Md[199][202] = Cd[1][19]; Md[199][203] = Cd[1][20]; Md[199][204] = Cd[1][22]; Md[199][296] = Cd[1][25]; Md[199][408] = Cd[1][31]; 
   Md[200][79] = Cd[1][24]; Md[200][81] = Cd[1][30]; Md[200][82] = Cd[1][33]; Md[200][126] = Cd[1][6]; Md[200][127] = Cd[1][7]; Md[200][129] = Cd[1][10]; Md[200][130] = Cd[1][12]; Md[200][132] = Cd[1][20]; Md[200][133] = Cd[1][22]; Md[200][134] = Cd[1][28]; Md[200][197] = Cd[1][5]; Md[200][200] = Cd[1][15]; Md[200][202] = Cd[1][16]; Md[200][203] = Cd[1][17]; Md[200][293] = Cd[1][9]; Md[200][296] = Cd[1][19]; Md[200][297] = Cd[1][25]; Md[200][298] = Cd[1][26]; Md[200][409] = Cd[1][31]; 
   Md[201][80] = Cd[1][24]; Md[201][82] = Cd[1][30]; Md[201][83] = Cd[1][33]; Md[201][127] = Cd[1][6]; Md[201][128] = Cd[1][7]; Md[201][129] = Cd[1][9]; Md[201][130] = Cd[1][10]; Md[201][132] = Cd[1][19]; Md[201][133] = Cd[1][20]; Md[201][134] = Cd[1][26]; Md[201][198] = Cd[1][5]; Md[201][201] = Cd[1][15]; Md[201][203] = Cd[1][16]; Md[201][204] = Cd[1][17]; Md[201][298] = Cd[1][25]; Md[201][405] = Cd[1][12]; Md[201][408] = Cd[1][22]; Md[201][409] = Cd[1][28]; Md[201][410] = Cd[1][31]; 
   Md[202][81] = Cd[1][24]; Md[202][84] = Cd[1][33]; Md[202][126] = Cd[1][5]; Md[202][129] = Cd[1][7]; Md[202][131] = Cd[1][12]; Md[202][132] = Cd[1][17]; Md[202][134] = Cd[1][22]; Md[202][202] = Cd[1][15]; Md[202][263] = Cd[1][30]; Md[202][293] = Cd[1][6]; Md[202][294] = Cd[1][9]; Md[202][295] = Cd[1][10]; Md[202][296] = Cd[1][16]; Md[202][297] = Cd[1][19]; Md[202][298] = Cd[1][20]; Md[202][299] = Cd[1][25]; Md[202][300] = Cd[1][26]; Md[202][301] = Cd[1][28]; Md[202][411] = Cd[1][31]; 
   Md[203][82] = Cd[1][24]; Md[203][84] = Cd[1][30]; Md[203][85] = Cd[1][33]; Md[203][127] = Cd[1][5]; Md[203][129] = Cd[1][6]; Md[203][130] = Cd[1][7]; Md[203][131] = Cd[1][10]; Md[203][132] = Cd[1][16]; Md[203][133] = Cd[1][17]; Md[203][134] = Cd[1][20]; Md[203][203] = Cd[1][15]; Md[203][295] = Cd[1][9]; Md[203][298] = Cd[1][19]; Md[203][300] = Cd[1][25]; Md[203][301] = Cd[1][26]; Md[203][406] = Cd[1][12]; Md[203][409] = Cd[1][22]; Md[203][411] = Cd[1][28]; Md[203][412] = Cd[1][31]; 
   Md[204][83] = Cd[1][24]; Md[204][85] = Cd[1][30]; Md[204][128] = Cd[1][5]; Md[204][130] = Cd[1][6]; Md[204][131] = Cd[1][9]; Md[204][133] = Cd[1][16]; Md[204][134] = Cd[1][19]; Md[204][204] = Cd[1][15]; Md[204][301] = Cd[1][25]; Md[204][374] = Cd[1][33]; Md[204][405] = Cd[1][7]; Md[204][406] = Cd[1][10]; Md[204][407] = Cd[1][12]; Md[204][408] = Cd[1][17]; Md[204][409] = Cd[1][20]; Md[204][410] = Cd[1][22]; Md[204][411] = Cd[1][26]; Md[204][412] = Cd[1][28]; Md[204][413] = Cd[1][31]; 
   Md[205][168] = Cd[1][24]; Md[205][169] = Cd[1][30]; Md[205][170] = Cd[1][33]; Md[205][199] = Cd[1][5]; Md[205][200] = Cd[1][6]; Md[205][201] = Cd[1][7]; Md[205][202] = Cd[1][9]; Md[205][203] = Cd[1][10]; Md[205][204] = Cd[1][12]; Md[205][205] = Cd[1][15]; Md[205][206] = Cd[1][16]; Md[205][207] = Cd[1][17]; Md[205][208] = Cd[1][19]; Md[205][209] = Cd[1][20]; Md[205][210] = Cd[1][22]; Md[205][211] = Cd[1][26]; Md[205][212] = Cd[1][28]; Md[205][302] = Cd[1][25]; Md[205][414] = Cd[1][31]; 
   Md[206][40] = Cd[1][30]; Md[206][41] = Cd[1][33]; Md[206][53] = Cd[1][28]; Md[206][132] = Cd[1][10]; Md[206][133] = Cd[1][12]; Md[206][169] = Cd[1][24]; Md[206][200] = Cd[1][5]; Md[206][202] = Cd[1][6]; Md[206][203] = Cd[1][7]; Md[206][206] = Cd[1][15]; Md[206][208] = Cd[1][16]; Md[206][209] = Cd[1][17]; Md[206][211] = Cd[1][20]; Md[206][212] = Cd[1][22]; Md[206][296] = Cd[1][9]; Md[206][302] = Cd[1][19]; Md[206][303] = Cd[1][25]; Md[206][304] = Cd[1][26]; Md[206][415] = Cd[1][31]; 
   Md[207][41] = Cd[1][30]; Md[207][42] = Cd[1][33]; Md[207][53] = Cd[1][26]; Md[207][132] = Cd[1][9]; Md[207][133] = Cd[1][10]; Md[207][170] = Cd[1][24]; Md[207][201] = Cd[1][5]; Md[207][203] = Cd[1][6]; Md[207][204] = Cd[1][7]; Md[207][207] = Cd[1][15]; Md[207][209] = Cd[1][16]; Md[207][210] = Cd[1][17]; Md[207][211] = Cd[1][19]; Md[207][212] = Cd[1][20]; Md[207][304] = Cd[1][25]; Md[207][408] = Cd[1][12]; Md[207][414] = Cd[1][22]; Md[207][415] = Cd[1][28]; Md[207][416] = Cd[1][31]; 
   Md[208][40] = Cd[1][24]; Md[208][43] = Cd[1][33]; Md[208][53] = Cd[1][22]; Md[208][132] = Cd[1][7]; Md[208][134] = Cd[1][12]; Md[208][202] = Cd[1][5]; Md[208][208] = Cd[1][15]; Md[208][211] = Cd[1][17]; Md[208][264] = Cd[1][30]; Md[208][296] = Cd[1][6]; Md[208][297] = Cd[1][9]; Md[208][298] = Cd[1][10]; Md[208][302] = Cd[1][16]; Md[208][303] = Cd[1][19]; Md[208][304] = Cd[1][20]; Md[208][305] = Cd[1][25]; Md[208][306] = Cd[1][26]; Md[208][307] = Cd[1][28]; Md[208][417] = Cd[1][31]; 
   Md[209][41] = Cd[1][24]; Md[209][43] = Cd[1][30]; Md[209][44] = Cd[1][33]; Md[209][53] = Cd[1][20]; Md[209][132] = Cd[1][6]; Md[209][133] = Cd[1][7]; Md[209][134] = Cd[1][10]; Md[209][203] = Cd[1][5]; Md[209][209] = Cd[1][15]; Md[209][211] = Cd[1][16]; Md[209][212] = Cd[1][17]; Md[209][298] = Cd[1][9]; Md[209][304] = Cd[1][19]; Md[209][306] = Cd[1][25]; Md[209][307] = Cd[1][26]; Md[209][409] = Cd[1][12]; Md[209][415] = Cd[1][22]; Md[209][417] = Cd[1][28]; Md[209][418] = Cd[1][31]; 
   Md[210][42] = Cd[1][24]; Md[210][44] = Cd[1][30]; Md[210][53] = Cd[1][19]; Md[210][133] = Cd[1][6]; Md[210][134] = Cd[1][9]; Md[210][204] = Cd[1][5]; Md[210][210] = Cd[1][15]; Md[210][212] = Cd[1][16]; Md[210][307] = Cd[1][25]; Md[210][375] = Cd[1][33]; Md[210][408] = Cd[1][7]; Md[210][409] = Cd[1][10]; Md[210][410] = Cd[1][12]; Md[210][414] = Cd[1][17]; Md[210][415] = Cd[1][20]; Md[210][416] = Cd[1][22]; Md[210][417] = Cd[1][26]; Md[210][418] = Cd[1][28]; Md[210][419] = Cd[1][31]; 
   Md[211][43] = Cd[1][24]; Md[211][45] = Cd[1][33]; Md[211][53] = Cd[1][17]; Md[211][132] = Cd[1][5]; Md[211][134] = Cd[1][7]; Md[211][211] = Cd[1][15]; Md[211][266] = Cd[1][30]; Md[211][298] = Cd[1][6]; Md[211][300] = Cd[1][9]; Md[211][301] = Cd[1][10]; Md[211][304] = Cd[1][16]; Md[211][306] = Cd[1][19]; Md[211][307] = Cd[1][20]; Md[211][309] = Cd[1][25]; Md[211][310] = Cd[1][26]; Md[211][411] = Cd[1][12]; Md[211][417] = Cd[1][22]; Md[211][420] = Cd[1][28]; Md[211][421] = Cd[1][31]; 
   Md[212][44] = Cd[1][24]; Md[212][45] = Cd[1][30]; Md[212][53] = Cd[1][16]; Md[212][133] = Cd[1][5]; Md[212][134] = Cd[1][6]; Md[212][212] = Cd[1][15]; Md[212][301] = Cd[1][9]; Md[212][307] = Cd[1][19]; Md[212][310] = Cd[1][25]; Md[212][376] = Cd[1][33]; Md[212][409] = Cd[1][7]; Md[212][411] = Cd[1][10]; Md[212][412] = Cd[1][12]; Md[212][415] = Cd[1][17]; Md[212][417] = Cd[1][20]; Md[212][418] = Cd[1][22]; Md[212][420] = Cd[1][26]; Md[212][421] = Cd[1][28]; Md[212][422] = Cd[1][31]; 
   Md[213][171] = Cd[1][24]; Md[213][172] = Cd[1][30]; Md[213][173] = Cd[1][33]; Md[213][205] = Cd[1][5]; Md[213][206] = Cd[1][6]; Md[213][207] = Cd[1][7]; Md[213][208] = Cd[1][9]; Md[213][209] = Cd[1][10]; Md[213][210] = Cd[1][12]; Md[213][213] = Cd[1][15]; Md[213][214] = Cd[1][16]; Md[213][215] = Cd[1][17]; Md[213][216] = Cd[1][19]; Md[213][217] = Cd[1][20]; Md[213][218] = Cd[1][22]; Md[213][219] = Cd[1][26]; Md[213][220] = Cd[1][28]; Md[213][311] = Cd[1][25]; Md[213][424] = Cd[1][31]; 
   Md[214][172] = Cd[1][24]; Md[214][174] = Cd[1][30]; Md[214][175] = Cd[1][33]; Md[214][206] = Cd[1][5]; Md[214][208] = Cd[1][6]; Md[214][209] = Cd[1][7]; Md[214][211] = Cd[1][10]; Md[214][212] = Cd[1][12]; Md[214][214] = Cd[1][15]; Md[214][216] = Cd[1][16]; Md[214][217] = Cd[1][17]; Md[214][219] = Cd[1][20]; Md[214][220] = Cd[1][22]; Md[214][221] = Cd[1][28]; Md[214][302] = Cd[1][9]; Md[214][311] = Cd[1][19]; Md[214][312] = Cd[1][25]; Md[214][313] = Cd[1][26]; Md[214][425] = Cd[1][31]; 
   Md[215][173] = Cd[1][24]; Md[215][175] = Cd[1][30]; Md[215][176] = Cd[1][33]; Md[215][207] = Cd[1][5]; Md[215][209] = Cd[1][6]; Md[215][210] = Cd[1][7]; Md[215][211] = Cd[1][9]; Md[215][212] = Cd[1][10]; Md[215][215] = Cd[1][15]; Md[215][217] = Cd[1][16]; Md[215][218] = Cd[1][17]; Md[215][219] = Cd[1][19]; Md[215][220] = Cd[1][20]; Md[215][221] = Cd[1][26]; Md[215][313] = Cd[1][25]; Md[215][414] = Cd[1][12]; Md[215][424] = Cd[1][22]; Md[215][425] = Cd[1][28]; Md[215][426] = Cd[1][31]; 
   Md[216][46] = Cd[1][33]; Md[216][53] = Cd[1][12]; Md[216][174] = Cd[1][24]; Md[216][208] = Cd[1][5]; Md[216][211] = Cd[1][7]; Md[216][216] = Cd[1][15]; Md[216][219] = Cd[1][17]; Md[216][221] = Cd[1][22]; Md[216][267] = Cd[1][30]; Md[216][302] = Cd[1][6]; Md[216][303] = Cd[1][9]; Md[216][304] = Cd[1][10]; Md[216][311] = Cd[1][16]; Md[216][312] = Cd[1][19]; Md[216][313] = Cd[1][20]; Md[216][314] = Cd[1][25]; Md[216][315] = Cd[1][26]; Md[216][316] = Cd[1][28]; Md[216][427] = Cd[1][31]; 
   Md[217][46] = Cd[1][30]; Md[217][47] = Cd[1][33]; Md[217][53] = Cd[1][10]; Md[217][175] = Cd[1][24]; Md[217][209] = Cd[1][5]; Md[217][211] = Cd[1][6]; Md[217][212] = Cd[1][7]; Md[217][217] = Cd[1][15]; Md[217][219] = Cd[1][16]; Md[217][220] = Cd[1][17]; Md[217][221] = Cd[1][20]; Md[217][304] = Cd[1][9]; Md[217][313] = Cd[1][19]; Md[217][315] = Cd[1][25]; Md[217][316] = Cd[1][26]; Md[217][415] = Cd[1][12]; Md[217][425] = Cd[1][22]; Md[217][427] = Cd[1][28]; Md[217][428] = Cd[1][31]; 
   Md[218][47] = Cd[1][30]; Md[218][53] = Cd[1][9]; Md[218][176] = Cd[1][24]; Md[218][210] = Cd[1][5]; Md[218][212] = Cd[1][6]; Md[218][218] = Cd[1][15]; Md[218][220] = Cd[1][16]; Md[218][221] = Cd[1][19]; Md[218][316] = Cd[1][25]; Md[218][378] = Cd[1][33]; Md[218][414] = Cd[1][7]; Md[218][415] = Cd[1][10]; Md[218][416] = Cd[1][12]; Md[218][424] = Cd[1][17]; Md[218][425] = Cd[1][20]; Md[218][426] = Cd[1][22]; Md[218][427] = Cd[1][26]; Md[218][428] = Cd[1][28]; Md[218][429] = Cd[1][31]; 
   Md[219][46] = Cd[1][24]; Md[219][48] = Cd[1][33]; Md[219][53] = Cd[1][7]; Md[219][211] = Cd[1][5]; Md[219][219] = Cd[1][15]; Md[219][221] = Cd[1][17]; Md[219][269] = Cd[1][30]; Md[219][304] = Cd[1][6]; Md[219][306] = Cd[1][9]; Md[219][307] = Cd[1][10]; Md[219][313] = Cd[1][16]; Md[219][315] = Cd[1][19]; Md[219][316] = Cd[1][20]; Md[219][318] = Cd[1][25]; Md[219][319] = Cd[1][26]; Md[219][417] = Cd[1][12]; Md[219][427] = Cd[1][22]; Md[219][430] = Cd[1][28]; Md[219][431] = Cd[1][31]; 
   Md[220][47] = Cd[1][24]; Md[220][48] = Cd[1][30]; Md[220][53] = Cd[1][6]; Md[220][212] = Cd[1][5]; Md[220][220] = Cd[1][15]; Md[220][221] = Cd[1][16]; Md[220][307] = Cd[1][9]; Md[220][316] = Cd[1][19]; Md[220][319] = Cd[1][25]; Md[220][379] = Cd[1][33]; Md[220][415] = Cd[1][7]; Md[220][417] = Cd[1][10]; Md[220][418] = Cd[1][12]; Md[220][425] = Cd[1][17]; Md[220][427] = Cd[1][20]; Md[220][428] = Cd[1][22]; Md[220][430] = Cd[1][26]; Md[220][431] = Cd[1][28]; Md[220][432] = Cd[1][31]; 
   Md[221][48] = Cd[1][24]; Md[221][53] = Cd[1][5]; Md[221][221] = Cd[1][15]; Md[221][272] = Cd[1][30]; Md[221][307] = Cd[1][6]; Md[221][310] = Cd[1][9]; Md[221][316] = Cd[1][16]; Md[221][319] = Cd[1][19]; Md[221][322] = Cd[1][25]; Md[221][381] = Cd[1][33]; Md[221][417] = Cd[1][7]; Md[221][420] = Cd[1][10]; Md[221][421] = Cd[1][12]; Md[221][427] = Cd[1][17]; Md[221][430] = Cd[1][20]; Md[221][431] = Cd[1][22]; Md[221][434] = Cd[1][26]; Md[221][435] = Cd[1][28]; Md[221][436] = Cd[1][31]; 
   Md[222][87] = Cd[1][24]; Md[222][88] = Cd[1][30]; Md[222][89] = Cd[1][33]; Md[222][139] = Cd[1][5]; Md[222][140] = Cd[1][6]; Md[222][141] = Cd[1][7]; Md[222][142] = Cd[1][9]; Md[222][143] = Cd[1][10]; Md[222][144] = Cd[1][12]; Md[222][145] = Cd[1][16]; Md[222][146] = Cd[1][17]; Md[222][147] = Cd[1][19]; Md[222][148] = Cd[1][20]; Md[222][149] = Cd[1][22]; Md[222][150] = Cd[1][26]; Md[222][151] = Cd[1][28]; Md[222][222] = Cd[1][15]; Md[222][323] = Cd[1][25]; Md[222][439] = Cd[1][31]; 
   Md[223][90] = Cd[1][24]; Md[223][91] = Cd[1][30]; Md[223][92] = Cd[1][33]; Md[223][145] = Cd[1][6]; Md[223][146] = Cd[1][7]; Md[223][147] = Cd[1][9]; Md[223][148] = Cd[1][10]; Md[223][149] = Cd[1][12]; Md[223][152] = Cd[1][19]; Md[223][153] = Cd[1][20]; Md[223][154] = Cd[1][22]; Md[223][155] = Cd[1][26]; Md[223][156] = Cd[1][28]; Md[223][222] = Cd[1][5]; Md[223][223] = Cd[1][15]; Md[223][224] = Cd[1][16]; Md[223][225] = Cd[1][17]; Md[223][324] = Cd[1][25]; Md[223][440] = Cd[1][31]; 
   Md[224][91] = Cd[1][24]; Md[224][93] = Cd[1][30]; Md[224][94] = Cd[1][33]; Md[224][145] = Cd[1][5]; Md[224][147] = Cd[1][6]; Md[224][148] = Cd[1][7]; Md[224][150] = Cd[1][10]; Md[224][151] = Cd[1][12]; Md[224][152] = Cd[1][16]; Md[224][153] = Cd[1][17]; Md[224][155] = Cd[1][20]; Md[224][156] = Cd[1][22]; Md[224][157] = Cd[1][28]; Md[224][224] = Cd[1][15]; Md[224][323] = Cd[1][9]; Md[224][324] = Cd[1][19]; Md[224][325] = Cd[1][25]; Md[224][326] = Cd[1][26]; Md[224][441] = Cd[1][31]; 
   Md[225][92] = Cd[1][24]; Md[225][94] = Cd[1][30]; Md[225][95] = Cd[1][33]; Md[225][146] = Cd[1][5]; Md[225][148] = Cd[1][6]; Md[225][149] = Cd[1][7]; Md[225][150] = Cd[1][9]; Md[225][151] = Cd[1][10]; Md[225][153] = Cd[1][16]; Md[225][154] = Cd[1][17]; Md[225][155] = Cd[1][19]; Md[225][156] = Cd[1][20]; Md[225][157] = Cd[1][26]; Md[225][225] = Cd[1][15]; Md[225][326] = Cd[1][25]; Md[225][439] = Cd[1][12]; Md[225][440] = Cd[1][22]; Md[225][441] = Cd[1][28]; Md[225][442] = Cd[1][31]; 
   Md[226][96] = Cd[1][30]; Md[226][97] = Cd[1][33]; Md[226][152] = Cd[1][9]; Md[226][153] = Cd[1][10]; Md[226][154] = Cd[1][12]; Md[226][158] = Cd[1][26]; Md[226][159] = Cd[1][28]; Md[226][177] = Cd[1][24]; Md[226][223] = Cd[1][5]; Md[226][224] = Cd[1][6]; Md[226][225] = Cd[1][7]; Md[226][226] = Cd[1][15]; Md[226][227] = Cd[1][16]; Md[226][228] = Cd[1][17]; Md[226][229] = Cd[1][19]; Md[226][230] = Cd[1][20]; Md[226][231] = Cd[1][22]; Md[226][327] = Cd[1][25]; Md[226][443] = Cd[1][31]; 
   Md[227][96] = Cd[1][24]; Md[227][98] = Cd[1][30]; Md[227][99] = Cd[1][33]; Md[227][152] = Cd[1][6]; Md[227][153] = Cd[1][7]; Md[227][155] = Cd[1][10]; Md[227][156] = Cd[1][12]; Md[227][158] = Cd[1][20]; Md[227][159] = Cd[1][22]; Md[227][160] = Cd[1][28]; Md[227][224] = Cd[1][5]; Md[227][227] = Cd[1][15]; Md[227][229] = Cd[1][16]; Md[227][230] = Cd[1][17]; Md[227][324] = Cd[1][9]; Md[227][327] = Cd[1][19]; Md[227][328] = Cd[1][25]; Md[227][329] = Cd[1][26]; Md[227][444] = Cd[1][31]; 
   Md[228][97] = Cd[1][24]; Md[228][99] = Cd[1][30]; Md[228][100] = Cd[1][33]; Md[228][153] = Cd[1][6]; Md[228][154] = Cd[1][7]; Md[228][155] = Cd[1][9]; Md[228][156] = Cd[1][10]; Md[228][158] = Cd[1][19]; Md[228][159] = Cd[1][20]; Md[228][160] = Cd[1][26]; Md[228][225] = Cd[1][5]; Md[228][228] = Cd[1][15]; Md[228][230] = Cd[1][16]; Md[228][231] = Cd[1][17]; Md[228][329] = Cd[1][25]; Md[228][440] = Cd[1][12]; Md[228][443] = Cd[1][22]; Md[228][444] = Cd[1][28]; Md[228][445] = Cd[1][31]; 
   Md[229][98] = Cd[1][24]; Md[229][101] = Cd[1][33]; Md[229][152] = Cd[1][5]; Md[229][155] = Cd[1][7]; Md[229][157] = Cd[1][12]; Md[229][158] = Cd[1][17]; Md[229][160] = Cd[1][22]; Md[229][229] = Cd[1][15]; Md[229][273] = Cd[1][30]; Md[229][324] = Cd[1][6]; Md[229][325] = Cd[1][9]; Md[229][326] = Cd[1][10]; Md[229][327] = Cd[1][16]; Md[229][328] = Cd[1][19]; Md[229][329] = Cd[1][20]; Md[229][330] = Cd[1][25]; Md[229][331] = Cd[1][26]; Md[229][332] = Cd[1][28]; Md[229][446] = Cd[1][31]; 
   Md[230][99] = Cd[1][24]; Md[230][101] = Cd[1][30]; Md[230][102] = Cd[1][33]; Md[230][153] = Cd[1][5]; Md[230][155] = Cd[1][6]; Md[230][156] = Cd[1][7]; Md[230][157] = Cd[1][10]; Md[230][158] = Cd[1][16]; Md[230][159] = Cd[1][17]; Md[230][160] = Cd[1][20]; Md[230][230] = Cd[1][15]; Md[230][326] = Cd[1][9]; Md[230][329] = Cd[1][19]; Md[230][331] = Cd[1][25]; Md[230][332] = Cd[1][26]; Md[230][441] = Cd[1][12]; Md[230][444] = Cd[1][22]; Md[230][446] = Cd[1][28]; Md[230][447] = Cd[1][31]; 
   Md[231][100] = Cd[1][24]; Md[231][102] = Cd[1][30]; Md[231][154] = Cd[1][5]; Md[231][156] = Cd[1][6]; Md[231][157] = Cd[1][9]; Md[231][159] = Cd[1][16]; Md[231][160] = Cd[1][19]; Md[231][231] = Cd[1][15]; Md[231][332] = Cd[1][25]; Md[231][384] = Cd[1][33]; Md[231][440] = Cd[1][7]; Md[231][441] = Cd[1][10]; Md[231][442] = Cd[1][12]; Md[231][443] = Cd[1][17]; Md[231][444] = Cd[1][20]; Md[231][445] = Cd[1][22]; Md[231][446] = Cd[1][26]; Md[231][447] = Cd[1][28]; Md[231][448] = Cd[1][31]; 
   Md[232][178] = Cd[1][24]; Md[232][179] = Cd[1][30]; Md[232][180] = Cd[1][33]; Md[232][226] = Cd[1][5]; Md[232][227] = Cd[1][6]; Md[232][228] = Cd[1][7]; Md[232][229] = Cd[1][9]; Md[232][230] = Cd[1][10]; Md[232][231] = Cd[1][12]; Md[232][232] = Cd[1][15]; Md[232][233] = Cd[1][16]; Md[232][234] = Cd[1][17]; Md[232][235] = Cd[1][19]; Md[232][236] = Cd[1][20]; Md[232][237] = Cd[1][22]; Md[232][238] = Cd[1][26]; Md[232][239] = Cd[1][28]; Md[232][333] = Cd[1][25]; Md[232][449] = Cd[1][31]; 
   Md[233][103] = Cd[1][30]; Md[233][104] = Cd[1][33]; Md[233][158] = Cd[1][10]; Md[233][159] = Cd[1][12]; Md[233][161] = Cd[1][28]; Md[233][179] = Cd[1][24]; Md[233][227] = Cd[1][5]; Md[233][229] = Cd[1][6]; Md[233][230] = Cd[1][7]; Md[233][233] = Cd[1][15]; Md[233][235] = Cd[1][16]; Md[233][236] = Cd[1][17]; Md[233][238] = Cd[1][20]; Md[233][239] = Cd[1][22]; Md[233][327] = Cd[1][9]; Md[233][333] = Cd[1][19]; Md[233][334] = Cd[1][25]; Md[233][335] = Cd[1][26]; Md[233][450] = Cd[1][31]; 
   Md[234][104] = Cd[1][30]; Md[234][105] = Cd[1][33]; Md[234][158] = Cd[1][9]; Md[234][159] = Cd[1][10]; Md[234][161] = Cd[1][26]; Md[234][180] = Cd[1][24]; Md[234][228] = Cd[1][5]; Md[234][230] = Cd[1][6]; Md[234][231] = Cd[1][7]; Md[234][234] = Cd[1][15]; Md[234][236] = Cd[1][16]; Md[234][237] = Cd[1][17]; Md[234][238] = Cd[1][19]; Md[234][239] = Cd[1][20]; Md[234][335] = Cd[1][25]; Md[234][443] = Cd[1][12]; Md[234][449] = Cd[1][22]; Md[234][450] = Cd[1][28]; Md[234][451] = Cd[1][31]; 
   Md[235][103] = Cd[1][24]; Md[235][106] = Cd[1][33]; Md[235][158] = Cd[1][7]; Md[235][160] = Cd[1][12]; Md[235][161] = Cd[1][22]; Md[235][229] = Cd[1][5]; Md[235][235] = Cd[1][15]; Md[235][238] = Cd[1][17]; Md[235][274] = Cd[1][30]; Md[235][327] = Cd[1][6]; Md[235][328] = Cd[1][9]; Md[235][329] = Cd[1][10]; Md[235][333] = Cd[1][16]; Md[235][334] = Cd[1][19]; Md[235][335] = Cd[1][20]; Md[235][336] = Cd[1][25]; Md[235][337] = Cd[1][26]; Md[235][338] = Cd[1][28]; Md[235][452] = Cd[1][31]; 
   Md[236][104] = Cd[1][24]; Md[236][106] = Cd[1][30]; Md[236][107] = Cd[1][33]; Md[236][158] = Cd[1][6]; Md[236][159] = Cd[1][7]; Md[236][160] = Cd[1][10]; Md[236][161] = Cd[1][20]; Md[236][230] = Cd[1][5]; Md[236][236] = Cd[1][15]; Md[236][238] = Cd[1][16]; Md[236][239] = Cd[1][17]; Md[236][329] = Cd[1][9]; Md[236][335] = Cd[1][19]; Md[236][337] = Cd[1][25]; Md[236][338] = Cd[1][26]; Md[236][444] = Cd[1][12]; Md[236][450] = Cd[1][22]; Md[236][452] = Cd[1][28]; Md[236][453] = Cd[1][31]; 
   Md[237][105] = Cd[1][24]; Md[237][107] = Cd[1][30]; Md[237][159] = Cd[1][6]; Md[237][160] = Cd[1][9]; Md[237][161] = Cd[1][19]; Md[237][231] = Cd[1][5]; Md[237][237] = Cd[1][15]; Md[237][239] = Cd[1][16]; Md[237][338] = Cd[1][25]; Md[237][385] = Cd[1][33]; Md[237][443] = Cd[1][7]; Md[237][444] = Cd[1][10]; Md[237][445] = Cd[1][12]; Md[237][449] = Cd[1][17]; Md[237][450] = Cd[1][20]; Md[237][451] = Cd[1][22]; Md[237][452] = Cd[1][26]; Md[237][453] = Cd[1][28]; Md[237][454] = Cd[1][31]; 
   Md[238][106] = Cd[1][24]; Md[238][108] = Cd[1][33]; Md[238][158] = Cd[1][5]; Md[238][160] = Cd[1][7]; Md[238][161] = Cd[1][17]; Md[238][238] = Cd[1][15]; Md[238][276] = Cd[1][30]; Md[238][329] = Cd[1][6]; Md[238][331] = Cd[1][9]; Md[238][332] = Cd[1][10]; Md[238][335] = Cd[1][16]; Md[238][337] = Cd[1][19]; Md[238][338] = Cd[1][20]; Md[238][340] = Cd[1][25]; Md[238][341] = Cd[1][26]; Md[238][446] = Cd[1][12]; Md[238][452] = Cd[1][22]; Md[238][455] = Cd[1][28]; Md[238][456] = Cd[1][31]; 
   Md[239][107] = Cd[1][24]; Md[239][108] = Cd[1][30]; Md[239][159] = Cd[1][5]; Md[239][160] = Cd[1][6]; Md[239][161] = Cd[1][16]; Md[239][239] = Cd[1][15]; Md[239][332] = Cd[1][9]; Md[239][338] = Cd[1][19]; Md[239][341] = Cd[1][25]; Md[239][386] = Cd[1][33]; Md[239][444] = Cd[1][7]; Md[239][446] = Cd[1][10]; Md[239][447] = Cd[1][12]; Md[239][450] = Cd[1][17]; Md[239][452] = Cd[1][20]; Md[239][453] = Cd[1][22]; Md[239][455] = Cd[1][26]; Md[239][456] = Cd[1][28]; Md[239][457] = Cd[1][31]; 
   Md[240][181] = Cd[1][24]; Md[240][182] = Cd[1][30]; Md[240][183] = Cd[1][33]; Md[240][232] = Cd[1][5]; Md[240][233] = Cd[1][6]; Md[240][234] = Cd[1][7]; Md[240][235] = Cd[1][9]; Md[240][236] = Cd[1][10]; Md[240][237] = Cd[1][12]; Md[240][240] = Cd[1][15]; Md[240][241] = Cd[1][16]; Md[240][242] = Cd[1][17]; Md[240][243] = Cd[1][19]; Md[240][244] = Cd[1][20]; Md[240][245] = Cd[1][22]; Md[240][246] = Cd[1][26]; Md[240][247] = Cd[1][28]; Md[240][342] = Cd[1][25]; Md[240][459] = Cd[1][31]; 
   Md[241][182] = Cd[1][24]; Md[241][184] = Cd[1][30]; Md[241][185] = Cd[1][33]; Md[241][233] = Cd[1][5]; Md[241][235] = Cd[1][6]; Md[241][236] = Cd[1][7]; Md[241][238] = Cd[1][10]; Md[241][239] = Cd[1][12]; Md[241][241] = Cd[1][15]; Md[241][243] = Cd[1][16]; Md[241][244] = Cd[1][17]; Md[241][246] = Cd[1][20]; Md[241][247] = Cd[1][22]; Md[241][248] = Cd[1][28]; Md[241][333] = Cd[1][9]; Md[241][342] = Cd[1][19]; Md[241][343] = Cd[1][25]; Md[241][344] = Cd[1][26]; Md[241][460] = Cd[1][31]; 
   Md[242][183] = Cd[1][24]; Md[242][185] = Cd[1][30]; Md[242][186] = Cd[1][33]; Md[242][234] = Cd[1][5]; Md[242][236] = Cd[1][6]; Md[242][237] = Cd[1][7]; Md[242][238] = Cd[1][9]; Md[242][239] = Cd[1][10]; Md[242][242] = Cd[1][15]; Md[242][244] = Cd[1][16]; Md[242][245] = Cd[1][17]; Md[242][246] = Cd[1][19]; Md[242][247] = Cd[1][20]; Md[242][248] = Cd[1][26]; Md[242][344] = Cd[1][25]; Md[242][449] = Cd[1][12]; Md[242][459] = Cd[1][22]; Md[242][460] = Cd[1][28]; Md[242][461] = Cd[1][31]; 
   Md[243][49] = Cd[1][33]; Md[243][161] = Cd[1][12]; Md[243][184] = Cd[1][24]; Md[243][235] = Cd[1][5]; Md[243][238] = Cd[1][7]; Md[243][243] = Cd[1][15]; Md[243][246] = Cd[1][17]; Md[243][248] = Cd[1][22]; Md[243][277] = Cd[1][30]; Md[243][333] = Cd[1][6]; Md[243][334] = Cd[1][9]; Md[243][335] = Cd[1][10]; Md[243][342] = Cd[1][16]; Md[243][343] = Cd[1][19]; Md[243][344] = Cd[1][20]; Md[243][345] = Cd[1][25]; Md[243][346] = Cd[1][26]; Md[243][347] = Cd[1][28]; Md[243][462] = Cd[1][31]; 
   Md[244][49] = Cd[1][30]; Md[244][50] = Cd[1][33]; Md[244][161] = Cd[1][10]; Md[244][185] = Cd[1][24]; Md[244][236] = Cd[1][5]; Md[244][238] = Cd[1][6]; Md[244][239] = Cd[1][7]; Md[244][244] = Cd[1][15]; Md[244][246] = Cd[1][16]; Md[244][247] = Cd[1][17]; Md[244][248] = Cd[1][20]; Md[244][335] = Cd[1][9]; Md[244][344] = Cd[1][19]; Md[244][346] = Cd[1][25]; Md[244][347] = Cd[1][26]; Md[244][450] = Cd[1][12]; Md[244][460] = Cd[1][22]; Md[244][462] = Cd[1][28]; Md[244][463] = Cd[1][31]; 
   Md[245][50] = Cd[1][30]; Md[245][161] = Cd[1][9]; Md[245][186] = Cd[1][24]; Md[245][237] = Cd[1][5]; Md[245][239] = Cd[1][6]; Md[245][245] = Cd[1][15]; Md[245][247] = Cd[1][16]; Md[245][248] = Cd[1][19]; Md[245][347] = Cd[1][25]; Md[245][388] = Cd[1][33]; Md[245][449] = Cd[1][7]; Md[245][450] = Cd[1][10]; Md[245][451] = Cd[1][12]; Md[245][459] = Cd[1][17]; Md[245][460] = Cd[1][20]; Md[245][461] = Cd[1][22]; Md[245][462] = Cd[1][26]; Md[245][463] = Cd[1][28]; Md[245][464] = Cd[1][31]; 
   Md[246][49] = Cd[1][24]; Md[246][51] = Cd[1][33]; Md[246][161] = Cd[1][7]; Md[246][238] = Cd[1][5]; Md[246][246] = Cd[1][15]; Md[246][248] = Cd[1][17]; Md[246][279] = Cd[1][30]; Md[246][335] = Cd[1][6]; Md[246][337] = Cd[1][9]; Md[246][338] = Cd[1][10]; Md[246][344] = Cd[1][16]; Md[246][346] = Cd[1][19]; Md[246][347] = Cd[1][20]; Md[246][349] = Cd[1][25]; Md[246][350] = Cd[1][26]; Md[246][452] = Cd[1][12]; Md[246][462] = Cd[1][22]; Md[246][465] = Cd[1][28]; Md[246][466] = Cd[1][31]; 
   Md[247][50] = Cd[1][24]; Md[247][51] = Cd[1][30]; Md[247][161] = Cd[1][6]; Md[247][239] = Cd[1][5]; Md[247][247] = Cd[1][15]; Md[247][248] = Cd[1][16]; Md[247][338] = Cd[1][9]; Md[247][347] = Cd[1][19]; Md[247][350] = Cd[1][25]; Md[247][389] = Cd[1][33]; Md[247][450] = Cd[1][7]; Md[247][452] = Cd[1][10]; Md[247][453] = Cd[1][12]; Md[247][460] = Cd[1][17]; Md[247][462] = Cd[1][20]; Md[247][463] = Cd[1][22]; Md[247][465] = Cd[1][26]; Md[247][466] = Cd[1][28]; Md[247][467] = Cd[1][31]; 
   Md[248][51] = Cd[1][24]; Md[248][161] = Cd[1][5]; Md[248][248] = Cd[1][15]; Md[248][282] = Cd[1][30]; Md[248][338] = Cd[1][6]; Md[248][341] = Cd[1][9]; Md[248][347] = Cd[1][16]; Md[248][350] = Cd[1][19]; Md[248][353] = Cd[1][25]; Md[248][391] = Cd[1][33]; Md[248][452] = Cd[1][7]; Md[248][455] = Cd[1][10]; Md[248][456] = Cd[1][12]; Md[248][462] = Cd[1][17]; Md[248][465] = Cd[1][20]; Md[248][466] = Cd[1][22]; Md[248][469] = Cd[1][26]; Md[248][470] = Cd[1][28]; Md[248][471] = Cd[1][31]; 
   Md[249][187] = Cd[1][24]; Md[249][188] = Cd[1][30]; Md[249][189] = Cd[1][33]; Md[249][240] = Cd[1][5]; Md[249][241] = Cd[1][6]; Md[249][242] = Cd[1][7]; Md[249][243] = Cd[1][9]; Md[249][244] = Cd[1][10]; Md[249][245] = Cd[1][12]; Md[249][249] = Cd[1][15]; Md[249][250] = Cd[1][16]; Md[249][251] = Cd[1][17]; Md[249][252] = Cd[1][19]; Md[249][253] = Cd[1][20]; Md[249][254] = Cd[1][22]; Md[249][255] = Cd[1][26]; Md[249][256] = Cd[1][28]; Md[249][354] = Cd[1][25]; Md[249][474] = Cd[1][31]; 
   Md[250][188] = Cd[1][24]; Md[250][190] = Cd[1][30]; Md[250][191] = Cd[1][33]; Md[250][241] = Cd[1][5]; Md[250][243] = Cd[1][6]; Md[250][244] = Cd[1][7]; Md[250][246] = Cd[1][10]; Md[250][247] = Cd[1][12]; Md[250][250] = Cd[1][15]; Md[250][252] = Cd[1][16]; Md[250][253] = Cd[1][17]; Md[250][255] = Cd[1][20]; Md[250][256] = Cd[1][22]; Md[250][257] = Cd[1][28]; Md[250][342] = Cd[1][9]; Md[250][354] = Cd[1][19]; Md[250][355] = Cd[1][25]; Md[250][356] = Cd[1][26]; Md[250][475] = Cd[1][31]; 
   Md[251][189] = Cd[1][24]; Md[251][191] = Cd[1][30]; Md[251][192] = Cd[1][33]; Md[251][242] = Cd[1][5]; Md[251][244] = Cd[1][6]; Md[251][245] = Cd[1][7]; Md[251][246] = Cd[1][9]; Md[251][247] = Cd[1][10]; Md[251][251] = Cd[1][15]; Md[251][253] = Cd[1][16]; Md[251][254] = Cd[1][17]; Md[251][255] = Cd[1][19]; Md[251][256] = Cd[1][20]; Md[251][257] = Cd[1][26]; Md[251][356] = Cd[1][25]; Md[251][459] = Cd[1][12]; Md[251][474] = Cd[1][22]; Md[251][475] = Cd[1][28]; Md[251][476] = Cd[1][31]; 
   Md[252][190] = Cd[1][24]; Md[252][193] = Cd[1][33]; Md[252][243] = Cd[1][5]; Md[252][246] = Cd[1][7]; Md[252][248] = Cd[1][12]; Md[252][252] = Cd[1][15]; Md[252][255] = Cd[1][17]; Md[252][257] = Cd[1][22]; Md[252][283] = Cd[1][30]; Md[252][342] = Cd[1][6]; Md[252][343] = Cd[1][9]; Md[252][344] = Cd[1][10]; Md[252][354] = Cd[1][16]; Md[252][355] = Cd[1][19]; Md[252][356] = Cd[1][20]; Md[252][357] = Cd[1][25]; Md[252][358] = Cd[1][26]; Md[252][359] = Cd[1][28]; Md[252][477] = Cd[1][31]; 
   Md[253][191] = Cd[1][24]; Md[253][193] = Cd[1][30]; Md[253][194] = Cd[1][33]; Md[253][244] = Cd[1][5]; Md[253][246] = Cd[1][6]; Md[253][247] = Cd[1][7]; Md[253][248] = Cd[1][10]; Md[253][253] = Cd[1][15]; Md[253][255] = Cd[1][16]; Md[253][256] = Cd[1][17]; Md[253][257] = Cd[1][20]; Md[253][344] = Cd[1][9]; Md[253][356] = Cd[1][19]; Md[253][358] = Cd[1][25]; Md[253][359] = Cd[1][26]; Md[253][460] = Cd[1][12]; Md[253][475] = Cd[1][22]; Md[253][477] = Cd[1][28]; Md[253][478] = Cd[1][31]; 
   Md[254][192] = Cd[1][24]; Md[254][194] = Cd[1][30]; Md[254][245] = Cd[1][5]; Md[254][247] = Cd[1][6]; Md[254][248] = Cd[1][9]; Md[254][254] = Cd[1][15]; Md[254][256] = Cd[1][16]; Md[254][257] = Cd[1][19]; Md[254][359] = Cd[1][25]; Md[254][394] = Cd[1][33]; Md[254][459] = Cd[1][7]; Md[254][460] = Cd[1][10]; Md[254][461] = Cd[1][12]; Md[254][474] = Cd[1][17]; Md[254][475] = Cd[1][20]; Md[254][476] = Cd[1][22]; Md[254][477] = Cd[1][26]; Md[254][478] = Cd[1][28]; Md[254][479] = Cd[1][31]; 
   Md[255][52] = Cd[1][33]; Md[255][193] = Cd[1][24]; Md[255][246] = Cd[1][5]; Md[255][248] = Cd[1][7]; Md[255][255] = Cd[1][15]; Md[255][257] = Cd[1][17]; Md[255][285] = Cd[1][30]; Md[255][344] = Cd[1][6]; Md[255][346] = Cd[1][9]; Md[255][347] = Cd[1][10]; Md[255][356] = Cd[1][16]; Md[255][358] = Cd[1][19]; Md[255][359] = Cd[1][20]; Md[255][361] = Cd[1][25]; Md[255][362] = Cd[1][26]; Md[255][462] = Cd[1][12]; Md[255][477] = Cd[1][22]; Md[255][480] = Cd[1][28]; Md[255][481] = Cd[1][31]; 
   Md[256][52] = Cd[1][30]; Md[256][194] = Cd[1][24]; Md[256][247] = Cd[1][5]; Md[256][248] = Cd[1][6]; Md[256][256] = Cd[1][15]; Md[256][257] = Cd[1][16]; Md[256][347] = Cd[1][9]; Md[256][359] = Cd[1][19]; Md[256][362] = Cd[1][25]; Md[256][395] = Cd[1][33]; Md[256][460] = Cd[1][7]; Md[256][462] = Cd[1][10]; Md[256][463] = Cd[1][12]; Md[256][475] = Cd[1][17]; Md[256][477] = Cd[1][20]; Md[256][478] = Cd[1][22]; Md[256][480] = Cd[1][26]; Md[256][481] = Cd[1][28]; Md[256][482] = Cd[1][31]; 
   Md[257][52] = Cd[1][24]; Md[257][248] = Cd[1][5]; Md[257][257] = Cd[1][15]; Md[257][288] = Cd[1][30]; Md[257][347] = Cd[1][6]; Md[257][350] = Cd[1][9]; Md[257][359] = Cd[1][16]; Md[257][362] = Cd[1][19]; Md[257][365] = Cd[1][25]; Md[257][397] = Cd[1][33]; Md[257][462] = Cd[1][7]; Md[257][465] = Cd[1][10]; Md[257][466] = Cd[1][12]; Md[257][477] = Cd[1][17]; Md[257][480] = Cd[1][20]; Md[257][481] = Cd[1][22]; Md[257][484] = Cd[1][26]; Md[257][485] = Cd[1][28]; Md[257][486] = Cd[1][31]; 
   Md[258][1] = Cd[2][14]; Md[258][2] = Cd[2][24]; Md[258][3] = Cd[2][30]; Md[258][4] = Cd[2][33]; Md[258][14] = Cd[2][5]; Md[258][15] = Cd[2][6]; Md[258][16] = Cd[2][7]; Md[258][17] = Cd[2][9]; Md[258][18] = Cd[2][10]; Md[258][19] = Cd[2][12]; Md[258][20] = Cd[2][16]; Md[258][21] = Cd[2][17]; Md[258][22] = Cd[2][19]; Md[258][23] = Cd[2][20]; Md[258][24] = Cd[2][22]; Md[258][25] = Cd[2][26]; Md[258][26] = Cd[2][28]; Md[258][56] = Cd[2][1]; Md[258][57] = Cd[2][2]; Md[258][58] = Cd[2][3]; Md[258][162] = Cd[2][15]; Md[258][258] = Cd[2][25]; Md[258][369] = Cd[2][31]; 
   Md[259][5] = Cd[2][24]; Md[259][6] = Cd[2][30]; Md[259][7] = Cd[2][33]; Md[259][27] = Cd[2][16]; Md[259][28] = Cd[2][17]; Md[259][29] = Cd[2][19]; Md[259][30] = Cd[2][20]; Md[259][31] = Cd[2][22]; Md[259][32] = Cd[2][26]; Md[259][33] = Cd[2][28]; Md[259][54] = Cd[2][14]; Md[259][60] = Cd[2][1]; Md[259][61] = Cd[2][2]; Md[259][62] = Cd[2][3]; Md[259][63] = Cd[2][5]; Md[259][64] = Cd[2][6]; Md[259][65] = Cd[2][7]; Md[259][66] = Cd[2][9]; Md[259][67] = Cd[2][10]; Md[259][68] = Cd[2][12]; Md[259][163] = Cd[2][15]; Md[259][259] = Cd[2][25]; Md[259][370] = Cd[2][31]; 
   Md[260][5] = Cd[2][14]; Md[260][8] = Cd[2][24]; Md[260][9] = Cd[2][30]; Md[260][10] = Cd[2][33]; Md[260][27] = Cd[2][6]; Md[260][28] = Cd[2][7]; Md[260][29] = Cd[2][9]; Md[260][30] = Cd[2][10]; Md[260][31] = Cd[2][12]; Md[260][34] = Cd[2][19]; Md[260][35] = Cd[2][20]; Md[260][36] = Cd[2][22]; Md[260][37] = Cd[2][26]; Md[260][38] = Cd[2][28]; Md[260][63] = Cd[2][1]; Md[260][64] = Cd[2][2]; Md[260][65] = Cd[2][3]; Md[260][163] = Cd[2][5]; Md[260][164] = Cd[2][15]; Md[260][165] = Cd[2][16]; Md[260][166] = Cd[2][17]; Md[260][260] = Cd[2][25]; Md[260][371] = Cd[2][31]; 
   Md[261][6] = Cd[2][14]; Md[261][9] = Cd[2][24]; Md[261][11] = Cd[2][30]; Md[261][12] = Cd[2][33]; Md[261][27] = Cd[2][5]; Md[261][29] = Cd[2][6]; Md[261][30] = Cd[2][7]; Md[261][32] = Cd[2][10]; Md[261][33] = Cd[2][12]; Md[261][34] = Cd[2][16]; Md[261][35] = Cd[2][17]; Md[261][37] = Cd[2][20]; Md[261][38] = Cd[2][22]; Md[261][39] = Cd[2][28]; Md[261][64] = Cd[2][1]; Md[261][66] = Cd[2][2]; Md[261][67] = Cd[2][3]; Md[261][165] = Cd[2][15]; Md[261][259] = Cd[2][9]; Md[261][260] = Cd[2][19]; Md[261][261] = Cd[2][25]; Md[261][262] = Cd[2][26]; Md[261][372] = Cd[2][31]; 
   Md[262][7] = Cd[2][14]; Md[262][10] = Cd[2][24]; Md[262][12] = Cd[2][30]; Md[262][13] = Cd[2][33]; Md[262][28] = Cd[2][5]; Md[262][30] = Cd[2][6]; Md[262][31] = Cd[2][7]; Md[262][32] = Cd[2][9]; Md[262][33] = Cd[2][10]; Md[262][35] = Cd[2][16]; Md[262][36] = Cd[2][17]; Md[262][37] = Cd[2][19]; Md[262][38] = Cd[2][20]; Md[262][39] = Cd[2][26]; Md[262][65] = Cd[2][1]; Md[262][67] = Cd[2][2]; Md[262][68] = Cd[2][3]; Md[262][166] = Cd[2][15]; Md[262][262] = Cd[2][25]; Md[262][370] = Cd[2][12]; Md[262][371] = Cd[2][22]; Md[262][372] = Cd[2][28]; Md[262][373] = Cd[2][31]; 
   Md[263][55] = Cd[2][14]; Md[263][56] = Cd[2][24]; Md[263][57] = Cd[2][30]; Md[263][58] = Cd[2][33]; Md[263][70] = Cd[2][1]; Md[263][71] = Cd[2][2]; Md[263][72] = Cd[2][3]; Md[263][73] = Cd[2][5]; Md[263][74] = Cd[2][6]; Md[263][75] = Cd[2][7]; Md[263][76] = Cd[2][9]; Md[263][77] = Cd[2][10]; Md[263][78] = Cd[2][12]; Md[263][79] = Cd[2][16]; Md[263][80] = Cd[2][17]; Md[263][81] = Cd[2][19]; Md[263][82] = Cd[2][20]; Md[263][83] = Cd[2][22]; Md[263][84] = Cd[2][26]; Md[263][85] = Cd[2][28]; Md[263][167] = Cd[2][15]; Md[263][263] = Cd[2][25]; Md[263][374] = Cd[2][31]; 
   Md[264][14] = Cd[2][24]; Md[264][15] = Cd[2][30]; Md[264][16] = Cd[2][33]; Md[264][40] = Cd[2][19]; Md[264][41] = Cd[2][20]; Md[264][42] = Cd[2][22]; Md[264][43] = Cd[2][26]; Md[264][44] = Cd[2][28]; Md[264][56] = Cd[2][14]; Md[264][73] = Cd[2][1]; Md[264][74] = Cd[2][2]; Md[264][75] = Cd[2][3]; Md[264][79] = Cd[2][6]; Md[264][80] = Cd[2][7]; Md[264][81] = Cd[2][9]; Md[264][82] = Cd[2][10]; Md[264][83] = Cd[2][12]; Md[264][167] = Cd[2][5]; Md[264][168] = Cd[2][15]; Md[264][169] = Cd[2][16]; Md[264][170] = Cd[2][17]; Md[264][264] = Cd[2][25]; Md[264][375] = Cd[2][31]; 
   Md[265][15] = Cd[2][24]; Md[265][17] = Cd[2][30]; Md[265][18] = Cd[2][33]; Md[265][40] = Cd[2][16]; Md[265][41] = Cd[2][17]; Md[265][43] = Cd[2][20]; Md[265][44] = Cd[2][22]; Md[265][45] = Cd[2][28]; Md[265][57] = Cd[2][14]; Md[265][74] = Cd[2][1]; Md[265][76] = Cd[2][2]; Md[265][77] = Cd[2][3]; Md[265][79] = Cd[2][5]; Md[265][81] = Cd[2][6]; Md[265][82] = Cd[2][7]; Md[265][84] = Cd[2][10]; Md[265][85] = Cd[2][12]; Md[265][169] = Cd[2][15]; Md[265][263] = Cd[2][9]; Md[265][264] = Cd[2][19]; Md[265][265] = Cd[2][25]; Md[265][266] = Cd[2][26]; Md[265][376] = Cd[2][31]; 
   Md[266][16] = Cd[2][24]; Md[266][18] = Cd[2][30]; Md[266][19] = Cd[2][33]; Md[266][41] = Cd[2][16]; Md[266][42] = Cd[2][17]; Md[266][43] = Cd[2][19]; Md[266][44] = Cd[2][20]; Md[266][45] = Cd[2][26]; Md[266][58] = Cd[2][14]; Md[266][75] = Cd[2][1]; Md[266][77] = Cd[2][2]; Md[266][78] = Cd[2][3]; Md[266][80] = Cd[2][5]; Md[266][82] = Cd[2][6]; Md[266][83] = Cd[2][7]; Md[266][84] = Cd[2][9]; Md[266][85] = Cd[2][10]; Md[266][170] = Cd[2][15]; Md[266][266] = Cd[2][25]; Md[266][374] = Cd[2][12]; Md[266][375] = Cd[2][22]; Md[266][376] = Cd[2][28]; Md[266][377] = Cd[2][31]; 
   Md[267][14] = Cd[2][14]; Md[267][20] = Cd[2][30]; Md[267][21] = Cd[2][33]; Md[267][40] = Cd[2][9]; Md[267][41] = Cd[2][10]; Md[267][42] = Cd[2][12]; Md[267][46] = Cd[2][26]; Md[267][47] = Cd[2][28]; Md[267][79] = Cd[2][2]; Md[267][80] = Cd[2][3]; Md[267][162] = Cd[2][24]; Md[267][167] = Cd[2][1]; Md[267][168] = Cd[2][5]; Md[267][169] = Cd[2][6]; Md[267][170] = Cd[2][7]; Md[267][171] = Cd[2][15]; Md[267][172] = Cd[2][16]; Md[267][173] = Cd[2][17]; Md[267][174] = Cd[2][19]; Md[267][175] = Cd[2][20]; Md[267][176] = Cd[2][22]; Md[267][267] = Cd[2][25]; Md[267][378] = Cd[2][31]; 
   Md[268][15] = Cd[2][14]; Md[268][20] = Cd[2][24]; Md[268][22] = Cd[2][30]; Md[268][23] = Cd[2][33]; Md[268][40] = Cd[2][6]; Md[268][41] = Cd[2][7]; Md[268][43] = Cd[2][10]; Md[268][44] = Cd[2][12]; Md[268][46] = Cd[2][20]; Md[268][47] = Cd[2][22]; Md[268][48] = Cd[2][28]; Md[268][79] = Cd[2][1]; Md[268][81] = Cd[2][2]; Md[268][82] = Cd[2][3]; Md[268][169] = Cd[2][5]; Md[268][172] = Cd[2][15]; Md[268][174] = Cd[2][16]; Md[268][175] = Cd[2][17]; Md[268][264] = Cd[2][9]; Md[268][267] = Cd[2][19]; Md[268][268] = Cd[2][25]; Md[268][269] = Cd[2][26]; Md[268][379] = Cd[2][31]; 
   Md[269][16] = Cd[2][14]; Md[269][21] = Cd[2][24]; Md[269][23] = Cd[2][30]; Md[269][24] = Cd[2][33]; Md[269][41] = Cd[2][6]; Md[269][42] = Cd[2][7]; Md[269][43] = Cd[2][9]; Md[269][44] = Cd[2][10]; Md[269][46] = Cd[2][19]; Md[269][47] = Cd[2][20]; Md[269][48] = Cd[2][26]; Md[269][80] = Cd[2][1]; Md[269][82] = Cd[2][2]; Md[269][83] = Cd[2][3]; Md[269][170] = Cd[2][5]; Md[269][173] = Cd[2][15]; Md[269][175] = Cd[2][16]; Md[269][176] = Cd[2][17]; Md[269][269] = Cd[2][25]; Md[269][375] = Cd[2][12]; Md[269][378] = Cd[2][22]; Md[269][379] = Cd[2][28]; Md[269][380] = Cd[2][31]; 
   Md[270][17] = Cd[2][14]; Md[270][22] = Cd[2][24]; Md[270][25] = Cd[2][33]; Md[270][40] = Cd[2][5]; Md[270][43] = Cd[2][7]; Md[270][45] = Cd[2][12]; Md[270][46] = Cd[2][17]; Md[270][48] = Cd[2][22]; Md[270][81] = Cd[2][1]; Md[270][84] = Cd[2][3]; Md[270][174] = Cd[2][15]; Md[270][258] = Cd[2][30]; Md[270][263] = Cd[2][2]; Md[270][264] = Cd[2][6]; Md[270][265] = Cd[2][9]; Md[270][266] = Cd[2][10]; Md[270][267] = Cd[2][16]; Md[270][268] = Cd[2][19]; Md[270][269] = Cd[2][20]; Md[270][270] = Cd[2][25]; Md[270][271] = Cd[2][26]; Md[270][272] = Cd[2][28]; Md[270][381] = Cd[2][31]; 
   Md[271][18] = Cd[2][14]; Md[271][23] = Cd[2][24]; Md[271][25] = Cd[2][30]; Md[271][26] = Cd[2][33]; Md[271][41] = Cd[2][5]; Md[271][43] = Cd[2][6]; Md[271][44] = Cd[2][7]; Md[271][45] = Cd[2][10]; Md[271][46] = Cd[2][16]; Md[271][47] = Cd[2][17]; Md[271][48] = Cd[2][20]; Md[271][82] = Cd[2][1]; Md[271][84] = Cd[2][2]; Md[271][85] = Cd[2][3]; Md[271][175] = Cd[2][15]; Md[271][266] = Cd[2][9]; Md[271][269] = Cd[2][19]; Md[271][271] = Cd[2][25]; Md[271][272] = Cd[2][26]; Md[271][376] = Cd[2][12]; Md[271][379] = Cd[2][22]; Md[271][381] = Cd[2][28]; Md[271][382] = Cd[2][31]; 
   Md[272][19] = Cd[2][14]; Md[272][24] = Cd[2][24]; Md[272][26] = Cd[2][30]; Md[272][42] = Cd[2][5]; Md[272][44] = Cd[2][6]; Md[272][45] = Cd[2][9]; Md[272][47] = Cd[2][16]; Md[272][48] = Cd[2][19]; Md[272][83] = Cd[2][1]; Md[272][85] = Cd[2][2]; Md[272][176] = Cd[2][15]; Md[272][272] = Cd[2][25]; Md[272][369] = Cd[2][33]; Md[272][374] = Cd[2][3]; Md[272][375] = Cd[2][7]; Md[272][376] = Cd[2][10]; Md[272][377] = Cd[2][12]; Md[272][378] = Cd[2][17]; Md[272][379] = Cd[2][20]; Md[272][380] = Cd[2][22]; Md[272][381] = Cd[2][26]; Md[272][382] = Cd[2][28]; Md[272][383] = Cd[2][31]; 
   Md[273][59] = Cd[2][14]; Md[273][60] = Cd[2][24]; Md[273][61] = Cd[2][30]; Md[273][62] = Cd[2][33]; Md[273][87] = Cd[2][1]; Md[273][88] = Cd[2][2]; Md[273][89] = Cd[2][3]; Md[273][90] = Cd[2][5]; Md[273][91] = Cd[2][6]; Md[273][92] = Cd[2][7]; Md[273][93] = Cd[2][9]; Md[273][94] = Cd[2][10]; Md[273][95] = Cd[2][12]; Md[273][96] = Cd[2][16]; Md[273][97] = Cd[2][17]; Md[273][98] = Cd[2][19]; Md[273][99] = Cd[2][20]; Md[273][100] = Cd[2][22]; Md[273][101] = Cd[2][26]; Md[273][102] = Cd[2][28]; Md[273][177] = Cd[2][15]; Md[273][273] = Cd[2][25]; Md[273][384] = Cd[2][31]; 
   Md[274][60] = Cd[2][14]; Md[274][63] = Cd[2][24]; Md[274][64] = Cd[2][30]; Md[274][65] = Cd[2][33]; Md[274][90] = Cd[2][1]; Md[274][91] = Cd[2][2]; Md[274][92] = Cd[2][3]; Md[274][96] = Cd[2][6]; Md[274][97] = Cd[2][7]; Md[274][98] = Cd[2][9]; Md[274][99] = Cd[2][10]; Md[274][100] = Cd[2][12]; Md[274][103] = Cd[2][19]; Md[274][104] = Cd[2][20]; Md[274][105] = Cd[2][22]; Md[274][106] = Cd[2][26]; Md[274][107] = Cd[2][28]; Md[274][177] = Cd[2][5]; Md[274][178] = Cd[2][15]; Md[274][179] = Cd[2][16]; Md[274][180] = Cd[2][17]; Md[274][274] = Cd[2][25]; Md[274][385] = Cd[2][31]; 
   Md[275][61] = Cd[2][14]; Md[275][64] = Cd[2][24]; Md[275][66] = Cd[2][30]; Md[275][67] = Cd[2][33]; Md[275][91] = Cd[2][1]; Md[275][93] = Cd[2][2]; Md[275][94] = Cd[2][3]; Md[275][96] = Cd[2][5]; Md[275][98] = Cd[2][6]; Md[275][99] = Cd[2][7]; Md[275][101] = Cd[2][10]; Md[275][102] = Cd[2][12]; Md[275][103] = Cd[2][16]; Md[275][104] = Cd[2][17]; Md[275][106] = Cd[2][20]; Md[275][107] = Cd[2][22]; Md[275][108] = Cd[2][28]; Md[275][179] = Cd[2][15]; Md[275][273] = Cd[2][9]; Md[275][274] = Cd[2][19]; Md[275][275] = Cd[2][25]; Md[275][276] = Cd[2][26]; Md[275][386] = Cd[2][31]; 
   Md[276][62] = Cd[2][14]; Md[276][65] = Cd[2][24]; Md[276][67] = Cd[2][30]; Md[276][68] = Cd[2][33]; Md[276][92] = Cd[2][1]; Md[276][94] = Cd[2][2]; Md[276][95] = Cd[2][3]; Md[276][97] = Cd[2][5]; Md[276][99] = Cd[2][6]; Md[276][100] = Cd[2][7]; Md[276][101] = Cd[2][9]; Md[276][102] = Cd[2][10]; Md[276][104] = Cd[2][16]; Md[276][105] = Cd[2][17]; Md[276][106] = Cd[2][19]; Md[276][107] = Cd[2][20]; Md[276][108] = Cd[2][26]; Md[276][180] = Cd[2][15]; Md[276][276] = Cd[2][25]; Md[276][384] = Cd[2][12]; Md[276][385] = Cd[2][22]; Md[276][386] = Cd[2][28]; Md[276][387] = Cd[2][31]; 
   Md[277][27] = Cd[2][30]; Md[277][28] = Cd[2][33]; Md[277][49] = Cd[2][26]; Md[277][50] = Cd[2][28]; Md[277][63] = Cd[2][14]; Md[277][96] = Cd[2][2]; Md[277][97] = Cd[2][3]; Md[277][103] = Cd[2][9]; Md[277][104] = Cd[2][10]; Md[277][105] = Cd[2][12]; Md[277][163] = Cd[2][24]; Md[277][177] = Cd[2][1]; Md[277][178] = Cd[2][5]; Md[277][179] = Cd[2][6]; Md[277][180] = Cd[2][7]; Md[277][181] = Cd[2][15]; Md[277][182] = Cd[2][16]; Md[277][183] = Cd[2][17]; Md[277][184] = Cd[2][19]; Md[277][185] = Cd[2][20]; Md[277][186] = Cd[2][22]; Md[277][277] = Cd[2][25]; Md[277][388] = Cd[2][31]; 
   Md[278][27] = Cd[2][24]; Md[278][29] = Cd[2][30]; Md[278][30] = Cd[2][33]; Md[278][49] = Cd[2][20]; Md[278][50] = Cd[2][22]; Md[278][51] = Cd[2][28]; Md[278][64] = Cd[2][14]; Md[278][96] = Cd[2][1]; Md[278][98] = Cd[2][2]; Md[278][99] = Cd[2][3]; Md[278][103] = Cd[2][6]; Md[278][104] = Cd[2][7]; Md[278][106] = Cd[2][10]; Md[278][107] = Cd[2][12]; Md[278][179] = Cd[2][5]; Md[278][182] = Cd[2][15]; Md[278][184] = Cd[2][16]; Md[278][185] = Cd[2][17]; Md[278][274] = Cd[2][9]; Md[278][277] = Cd[2][19]; Md[278][278] = Cd[2][25]; Md[278][279] = Cd[2][26]; Md[278][389] = Cd[2][31]; 
   Md[279][28] = Cd[2][24]; Md[279][30] = Cd[2][30]; Md[279][31] = Cd[2][33]; Md[279][49] = Cd[2][19]; Md[279][50] = Cd[2][20]; Md[279][51] = Cd[2][26]; Md[279][65] = Cd[2][14]; Md[279][97] = Cd[2][1]; Md[279][99] = Cd[2][2]; Md[279][100] = Cd[2][3]; Md[279][104] = Cd[2][6]; Md[279][105] = Cd[2][7]; Md[279][106] = Cd[2][9]; Md[279][107] = Cd[2][10]; Md[279][180] = Cd[2][5]; Md[279][183] = Cd[2][15]; Md[279][185] = Cd[2][16]; Md[279][186] = Cd[2][17]; Md[279][279] = Cd[2][25]; Md[279][385] = Cd[2][12]; Md[279][388] = Cd[2][22]; Md[279][389] = Cd[2][28]; Md[279][390] = Cd[2][31]; 
   Md[280][29] = Cd[2][24]; Md[280][32] = Cd[2][33]; Md[280][49] = Cd[2][17]; Md[280][51] = Cd[2][22]; Md[280][66] = Cd[2][14]; Md[280][98] = Cd[2][1]; Md[280][101] = Cd[2][3]; Md[280][103] = Cd[2][5]; Md[280][106] = Cd[2][7]; Md[280][108] = Cd[2][12]; Md[280][184] = Cd[2][15]; Md[280][259] = Cd[2][30]; Md[280][273] = Cd[2][2]; Md[280][274] = Cd[2][6]; Md[280][275] = Cd[2][9]; Md[280][276] = Cd[2][10]; Md[280][277] = Cd[2][16]; Md[280][278] = Cd[2][19]; Md[280][279] = Cd[2][20]; Md[280][280] = Cd[2][25]; Md[280][281] = Cd[2][26]; Md[280][282] = Cd[2][28]; Md[280][391] = Cd[2][31]; 
   Md[281][30] = Cd[2][24]; Md[281][32] = Cd[2][30]; Md[281][33] = Cd[2][33]; Md[281][49] = Cd[2][16]; Md[281][50] = Cd[2][17]; Md[281][51] = Cd[2][20]; Md[281][67] = Cd[2][14]; Md[281][99] = Cd[2][1]; Md[281][101] = Cd[2][2]; Md[281][102] = Cd[2][3]; Md[281][104] = Cd[2][5]; Md[281][106] = Cd[2][6]; Md[281][107] = Cd[2][7]; Md[281][108] = Cd[2][10]; Md[281][185] = Cd[2][15]; Md[281][276] = Cd[2][9]; Md[281][279] = Cd[2][19]; Md[281][281] = Cd[2][25]; Md[281][282] = Cd[2][26]; Md[281][386] = Cd[2][12]; Md[281][389] = Cd[2][22]; Md[281][391] = Cd[2][28]; Md[281][392] = Cd[2][31]; 
   Md[282][31] = Cd[2][24]; Md[282][33] = Cd[2][30]; Md[282][50] = Cd[2][16]; Md[282][51] = Cd[2][19]; Md[282][68] = Cd[2][14]; Md[282][100] = Cd[2][1]; Md[282][102] = Cd[2][2]; Md[282][105] = Cd[2][5]; Md[282][107] = Cd[2][6]; Md[282][108] = Cd[2][9]; Md[282][186] = Cd[2][15]; Md[282][282] = Cd[2][25]; Md[282][370] = Cd[2][33]; Md[282][384] = Cd[2][3]; Md[282][385] = Cd[2][7]; Md[282][386] = Cd[2][10]; Md[282][387] = Cd[2][12]; Md[282][388] = Cd[2][17]; Md[282][389] = Cd[2][20]; Md[282][390] = Cd[2][22]; Md[282][391] = Cd[2][26]; Md[282][392] = Cd[2][28]; Md[282][393] = Cd[2][31]; 
   Md[283][163] = Cd[2][14]; Md[283][164] = Cd[2][24]; Md[283][165] = Cd[2][30]; Md[283][166] = Cd[2][33]; Md[283][178] = Cd[2][1]; Md[283][179] = Cd[2][2]; Md[283][180] = Cd[2][3]; Md[283][181] = Cd[2][5]; Md[283][182] = Cd[2][6]; Md[283][183] = Cd[2][7]; Md[283][184] = Cd[2][9]; Md[283][185] = Cd[2][10]; Md[283][186] = Cd[2][12]; Md[283][187] = Cd[2][15]; Md[283][188] = Cd[2][16]; Md[283][189] = Cd[2][17]; Md[283][190] = Cd[2][19]; Md[283][191] = Cd[2][20]; Md[283][192] = Cd[2][22]; Md[283][193] = Cd[2][26]; Md[283][194] = Cd[2][28]; Md[283][283] = Cd[2][25]; Md[283][394] = Cd[2][31]; 
   Md[284][27] = Cd[2][14]; Md[284][34] = Cd[2][30]; Md[284][35] = Cd[2][33]; Md[284][49] = Cd[2][10]; Md[284][50] = Cd[2][12]; Md[284][52] = Cd[2][28]; Md[284][103] = Cd[2][2]; Md[284][104] = Cd[2][3]; Md[284][165] = Cd[2][24]; Md[284][179] = Cd[2][1]; Md[284][182] = Cd[2][5]; Md[284][184] = Cd[2][6]; Md[284][185] = Cd[2][7]; Md[284][188] = Cd[2][15]; Md[284][190] = Cd[2][16]; Md[284][191] = Cd[2][17]; Md[284][193] = Cd[2][20]; Md[284][194] = Cd[2][22]; Md[284][277] = Cd[2][9]; Md[284][283] = Cd[2][19]; Md[284][284] = Cd[2][25]; Md[284][285] = Cd[2][26]; Md[284][395] = Cd[2][31]; 
   Md[285][28] = Cd[2][14]; Md[285][35] = Cd[2][30]; Md[285][36] = Cd[2][33]; Md[285][49] = Cd[2][9]; Md[285][50] = Cd[2][10]; Md[285][52] = Cd[2][26]; Md[285][104] = Cd[2][2]; Md[285][105] = Cd[2][3]; Md[285][166] = Cd[2][24]; Md[285][180] = Cd[2][1]; Md[285][183] = Cd[2][5]; Md[285][185] = Cd[2][6]; Md[285][186] = Cd[2][7]; Md[285][189] = Cd[2][15]; Md[285][191] = Cd[2][16]; Md[285][192] = Cd[2][17]; Md[285][193] = Cd[2][19]; Md[285][194] = Cd[2][20]; Md[285][285] = Cd[2][25]; Md[285][388] = Cd[2][12]; Md[285][394] = Cd[2][22]; Md[285][395] = Cd[2][28]; Md[285][396] = Cd[2][31]; 
   Md[286][29] = Cd[2][14]; Md[286][34] = Cd[2][24]; Md[286][37] = Cd[2][33]; Md[286][49] = Cd[2][7]; Md[286][51] = Cd[2][12]; Md[286][52] = Cd[2][22]; Md[286][103] = Cd[2][1]; Md[286][106] = Cd[2][3]; Md[286][184] = Cd[2][5]; Md[286][190] = Cd[2][15]; Md[286][193] = Cd[2][17]; Md[286][260] = Cd[2][30]; Md[286][274] = Cd[2][2]; Md[286][277] = Cd[2][6]; Md[286][278] = Cd[2][9]; Md[286][279] = Cd[2][10]; Md[286][283] = Cd[2][16]; Md[286][284] = Cd[2][19]; Md[286][285] = Cd[2][20]; Md[286][286] = Cd[2][25]; Md[286][287] = Cd[2][26]; Md[286][288] = Cd[2][28]; Md[286][397] = Cd[2][31]; 
   Md[287][30] = Cd[2][14]; Md[287][35] = Cd[2][24]; Md[287][37] = Cd[2][30]; Md[287][38] = Cd[2][33]; Md[287][49] = Cd[2][6]; Md[287][50] = Cd[2][7]; Md[287][51] = Cd[2][10]; Md[287][52] = Cd[2][20]; Md[287][104] = Cd[2][1]; Md[287][106] = Cd[2][2]; Md[287][107] = Cd[2][3]; Md[287][185] = Cd[2][5]; Md[287][191] = Cd[2][15]; Md[287][193] = Cd[2][16]; Md[287][194] = Cd[2][17]; Md[287][279] = Cd[2][9]; Md[287][285] = Cd[2][19]; Md[287][287] = Cd[2][25]; Md[287][288] = Cd[2][26]; Md[287][389] = Cd[2][12]; Md[287][395] = Cd[2][22]; Md[287][397] = Cd[2][28]; Md[287][398] = Cd[2][31]; 
   Md[288][31] = Cd[2][14]; Md[288][36] = Cd[2][24]; Md[288][38] = Cd[2][30]; Md[288][50] = Cd[2][6]; Md[288][51] = Cd[2][9]; Md[288][52] = Cd[2][19]; Md[288][105] = Cd[2][1]; Md[288][107] = Cd[2][2]; Md[288][186] = Cd[2][5]; Md[288][192] = Cd[2][15]; Md[288][194] = Cd[2][16]; Md[288][288] = Cd[2][25]; Md[288][371] = Cd[2][33]; Md[288][385] = Cd[2][3]; Md[288][388] = Cd[2][7]; Md[288][389] = Cd[2][10]; Md[288][390] = Cd[2][12]; Md[288][394] = Cd[2][17]; Md[288][395] = Cd[2][20]; Md[288][396] = Cd[2][22]; Md[288][397] = Cd[2][26]; Md[288][398] = Cd[2][28]; Md[288][399] = Cd[2][31]; 
   Md[289][259] = Cd[2][14]; Md[289][260] = Cd[2][24]; Md[289][261] = Cd[2][30]; Md[289][262] = Cd[2][33]; Md[289][274] = Cd[2][1]; Md[289][275] = Cd[2][2]; Md[289][276] = Cd[2][3]; Md[289][277] = Cd[2][5]; Md[289][278] = Cd[2][6]; Md[289][279] = Cd[2][7]; Md[289][280] = Cd[2][9]; Md[289][281] = Cd[2][10]; Md[289][282] = Cd[2][12]; Md[289][283] = Cd[2][15]; Md[289][284] = Cd[2][16]; Md[289][285] = Cd[2][17]; Md[289][286] = Cd[2][19]; Md[289][287] = Cd[2][20]; Md[289][288] = Cd[2][22]; Md[289][289] = Cd[2][25]; Md[289][290] = Cd[2][26]; Md[289][291] = Cd[2][28]; Md[289][400] = Cd[2][31]; 
   Md[290][32] = Cd[2][14]; Md[290][37] = Cd[2][24]; Md[290][39] = Cd[2][33]; Md[290][49] = Cd[2][5]; Md[290][51] = Cd[2][7]; Md[290][52] = Cd[2][17]; Md[290][106] = Cd[2][1]; Md[290][108] = Cd[2][3]; Md[290][193] = Cd[2][15]; Md[290][262] = Cd[2][30]; Md[290][276] = Cd[2][2]; Md[290][279] = Cd[2][6]; Md[290][281] = Cd[2][9]; Md[290][282] = Cd[2][10]; Md[290][285] = Cd[2][16]; Md[290][287] = Cd[2][19]; Md[290][288] = Cd[2][20]; Md[290][290] = Cd[2][25]; Md[290][291] = Cd[2][26]; Md[290][391] = Cd[2][12]; Md[290][397] = Cd[2][22]; Md[290][400] = Cd[2][28]; Md[290][401] = Cd[2][31]; 
   Md[291][33] = Cd[2][14]; Md[291][38] = Cd[2][24]; Md[291][39] = Cd[2][30]; Md[291][50] = Cd[2][5]; Md[291][51] = Cd[2][6]; Md[291][52] = Cd[2][16]; Md[291][107] = Cd[2][1]; Md[291][108] = Cd[2][2]; Md[291][194] = Cd[2][15]; Md[291][282] = Cd[2][9]; Md[291][288] = Cd[2][19]; Md[291][291] = Cd[2][25]; Md[291][372] = Cd[2][33]; Md[291][386] = Cd[2][3]; Md[291][389] = Cd[2][7]; Md[291][391] = Cd[2][10]; Md[291][392] = Cd[2][12]; Md[291][395] = Cd[2][17]; Md[291][397] = Cd[2][20]; Md[291][398] = Cd[2][22]; Md[291][400] = Cd[2][26]; Md[291][401] = Cd[2][28]; Md[291][402] = Cd[2][31]; 
   Md[292][69] = Cd[2][14]; Md[292][70] = Cd[2][24]; Md[292][71] = Cd[2][30]; Md[292][72] = Cd[2][33]; Md[292][110] = Cd[2][1]; Md[292][111] = Cd[2][2]; Md[292][112] = Cd[2][3]; Md[292][113] = Cd[2][5]; Md[292][114] = Cd[2][6]; Md[292][115] = Cd[2][7]; Md[292][116] = Cd[2][9]; Md[292][117] = Cd[2][10]; Md[292][118] = Cd[2][12]; Md[292][119] = Cd[2][16]; Md[292][120] = Cd[2][17]; Md[292][121] = Cd[2][19]; Md[292][122] = Cd[2][20]; Md[292][123] = Cd[2][22]; Md[292][124] = Cd[2][26]; Md[292][125] = Cd[2][28]; Md[292][195] = Cd[2][15]; Md[292][292] = Cd[2][25]; Md[292][404] = Cd[2][31]; 
   Md[293][70] = Cd[2][14]; Md[293][73] = Cd[2][24]; Md[293][74] = Cd[2][30]; Md[293][75] = Cd[2][33]; Md[293][113] = Cd[2][1]; Md[293][114] = Cd[2][2]; Md[293][115] = Cd[2][3]; Md[293][119] = Cd[2][6]; Md[293][120] = Cd[2][7]; Md[293][121] = Cd[2][9]; Md[293][122] = Cd[2][10]; Md[293][123] = Cd[2][12]; Md[293][126] = Cd[2][19]; Md[293][127] = Cd[2][20]; Md[293][128] = Cd[2][22]; Md[293][129] = Cd[2][26]; Md[293][130] = Cd[2][28]; Md[293][195] = Cd[2][5]; Md[293][196] = Cd[2][15]; Md[293][197] = Cd[2][16]; Md[293][198] = Cd[2][17]; Md[293][293] = Cd[2][25]; Md[293][405] = Cd[2][31]; 
   Md[294][71] = Cd[2][14]; Md[294][74] = Cd[2][24]; Md[294][76] = Cd[2][30]; Md[294][77] = Cd[2][33]; Md[294][114] = Cd[2][1]; Md[294][116] = Cd[2][2]; Md[294][117] = Cd[2][3]; Md[294][119] = Cd[2][5]; Md[294][121] = Cd[2][6]; Md[294][122] = Cd[2][7]; Md[294][124] = Cd[2][10]; Md[294][125] = Cd[2][12]; Md[294][126] = Cd[2][16]; Md[294][127] = Cd[2][17]; Md[294][129] = Cd[2][20]; Md[294][130] = Cd[2][22]; Md[294][131] = Cd[2][28]; Md[294][197] = Cd[2][15]; Md[294][292] = Cd[2][9]; Md[294][293] = Cd[2][19]; Md[294][294] = Cd[2][25]; Md[294][295] = Cd[2][26]; Md[294][406] = Cd[2][31]; 
   Md[295][72] = Cd[2][14]; Md[295][75] = Cd[2][24]; Md[295][77] = Cd[2][30]; Md[295][78] = Cd[2][33]; Md[295][115] = Cd[2][1]; Md[295][117] = Cd[2][2]; Md[295][118] = Cd[2][3]; Md[295][120] = Cd[2][5]; Md[295][122] = Cd[2][6]; Md[295][123] = Cd[2][7]; Md[295][124] = Cd[2][9]; Md[295][125] = Cd[2][10]; Md[295][127] = Cd[2][16]; Md[295][128] = Cd[2][17]; Md[295][129] = Cd[2][19]; Md[295][130] = Cd[2][20]; Md[295][131] = Cd[2][26]; Md[295][198] = Cd[2][15]; Md[295][295] = Cd[2][25]; Md[295][404] = Cd[2][12]; Md[295][405] = Cd[2][22]; Md[295][406] = Cd[2][28]; Md[295][407] = Cd[2][31]; 
   Md[296][73] = Cd[2][14]; Md[296][79] = Cd[2][30]; Md[296][80] = Cd[2][33]; Md[296][119] = Cd[2][2]; Md[296][120] = Cd[2][3]; Md[296][126] = Cd[2][9]; Md[296][127] = Cd[2][10]; Md[296][128] = Cd[2][12]; Md[296][132] = Cd[2][26]; Md[296][133] = Cd[2][28]; Md[296][167] = Cd[2][24]; Md[296][195] = Cd[2][1]; Md[296][196] = Cd[2][5]; Md[296][197] = Cd[2][6]; Md[296][198] = Cd[2][7]; Md[296][199] = Cd[2][15]; Md[296][200] = Cd[2][16]; Md[296][201] = Cd[2][17]; Md[296][202] = Cd[2][19]; Md[296][203] = Cd[2][20]; Md[296][204] = Cd[2][22]; Md[296][296] = Cd[2][25]; Md[296][408] = Cd[2][31]; 
   Md[297][74] = Cd[2][14]; Md[297][79] = Cd[2][24]; Md[297][81] = Cd[2][30]; Md[297][82] = Cd[2][33]; Md[297][119] = Cd[2][1]; Md[297][121] = Cd[2][2]; Md[297][122] = Cd[2][3]; Md[297][126] = Cd[2][6]; Md[297][127] = Cd[2][7]; Md[297][129] = Cd[2][10]; Md[297][130] = Cd[2][12]; Md[297][132] = Cd[2][20]; Md[297][133] = Cd[2][22]; Md[297][134] = Cd[2][28]; Md[297][197] = Cd[2][5]; Md[297][200] = Cd[2][15]; Md[297][202] = Cd[2][16]; Md[297][203] = Cd[2][17]; Md[297][293] = Cd[2][9]; Md[297][296] = Cd[2][19]; Md[297][297] = Cd[2][25]; Md[297][298] = Cd[2][26]; Md[297][409] = Cd[2][31]; 
   Md[298][75] = Cd[2][14]; Md[298][80] = Cd[2][24]; Md[298][82] = Cd[2][30]; Md[298][83] = Cd[2][33]; Md[298][120] = Cd[2][1]; Md[298][122] = Cd[2][2]; Md[298][123] = Cd[2][3]; Md[298][127] = Cd[2][6]; Md[298][128] = Cd[2][7]; Md[298][129] = Cd[2][9]; Md[298][130] = Cd[2][10]; Md[298][132] = Cd[2][19]; Md[298][133] = Cd[2][20]; Md[298][134] = Cd[2][26]; Md[298][198] = Cd[2][5]; Md[298][201] = Cd[2][15]; Md[298][203] = Cd[2][16]; Md[298][204] = Cd[2][17]; Md[298][298] = Cd[2][25]; Md[298][405] = Cd[2][12]; Md[298][408] = Cd[2][22]; Md[298][409] = Cd[2][28]; Md[298][410] = Cd[2][31]; 
   Md[299][76] = Cd[2][14]; Md[299][81] = Cd[2][24]; Md[299][84] = Cd[2][33]; Md[299][121] = Cd[2][1]; Md[299][124] = Cd[2][3]; Md[299][126] = Cd[2][5]; Md[299][129] = Cd[2][7]; Md[299][131] = Cd[2][12]; Md[299][132] = Cd[2][17]; Md[299][134] = Cd[2][22]; Md[299][202] = Cd[2][15]; Md[299][263] = Cd[2][30]; Md[299][292] = Cd[2][2]; Md[299][293] = Cd[2][6]; Md[299][294] = Cd[2][9]; Md[299][295] = Cd[2][10]; Md[299][296] = Cd[2][16]; Md[299][297] = Cd[2][19]; Md[299][298] = Cd[2][20]; Md[299][299] = Cd[2][25]; Md[299][300] = Cd[2][26]; Md[299][301] = Cd[2][28]; Md[299][411] = Cd[2][31]; 
   Md[300][77] = Cd[2][14]; Md[300][82] = Cd[2][24]; Md[300][84] = Cd[2][30]; Md[300][85] = Cd[2][33]; Md[300][122] = Cd[2][1]; Md[300][124] = Cd[2][2]; Md[300][125] = Cd[2][3]; Md[300][127] = Cd[2][5]; Md[300][129] = Cd[2][6]; Md[300][130] = Cd[2][7]; Md[300][131] = Cd[2][10]; Md[300][132] = Cd[2][16]; Md[300][133] = Cd[2][17]; Md[300][134] = Cd[2][20]; Md[300][203] = Cd[2][15]; Md[300][295] = Cd[2][9]; Md[300][298] = Cd[2][19]; Md[300][300] = Cd[2][25]; Md[300][301] = Cd[2][26]; Md[300][406] = Cd[2][12]; Md[300][409] = Cd[2][22]; Md[300][411] = Cd[2][28]; Md[300][412] = Cd[2][31]; 
   Md[301][78] = Cd[2][14]; Md[301][83] = Cd[2][24]; Md[301][85] = Cd[2][30]; Md[301][123] = Cd[2][1]; Md[301][125] = Cd[2][2]; Md[301][128] = Cd[2][5]; Md[301][130] = Cd[2][6]; Md[301][131] = Cd[2][9]; Md[301][133] = Cd[2][16]; Md[301][134] = Cd[2][19]; Md[301][204] = Cd[2][15]; Md[301][301] = Cd[2][25]; Md[301][374] = Cd[2][33]; Md[301][404] = Cd[2][3]; Md[301][405] = Cd[2][7]; Md[301][406] = Cd[2][10]; Md[301][407] = Cd[2][12]; Md[301][408] = Cd[2][17]; Md[301][409] = Cd[2][20]; Md[301][410] = Cd[2][22]; Md[301][411] = Cd[2][26]; Md[301][412] = Cd[2][28]; Md[301][413] = Cd[2][31]; 
   Md[302][167] = Cd[2][14]; Md[302][168] = Cd[2][24]; Md[302][169] = Cd[2][30]; Md[302][170] = Cd[2][33]; Md[302][196] = Cd[2][1]; Md[302][197] = Cd[2][2]; Md[302][198] = Cd[2][3]; Md[302][199] = Cd[2][5]; Md[302][200] = Cd[2][6]; Md[302][201] = Cd[2][7]; Md[302][202] = Cd[2][9]; Md[302][203] = Cd[2][10]; Md[302][204] = Cd[2][12]; Md[302][205] = Cd[2][15]; Md[302][206] = Cd[2][16]; Md[302][207] = Cd[2][17]; Md[302][208] = Cd[2][19]; Md[302][209] = Cd[2][20]; Md[302][210] = Cd[2][22]; Md[302][211] = Cd[2][26]; Md[302][212] = Cd[2][28]; Md[302][302] = Cd[2][25]; Md[302][414] = Cd[2][31]; 
   Md[303][40] = Cd[2][30]; Md[303][41] = Cd[2][33]; Md[303][53] = Cd[2][28]; Md[303][79] = Cd[2][14]; Md[303][126] = Cd[2][2]; Md[303][127] = Cd[2][3]; Md[303][132] = Cd[2][10]; Md[303][133] = Cd[2][12]; Md[303][169] = Cd[2][24]; Md[303][197] = Cd[2][1]; Md[303][200] = Cd[2][5]; Md[303][202] = Cd[2][6]; Md[303][203] = Cd[2][7]; Md[303][206] = Cd[2][15]; Md[303][208] = Cd[2][16]; Md[303][209] = Cd[2][17]; Md[303][211] = Cd[2][20]; Md[303][212] = Cd[2][22]; Md[303][296] = Cd[2][9]; Md[303][302] = Cd[2][19]; Md[303][303] = Cd[2][25]; Md[303][304] = Cd[2][26]; Md[303][415] = Cd[2][31]; 
   Md[304][41] = Cd[2][30]; Md[304][42] = Cd[2][33]; Md[304][53] = Cd[2][26]; Md[304][80] = Cd[2][14]; Md[304][127] = Cd[2][2]; Md[304][128] = Cd[2][3]; Md[304][132] = Cd[2][9]; Md[304][133] = Cd[2][10]; Md[304][170] = Cd[2][24]; Md[304][198] = Cd[2][1]; Md[304][201] = Cd[2][5]; Md[304][203] = Cd[2][6]; Md[304][204] = Cd[2][7]; Md[304][207] = Cd[2][15]; Md[304][209] = Cd[2][16]; Md[304][210] = Cd[2][17]; Md[304][211] = Cd[2][19]; Md[304][212] = Cd[2][20]; Md[304][304] = Cd[2][25]; Md[304][408] = Cd[2][12]; Md[304][414] = Cd[2][22]; Md[304][415] = Cd[2][28]; Md[304][416] = Cd[2][31]; 
   Md[305][40] = Cd[2][24]; Md[305][43] = Cd[2][33]; Md[305][53] = Cd[2][22]; Md[305][81] = Cd[2][14]; Md[305][126] = Cd[2][1]; Md[305][129] = Cd[2][3]; Md[305][132] = Cd[2][7]; Md[305][134] = Cd[2][12]; Md[305][202] = Cd[2][5]; Md[305][208] = Cd[2][15]; Md[305][211] = Cd[2][17]; Md[305][264] = Cd[2][30]; Md[305][293] = Cd[2][2]; Md[305][296] = Cd[2][6]; Md[305][297] = Cd[2][9]; Md[305][298] = Cd[2][10]; Md[305][302] = Cd[2][16]; Md[305][303] = Cd[2][19]; Md[305][304] = Cd[2][20]; Md[305][305] = Cd[2][25]; Md[305][306] = Cd[2][26]; Md[305][307] = Cd[2][28]; Md[305][417] = Cd[2][31]; 
   Md[306][41] = Cd[2][24]; Md[306][43] = Cd[2][30]; Md[306][44] = Cd[2][33]; Md[306][53] = Cd[2][20]; Md[306][82] = Cd[2][14]; Md[306][127] = Cd[2][1]; Md[306][129] = Cd[2][2]; Md[306][130] = Cd[2][3]; Md[306][132] = Cd[2][6]; Md[306][133] = Cd[2][7]; Md[306][134] = Cd[2][10]; Md[306][203] = Cd[2][5]; Md[306][209] = Cd[2][15]; Md[306][211] = Cd[2][16]; Md[306][212] = Cd[2][17]; Md[306][298] = Cd[2][9]; Md[306][304] = Cd[2][19]; Md[306][306] = Cd[2][25]; Md[306][307] = Cd[2][26]; Md[306][409] = Cd[2][12]; Md[306][415] = Cd[2][22]; Md[306][417] = Cd[2][28]; Md[306][418] = Cd[2][31]; 
   Md[307][42] = Cd[2][24]; Md[307][44] = Cd[2][30]; Md[307][53] = Cd[2][19]; Md[307][83] = Cd[2][14]; Md[307][128] = Cd[2][1]; Md[307][130] = Cd[2][2]; Md[307][133] = Cd[2][6]; Md[307][134] = Cd[2][9]; Md[307][204] = Cd[2][5]; Md[307][210] = Cd[2][15]; Md[307][212] = Cd[2][16]; Md[307][307] = Cd[2][25]; Md[307][375] = Cd[2][33]; Md[307][405] = Cd[2][3]; Md[307][408] = Cd[2][7]; Md[307][409] = Cd[2][10]; Md[307][410] = Cd[2][12]; Md[307][414] = Cd[2][17]; Md[307][415] = Cd[2][20]; Md[307][416] = Cd[2][22]; Md[307][417] = Cd[2][26]; Md[307][418] = Cd[2][28]; Md[307][419] = Cd[2][31]; 
   Md[308][263] = Cd[2][14]; Md[308][264] = Cd[2][24]; Md[308][265] = Cd[2][30]; Md[308][266] = Cd[2][33]; Md[308][293] = Cd[2][1]; Md[308][294] = Cd[2][2]; Md[308][295] = Cd[2][3]; Md[308][296] = Cd[2][5]; Md[308][297] = Cd[2][6]; Md[308][298] = Cd[2][7]; Md[308][299] = Cd[2][9]; Md[308][300] = Cd[2][10]; Md[308][301] = Cd[2][12]; Md[308][302] = Cd[2][15]; Md[308][303] = Cd[2][16]; Md[308][304] = Cd[2][17]; Md[308][305] = Cd[2][19]; Md[308][306] = Cd[2][20]; Md[308][307] = Cd[2][22]; Md[308][308] = Cd[2][25]; Md[308][309] = Cd[2][26]; Md[308][310] = Cd[2][28]; Md[308][420] = Cd[2][31]; 
   Md[309][43] = Cd[2][24]; Md[309][45] = Cd[2][33]; Md[309][53] = Cd[2][17]; Md[309][84] = Cd[2][14]; Md[309][129] = Cd[2][1]; Md[309][131] = Cd[2][3]; Md[309][132] = Cd[2][5]; Md[309][134] = Cd[2][7]; Md[309][211] = Cd[2][15]; Md[309][266] = Cd[2][30]; Md[309][295] = Cd[2][2]; Md[309][298] = Cd[2][6]; Md[309][300] = Cd[2][9]; Md[309][301] = Cd[2][10]; Md[309][304] = Cd[2][16]; Md[309][306] = Cd[2][19]; Md[309][307] = Cd[2][20]; Md[309][309] = Cd[2][25]; Md[309][310] = Cd[2][26]; Md[309][411] = Cd[2][12]; Md[309][417] = Cd[2][22]; Md[309][420] = Cd[2][28]; Md[309][421] = Cd[2][31]; 
   Md[310][44] = Cd[2][24]; Md[310][45] = Cd[2][30]; Md[310][53] = Cd[2][16]; Md[310][85] = Cd[2][14]; Md[310][130] = Cd[2][1]; Md[310][131] = Cd[2][2]; Md[310][133] = Cd[2][5]; Md[310][134] = Cd[2][6]; Md[310][212] = Cd[2][15]; Md[310][301] = Cd[2][9]; Md[310][307] = Cd[2][19]; Md[310][310] = Cd[2][25]; Md[310][376] = Cd[2][33]; Md[310][406] = Cd[2][3]; Md[310][409] = Cd[2][7]; Md[310][411] = Cd[2][10]; Md[310][412] = Cd[2][12]; Md[310][415] = Cd[2][17]; Md[310][417] = Cd[2][20]; Md[310][418] = Cd[2][22]; Md[310][420] = Cd[2][26]; Md[310][421] = Cd[2][28]; Md[310][422] = Cd[2][31]; 
   Md[311][168] = Cd[2][14]; Md[311][171] = Cd[2][24]; Md[311][172] = Cd[2][30]; Md[311][173] = Cd[2][33]; Md[311][199] = Cd[2][1]; Md[311][200] = Cd[2][2]; Md[311][201] = Cd[2][3]; Md[311][205] = Cd[2][5]; Md[311][206] = Cd[2][6]; Md[311][207] = Cd[2][7]; Md[311][208] = Cd[2][9]; Md[311][209] = Cd[2][10]; Md[311][210] = Cd[2][12]; Md[311][213] = Cd[2][15]; Md[311][214] = Cd[2][16]; Md[311][215] = Cd[2][17]; Md[311][216] = Cd[2][19]; Md[311][217] = Cd[2][20]; Md[311][218] = Cd[2][22]; Md[311][219] = Cd[2][26]; Md[311][220] = Cd[2][28]; Md[311][311] = Cd[2][25]; Md[311][424] = Cd[2][31]; 
   Md[312][169] = Cd[2][14]; Md[312][172] = Cd[2][24]; Md[312][174] = Cd[2][30]; Md[312][175] = Cd[2][33]; Md[312][200] = Cd[2][1]; Md[312][202] = Cd[2][2]; Md[312][203] = Cd[2][3]; Md[312][206] = Cd[2][5]; Md[312][208] = Cd[2][6]; Md[312][209] = Cd[2][7]; Md[312][211] = Cd[2][10]; Md[312][212] = Cd[2][12]; Md[312][214] = Cd[2][15]; Md[312][216] = Cd[2][16]; Md[312][217] = Cd[2][17]; Md[312][219] = Cd[2][20]; Md[312][220] = Cd[2][22]; Md[312][221] = Cd[2][28]; Md[312][302] = Cd[2][9]; Md[312][311] = Cd[2][19]; Md[312][312] = Cd[2][25]; Md[312][313] = Cd[2][26]; Md[312][425] = Cd[2][31]; 
   Md[313][170] = Cd[2][14]; Md[313][173] = Cd[2][24]; Md[313][175] = Cd[2][30]; Md[313][176] = Cd[2][33]; Md[313][201] = Cd[2][1]; Md[313][203] = Cd[2][2]; Md[313][204] = Cd[2][3]; Md[313][207] = Cd[2][5]; Md[313][209] = Cd[2][6]; Md[313][210] = Cd[2][7]; Md[313][211] = Cd[2][9]; Md[313][212] = Cd[2][10]; Md[313][215] = Cd[2][15]; Md[313][217] = Cd[2][16]; Md[313][218] = Cd[2][17]; Md[313][219] = Cd[2][19]; Md[313][220] = Cd[2][20]; Md[313][221] = Cd[2][26]; Md[313][313] = Cd[2][25]; Md[313][414] = Cd[2][12]; Md[313][424] = Cd[2][22]; Md[313][425] = Cd[2][28]; Md[313][426] = Cd[2][31]; 
   Md[314][40] = Cd[2][14]; Md[314][46] = Cd[2][33]; Md[314][53] = Cd[2][12]; Md[314][132] = Cd[2][3]; Md[314][174] = Cd[2][24]; Md[314][202] = Cd[2][1]; Md[314][208] = Cd[2][5]; Md[314][211] = Cd[2][7]; Md[314][216] = Cd[2][15]; Md[314][219] = Cd[2][17]; Md[314][221] = Cd[2][22]; Md[314][267] = Cd[2][30]; Md[314][296] = Cd[2][2]; Md[314][302] = Cd[2][6]; Md[314][303] = Cd[2][9]; Md[314][304] = Cd[2][10]; Md[314][311] = Cd[2][16]; Md[314][312] = Cd[2][19]; Md[314][313] = Cd[2][20]; Md[314][314] = Cd[2][25]; Md[314][315] = Cd[2][26]; Md[314][316] = Cd[2][28]; Md[314][427] = Cd[2][31]; 
   Md[315][41] = Cd[2][14]; Md[315][46] = Cd[2][30]; Md[315][47] = Cd[2][33]; Md[315][53] = Cd[2][10]; Md[315][132] = Cd[2][2]; Md[315][133] = Cd[2][3]; Md[315][175] = Cd[2][24]; Md[315][203] = Cd[2][1]; Md[315][209] = Cd[2][5]; Md[315][211] = Cd[2][6]; Md[315][212] = Cd[2][7]; Md[315][217] = Cd[2][15]; Md[315][219] = Cd[2][16]; Md[315][220] = Cd[2][17]; Md[315][221] = Cd[2][20]; Md[315][304] = Cd[2][9]; Md[315][313] = Cd[2][19]; Md[315][315] = Cd[2][25]; Md[315][316] = Cd[2][26]; Md[315][415] = Cd[2][12]; Md[315][425] = Cd[2][22]; Md[315][427] = Cd[2][28]; Md[315][428] = Cd[2][31]; 
   Md[316][42] = Cd[2][14]; Md[316][47] = Cd[2][30]; Md[316][53] = Cd[2][9]; Md[316][133] = Cd[2][2]; Md[316][176] = Cd[2][24]; Md[316][204] = Cd[2][1]; Md[316][210] = Cd[2][5]; Md[316][212] = Cd[2][6]; Md[316][218] = Cd[2][15]; Md[316][220] = Cd[2][16]; Md[316][221] = Cd[2][19]; Md[316][316] = Cd[2][25]; Md[316][378] = Cd[2][33]; Md[316][408] = Cd[2][3]; Md[316][414] = Cd[2][7]; Md[316][415] = Cd[2][10]; Md[316][416] = Cd[2][12]; Md[316][424] = Cd[2][17]; Md[316][425] = Cd[2][20]; Md[316][426] = Cd[2][22]; Md[316][427] = Cd[2][26]; Md[316][428] = Cd[2][28]; Md[316][429] = Cd[2][31]; 
   Md[317][264] = Cd[2][14]; Md[317][267] = Cd[2][24]; Md[317][268] = Cd[2][30]; Md[317][269] = Cd[2][33]; Md[317][296] = Cd[2][1]; Md[317][297] = Cd[2][2]; Md[317][298] = Cd[2][3]; Md[317][302] = Cd[2][5]; Md[317][303] = Cd[2][6]; Md[317][304] = Cd[2][7]; Md[317][305] = Cd[2][9]; Md[317][306] = Cd[2][10]; Md[317][307] = Cd[2][12]; Md[317][311] = Cd[2][15]; Md[317][312] = Cd[2][16]; Md[317][313] = Cd[2][17]; Md[317][314] = Cd[2][19]; Md[317][315] = Cd[2][20]; Md[317][316] = Cd[2][22]; Md[317][317] = Cd[2][25]; Md[317][318] = Cd[2][26]; Md[317][319] = Cd[2][28]; Md[317][430] = Cd[2][31]; 
   Md[318][43] = Cd[2][14]; Md[318][46] = Cd[2][24]; Md[318][48] = Cd[2][33]; Md[318][53] = Cd[2][7]; Md[318][132] = Cd[2][1]; Md[318][134] = Cd[2][3]; Md[318][211] = Cd[2][5]; Md[318][219] = Cd[2][15]; Md[318][221] = Cd[2][17]; Md[318][269] = Cd[2][30]; Md[318][298] = Cd[2][2]; Md[318][304] = Cd[2][6]; Md[318][306] = Cd[2][9]; Md[318][307] = Cd[2][10]; Md[318][313] = Cd[2][16]; Md[318][315] = Cd[2][19]; Md[318][316] = Cd[2][20]; Md[318][318] = Cd[2][25]; Md[318][319] = Cd[2][26]; Md[318][417] = Cd[2][12]; Md[318][427] = Cd[2][22]; Md[318][430] = Cd[2][28]; Md[318][431] = Cd[2][31]; 
   Md[319][44] = Cd[2][14]; Md[319][47] = Cd[2][24]; Md[319][48] = Cd[2][30]; Md[319][53] = Cd[2][6]; Md[319][133] = Cd[2][1]; Md[319][134] = Cd[2][2]; Md[319][212] = Cd[2][5]; Md[319][220] = Cd[2][15]; Md[319][221] = Cd[2][16]; Md[319][307] = Cd[2][9]; Md[319][316] = Cd[2][19]; Md[319][319] = Cd[2][25]; Md[319][379] = Cd[2][33]; Md[319][409] = Cd[2][3]; Md[319][415] = Cd[2][7]; Md[319][417] = Cd[2][10]; Md[319][418] = Cd[2][12]; Md[319][425] = Cd[2][17]; Md[319][427] = Cd[2][20]; Md[319][428] = Cd[2][22]; Md[319][430] = Cd[2][26]; Md[319][431] = Cd[2][28]; Md[319][432] = Cd[2][31]; 
   Md[320][265] = Cd[2][14]; Md[320][268] = Cd[2][24]; Md[320][270] = Cd[2][30]; Md[320][271] = Cd[2][33]; Md[320][297] = Cd[2][1]; Md[320][299] = Cd[2][2]; Md[320][300] = Cd[2][3]; Md[320][303] = Cd[2][5]; Md[320][305] = Cd[2][6]; Md[320][306] = Cd[2][7]; Md[320][308] = Cd[2][9]; Md[320][309] = Cd[2][10]; Md[320][310] = Cd[2][12]; Md[320][312] = Cd[2][15]; Md[320][314] = Cd[2][16]; Md[320][315] = Cd[2][17]; Md[320][317] = Cd[2][19]; Md[320][318] = Cd[2][20]; Md[320][319] = Cd[2][22]; Md[320][320] = Cd[2][25]; Md[320][321] = Cd[2][26]; Md[320][322] = Cd[2][28]; Md[320][434] = Cd[2][31]; 
   Md[321][266] = Cd[2][14]; Md[321][269] = Cd[2][24]; Md[321][271] = Cd[2][30]; Md[321][272] = Cd[2][33]; Md[321][298] = Cd[2][1]; Md[321][300] = Cd[2][2]; Md[321][301] = Cd[2][3]; Md[321][304] = Cd[2][5]; Md[321][306] = Cd[2][6]; Md[321][307] = Cd[2][7]; Md[321][309] = Cd[2][9]; Md[321][310] = Cd[2][10]; Md[321][313] = Cd[2][15]; Md[321][315] = Cd[2][16]; Md[321][316] = Cd[2][17]; Md[321][318] = Cd[2][19]; Md[321][319] = Cd[2][20]; Md[321][321] = Cd[2][25]; Md[321][322] = Cd[2][26]; Md[321][420] = Cd[2][12]; Md[321][430] = Cd[2][22]; Md[321][434] = Cd[2][28]; Md[321][435] = Cd[2][31]; 
   Md[322][45] = Cd[2][14]; Md[322][48] = Cd[2][24]; Md[322][53] = Cd[2][5]; Md[322][134] = Cd[2][1]; Md[322][221] = Cd[2][15]; Md[322][272] = Cd[2][30]; Md[322][301] = Cd[2][2]; Md[322][307] = Cd[2][6]; Md[322][310] = Cd[2][9]; Md[322][316] = Cd[2][16]; Md[322][319] = Cd[2][19]; Md[322][322] = Cd[2][25]; Md[322][381] = Cd[2][33]; Md[322][411] = Cd[2][3]; Md[322][417] = Cd[2][7]; Md[322][420] = Cd[2][10]; Md[322][421] = Cd[2][12]; Md[322][427] = Cd[2][17]; Md[322][430] = Cd[2][20]; Md[322][431] = Cd[2][22]; Md[322][434] = Cd[2][26]; Md[322][435] = Cd[2][28]; Md[322][436] = Cd[2][31]; 
   Md[323][86] = Cd[2][14]; Md[323][87] = Cd[2][24]; Md[323][88] = Cd[2][30]; Md[323][89] = Cd[2][33]; Md[323][136] = Cd[2][1]; Md[323][137] = Cd[2][2]; Md[323][138] = Cd[2][3]; Md[323][139] = Cd[2][5]; Md[323][140] = Cd[2][6]; Md[323][141] = Cd[2][7]; Md[323][142] = Cd[2][9]; Md[323][143] = Cd[2][10]; Md[323][144] = Cd[2][12]; Md[323][145] = Cd[2][16]; Md[323][146] = Cd[2][17]; Md[323][147] = Cd[2][19]; Md[323][148] = Cd[2][20]; Md[323][149] = Cd[2][22]; Md[323][150] = Cd[2][26]; Md[323][151] = Cd[2][28]; Md[323][222] = Cd[2][15]; Md[323][323] = Cd[2][25]; Md[323][439] = Cd[2][31]; 
   Md[324][87] = Cd[2][14]; Md[324][90] = Cd[2][24]; Md[324][91] = Cd[2][30]; Md[324][92] = Cd[2][33]; Md[324][139] = Cd[2][1]; Md[324][140] = Cd[2][2]; Md[324][141] = Cd[2][3]; Md[324][145] = Cd[2][6]; Md[324][146] = Cd[2][7]; Md[324][147] = Cd[2][9]; Md[324][148] = Cd[2][10]; Md[324][149] = Cd[2][12]; Md[324][152] = Cd[2][19]; Md[324][153] = Cd[2][20]; Md[324][154] = Cd[2][22]; Md[324][155] = Cd[2][26]; Md[324][156] = Cd[2][28]; Md[324][222] = Cd[2][5]; Md[324][223] = Cd[2][15]; Md[324][224] = Cd[2][16]; Md[324][225] = Cd[2][17]; Md[324][324] = Cd[2][25]; Md[324][440] = Cd[2][31]; 
   Md[325][88] = Cd[2][14]; Md[325][91] = Cd[2][24]; Md[325][93] = Cd[2][30]; Md[325][94] = Cd[2][33]; Md[325][140] = Cd[2][1]; Md[325][142] = Cd[2][2]; Md[325][143] = Cd[2][3]; Md[325][145] = Cd[2][5]; Md[325][147] = Cd[2][6]; Md[325][148] = Cd[2][7]; Md[325][150] = Cd[2][10]; Md[325][151] = Cd[2][12]; Md[325][152] = Cd[2][16]; Md[325][153] = Cd[2][17]; Md[325][155] = Cd[2][20]; Md[325][156] = Cd[2][22]; Md[325][157] = Cd[2][28]; Md[325][224] = Cd[2][15]; Md[325][323] = Cd[2][9]; Md[325][324] = Cd[2][19]; Md[325][325] = Cd[2][25]; Md[325][326] = Cd[2][26]; Md[325][441] = Cd[2][31]; 
   Md[326][89] = Cd[2][14]; Md[326][92] = Cd[2][24]; Md[326][94] = Cd[2][30]; Md[326][95] = Cd[2][33]; Md[326][141] = Cd[2][1]; Md[326][143] = Cd[2][2]; Md[326][144] = Cd[2][3]; Md[326][146] = Cd[2][5]; Md[326][148] = Cd[2][6]; Md[326][149] = Cd[2][7]; Md[326][150] = Cd[2][9]; Md[326][151] = Cd[2][10]; Md[326][153] = Cd[2][16]; Md[326][154] = Cd[2][17]; Md[326][155] = Cd[2][19]; Md[326][156] = Cd[2][20]; Md[326][157] = Cd[2][26]; Md[326][225] = Cd[2][15]; Md[326][326] = Cd[2][25]; Md[326][439] = Cd[2][12]; Md[326][440] = Cd[2][22]; Md[326][441] = Cd[2][28]; Md[326][442] = Cd[2][31]; 
   Md[327][90] = Cd[2][14]; Md[327][96] = Cd[2][30]; Md[327][97] = Cd[2][33]; Md[327][145] = Cd[2][2]; Md[327][146] = Cd[2][3]; Md[327][152] = Cd[2][9]; Md[327][153] = Cd[2][10]; Md[327][154] = Cd[2][12]; Md[327][158] = Cd[2][26]; Md[327][159] = Cd[2][28]; Md[327][177] = Cd[2][24]; Md[327][222] = Cd[2][1]; Md[327][223] = Cd[2][5]; Md[327][224] = Cd[2][6]; Md[327][225] = Cd[2][7]; Md[327][226] = Cd[2][15]; Md[327][227] = Cd[2][16]; Md[327][228] = Cd[2][17]; Md[327][229] = Cd[2][19]; Md[327][230] = Cd[2][20]; Md[327][231] = Cd[2][22]; Md[327][327] = Cd[2][25]; Md[327][443] = Cd[2][31]; 
   Md[328][91] = Cd[2][14]; Md[328][96] = Cd[2][24]; Md[328][98] = Cd[2][30]; Md[328][99] = Cd[2][33]; Md[328][145] = Cd[2][1]; Md[328][147] = Cd[2][2]; Md[328][148] = Cd[2][3]; Md[328][152] = Cd[2][6]; Md[328][153] = Cd[2][7]; Md[328][155] = Cd[2][10]; Md[328][156] = Cd[2][12]; Md[328][158] = Cd[2][20]; Md[328][159] = Cd[2][22]; Md[328][160] = Cd[2][28]; Md[328][224] = Cd[2][5]; Md[328][227] = Cd[2][15]; Md[328][229] = Cd[2][16]; Md[328][230] = Cd[2][17]; Md[328][324] = Cd[2][9]; Md[328][327] = Cd[2][19]; Md[328][328] = Cd[2][25]; Md[328][329] = Cd[2][26]; Md[328][444] = Cd[2][31]; 
   Md[329][92] = Cd[2][14]; Md[329][97] = Cd[2][24]; Md[329][99] = Cd[2][30]; Md[329][100] = Cd[2][33]; Md[329][146] = Cd[2][1]; Md[329][148] = Cd[2][2]; Md[329][149] = Cd[2][3]; Md[329][153] = Cd[2][6]; Md[329][154] = Cd[2][7]; Md[329][155] = Cd[2][9]; Md[329][156] = Cd[2][10]; Md[329][158] = Cd[2][19]; Md[329][159] = Cd[2][20]; Md[329][160] = Cd[2][26]; Md[329][225] = Cd[2][5]; Md[329][228] = Cd[2][15]; Md[329][230] = Cd[2][16]; Md[329][231] = Cd[2][17]; Md[329][329] = Cd[2][25]; Md[329][440] = Cd[2][12]; Md[329][443] = Cd[2][22]; Md[329][444] = Cd[2][28]; Md[329][445] = Cd[2][31]; 
   Md[330][93] = Cd[2][14]; Md[330][98] = Cd[2][24]; Md[330][101] = Cd[2][33]; Md[330][147] = Cd[2][1]; Md[330][150] = Cd[2][3]; Md[330][152] = Cd[2][5]; Md[330][155] = Cd[2][7]; Md[330][157] = Cd[2][12]; Md[330][158] = Cd[2][17]; Md[330][160] = Cd[2][22]; Md[330][229] = Cd[2][15]; Md[330][273] = Cd[2][30]; Md[330][323] = Cd[2][2]; Md[330][324] = Cd[2][6]; Md[330][325] = Cd[2][9]; Md[330][326] = Cd[2][10]; Md[330][327] = Cd[2][16]; Md[330][328] = Cd[2][19]; Md[330][329] = Cd[2][20]; Md[330][330] = Cd[2][25]; Md[330][331] = Cd[2][26]; Md[330][332] = Cd[2][28]; Md[330][446] = Cd[2][31]; 
   Md[331][94] = Cd[2][14]; Md[331][99] = Cd[2][24]; Md[331][101] = Cd[2][30]; Md[331][102] = Cd[2][33]; Md[331][148] = Cd[2][1]; Md[331][150] = Cd[2][2]; Md[331][151] = Cd[2][3]; Md[331][153] = Cd[2][5]; Md[331][155] = Cd[2][6]; Md[331][156] = Cd[2][7]; Md[331][157] = Cd[2][10]; Md[331][158] = Cd[2][16]; Md[331][159] = Cd[2][17]; Md[331][160] = Cd[2][20]; Md[331][230] = Cd[2][15]; Md[331][326] = Cd[2][9]; Md[331][329] = Cd[2][19]; Md[331][331] = Cd[2][25]; Md[331][332] = Cd[2][26]; Md[331][441] = Cd[2][12]; Md[331][444] = Cd[2][22]; Md[331][446] = Cd[2][28]; Md[331][447] = Cd[2][31]; 
   Md[332][95] = Cd[2][14]; Md[332][100] = Cd[2][24]; Md[332][102] = Cd[2][30]; Md[332][149] = Cd[2][1]; Md[332][151] = Cd[2][2]; Md[332][154] = Cd[2][5]; Md[332][156] = Cd[2][6]; Md[332][157] = Cd[2][9]; Md[332][159] = Cd[2][16]; Md[332][160] = Cd[2][19]; Md[332][231] = Cd[2][15]; Md[332][332] = Cd[2][25]; Md[332][384] = Cd[2][33]; Md[332][439] = Cd[2][3]; Md[332][440] = Cd[2][7]; Md[332][441] = Cd[2][10]; Md[332][442] = Cd[2][12]; Md[332][443] = Cd[2][17]; Md[332][444] = Cd[2][20]; Md[332][445] = Cd[2][22]; Md[332][446] = Cd[2][26]; Md[332][447] = Cd[2][28]; Md[332][448] = Cd[2][31]; 
   Md[333][177] = Cd[2][14]; Md[333][178] = Cd[2][24]; Md[333][179] = Cd[2][30]; Md[333][180] = Cd[2][33]; Md[333][223] = Cd[2][1]; Md[333][224] = Cd[2][2]; Md[333][225] = Cd[2][3]; Md[333][226] = Cd[2][5]; Md[333][227] = Cd[2][6]; Md[333][228] = Cd[2][7]; Md[333][229] = Cd[2][9]; Md[333][230] = Cd[2][10]; Md[333][231] = Cd[2][12]; Md[333][232] = Cd[2][15]; Md[333][233] = Cd[2][16]; Md[333][234] = Cd[2][17]; Md[333][235] = Cd[2][19]; Md[333][236] = Cd[2][20]; Md[333][237] = Cd[2][22]; Md[333][238] = Cd[2][26]; Md[333][239] = Cd[2][28]; Md[333][333] = Cd[2][25]; Md[333][449] = Cd[2][31]; 
   Md[334][96] = Cd[2][14]; Md[334][103] = Cd[2][30]; Md[334][104] = Cd[2][33]; Md[334][152] = Cd[2][2]; Md[334][153] = Cd[2][3]; Md[334][158] = Cd[2][10]; Md[334][159] = Cd[2][12]; Md[334][161] = Cd[2][28]; Md[334][179] = Cd[2][24]; Md[334][224] = Cd[2][1]; Md[334][227] = Cd[2][5]; Md[334][229] = Cd[2][6]; Md[334][230] = Cd[2][7]; Md[334][233] = Cd[2][15]; Md[334][235] = Cd[2][16]; Md[334][236] = Cd[2][17]; Md[334][238] = Cd[2][20]; Md[334][239] = Cd[2][22]; Md[334][327] = Cd[2][9]; Md[334][333] = Cd[2][19]; Md[334][334] = Cd[2][25]; Md[334][335] = Cd[2][26]; Md[334][450] = Cd[2][31]; 
   Md[335][97] = Cd[2][14]; Md[335][104] = Cd[2][30]; Md[335][105] = Cd[2][33]; Md[335][153] = Cd[2][2]; Md[335][154] = Cd[2][3]; Md[335][158] = Cd[2][9]; Md[335][159] = Cd[2][10]; Md[335][161] = Cd[2][26]; Md[335][180] = Cd[2][24]; Md[335][225] = Cd[2][1]; Md[335][228] = Cd[2][5]; Md[335][230] = Cd[2][6]; Md[335][231] = Cd[2][7]; Md[335][234] = Cd[2][15]; Md[335][236] = Cd[2][16]; Md[335][237] = Cd[2][17]; Md[335][238] = Cd[2][19]; Md[335][239] = Cd[2][20]; Md[335][335] = Cd[2][25]; Md[335][443] = Cd[2][12]; Md[335][449] = Cd[2][22]; Md[335][450] = Cd[2][28]; Md[335][451] = Cd[2][31]; 
   Md[336][98] = Cd[2][14]; Md[336][103] = Cd[2][24]; Md[336][106] = Cd[2][33]; Md[336][152] = Cd[2][1]; Md[336][155] = Cd[2][3]; Md[336][158] = Cd[2][7]; Md[336][160] = Cd[2][12]; Md[336][161] = Cd[2][22]; Md[336][229] = Cd[2][5]; Md[336][235] = Cd[2][15]; Md[336][238] = Cd[2][17]; Md[336][274] = Cd[2][30]; Md[336][324] = Cd[2][2]; Md[336][327] = Cd[2][6]; Md[336][328] = Cd[2][9]; Md[336][329] = Cd[2][10]; Md[336][333] = Cd[2][16]; Md[336][334] = Cd[2][19]; Md[336][335] = Cd[2][20]; Md[336][336] = Cd[2][25]; Md[336][337] = Cd[2][26]; Md[336][338] = Cd[2][28]; Md[336][452] = Cd[2][31]; 
   Md[337][99] = Cd[2][14]; Md[337][104] = Cd[2][24]; Md[337][106] = Cd[2][30]; Md[337][107] = Cd[2][33]; Md[337][153] = Cd[2][1]; Md[337][155] = Cd[2][2]; Md[337][156] = Cd[2][3]; Md[337][158] = Cd[2][6]; Md[337][159] = Cd[2][7]; Md[337][160] = Cd[2][10]; Md[337][161] = Cd[2][20]; Md[337][230] = Cd[2][5]; Md[337][236] = Cd[2][15]; Md[337][238] = Cd[2][16]; Md[337][239] = Cd[2][17]; Md[337][329] = Cd[2][9]; Md[337][335] = Cd[2][19]; Md[337][337] = Cd[2][25]; Md[337][338] = Cd[2][26]; Md[337][444] = Cd[2][12]; Md[337][450] = Cd[2][22]; Md[337][452] = Cd[2][28]; Md[337][453] = Cd[2][31]; 
   Md[338][100] = Cd[2][14]; Md[338][105] = Cd[2][24]; Md[338][107] = Cd[2][30]; Md[338][154] = Cd[2][1]; Md[338][156] = Cd[2][2]; Md[338][159] = Cd[2][6]; Md[338][160] = Cd[2][9]; Md[338][161] = Cd[2][19]; Md[338][231] = Cd[2][5]; Md[338][237] = Cd[2][15]; Md[338][239] = Cd[2][16]; Md[338][338] = Cd[2][25]; Md[338][385] = Cd[2][33]; Md[338][440] = Cd[2][3]; Md[338][443] = Cd[2][7]; Md[338][444] = Cd[2][10]; Md[338][445] = Cd[2][12]; Md[338][449] = Cd[2][17]; Md[338][450] = Cd[2][20]; Md[338][451] = Cd[2][22]; Md[338][452] = Cd[2][26]; Md[338][453] = Cd[2][28]; Md[338][454] = Cd[2][31]; 
   Md[339][273] = Cd[2][14]; Md[339][274] = Cd[2][24]; Md[339][275] = Cd[2][30]; Md[339][276] = Cd[2][33]; Md[339][324] = Cd[2][1]; Md[339][325] = Cd[2][2]; Md[339][326] = Cd[2][3]; Md[339][327] = Cd[2][5]; Md[339][328] = Cd[2][6]; Md[339][329] = Cd[2][7]; Md[339][330] = Cd[2][9]; Md[339][331] = Cd[2][10]; Md[339][332] = Cd[2][12]; Md[339][333] = Cd[2][15]; Md[339][334] = Cd[2][16]; Md[339][335] = Cd[2][17]; Md[339][336] = Cd[2][19]; Md[339][337] = Cd[2][20]; Md[339][338] = Cd[2][22]; Md[339][339] = Cd[2][25]; Md[339][340] = Cd[2][26]; Md[339][341] = Cd[2][28]; Md[339][455] = Cd[2][31]; 
   Md[340][101] = Cd[2][14]; Md[340][106] = Cd[2][24]; Md[340][108] = Cd[2][33]; Md[340][155] = Cd[2][1]; Md[340][157] = Cd[2][3]; Md[340][158] = Cd[2][5]; Md[340][160] = Cd[2][7]; Md[340][161] = Cd[2][17]; Md[340][238] = Cd[2][15]; Md[340][276] = Cd[2][30]; Md[340][326] = Cd[2][2]; Md[340][329] = Cd[2][6]; Md[340][331] = Cd[2][9]; Md[340][332] = Cd[2][10]; Md[340][335] = Cd[2][16]; Md[340][337] = Cd[2][19]; Md[340][338] = Cd[2][20]; Md[340][340] = Cd[2][25]; Md[340][341] = Cd[2][26]; Md[340][446] = Cd[2][12]; Md[340][452] = Cd[2][22]; Md[340][455] = Cd[2][28]; Md[340][456] = Cd[2][31]; 
   Md[341][102] = Cd[2][14]; Md[341][107] = Cd[2][24]; Md[341][108] = Cd[2][30]; Md[341][156] = Cd[2][1]; Md[341][157] = Cd[2][2]; Md[341][159] = Cd[2][5]; Md[341][160] = Cd[2][6]; Md[341][161] = Cd[2][16]; Md[341][239] = Cd[2][15]; Md[341][332] = Cd[2][9]; Md[341][338] = Cd[2][19]; Md[341][341] = Cd[2][25]; Md[341][386] = Cd[2][33]; Md[341][441] = Cd[2][3]; Md[341][444] = Cd[2][7]; Md[341][446] = Cd[2][10]; Md[341][447] = Cd[2][12]; Md[341][450] = Cd[2][17]; Md[341][452] = Cd[2][20]; Md[341][453] = Cd[2][22]; Md[341][455] = Cd[2][26]; Md[341][456] = Cd[2][28]; Md[341][457] = Cd[2][31]; 
   Md[342][178] = Cd[2][14]; Md[342][181] = Cd[2][24]; Md[342][182] = Cd[2][30]; Md[342][183] = Cd[2][33]; Md[342][226] = Cd[2][1]; Md[342][227] = Cd[2][2]; Md[342][228] = Cd[2][3]; Md[342][232] = Cd[2][5]; Md[342][233] = Cd[2][6]; Md[342][234] = Cd[2][7]; Md[342][235] = Cd[2][9]; Md[342][236] = Cd[2][10]; Md[342][237] = Cd[2][12]; Md[342][240] = Cd[2][15]; Md[342][241] = Cd[2][16]; Md[342][242] = Cd[2][17]; Md[342][243] = Cd[2][19]; Md[342][244] = Cd[2][20]; Md[342][245] = Cd[2][22]; Md[342][246] = Cd[2][26]; Md[342][247] = Cd[2][28]; Md[342][342] = Cd[2][25]; Md[342][459] = Cd[2][31]; 
   Md[343][179] = Cd[2][14]; Md[343][182] = Cd[2][24]; Md[343][184] = Cd[2][30]; Md[343][185] = Cd[2][33]; Md[343][227] = Cd[2][1]; Md[343][229] = Cd[2][2]; Md[343][230] = Cd[2][3]; Md[343][233] = Cd[2][5]; Md[343][235] = Cd[2][6]; Md[343][236] = Cd[2][7]; Md[343][238] = Cd[2][10]; Md[343][239] = Cd[2][12]; Md[343][241] = Cd[2][15]; Md[343][243] = Cd[2][16]; Md[343][244] = Cd[2][17]; Md[343][246] = Cd[2][20]; Md[343][247] = Cd[2][22]; Md[343][248] = Cd[2][28]; Md[343][333] = Cd[2][9]; Md[343][342] = Cd[2][19]; Md[343][343] = Cd[2][25]; Md[343][344] = Cd[2][26]; Md[343][460] = Cd[2][31]; 
   Md[344][180] = Cd[2][14]; Md[344][183] = Cd[2][24]; Md[344][185] = Cd[2][30]; Md[344][186] = Cd[2][33]; Md[344][228] = Cd[2][1]; Md[344][230] = Cd[2][2]; Md[344][231] = Cd[2][3]; Md[344][234] = Cd[2][5]; Md[344][236] = Cd[2][6]; Md[344][237] = Cd[2][7]; Md[344][238] = Cd[2][9]; Md[344][239] = Cd[2][10]; Md[344][242] = Cd[2][15]; Md[344][244] = Cd[2][16]; Md[344][245] = Cd[2][17]; Md[344][246] = Cd[2][19]; Md[344][247] = Cd[2][20]; Md[344][248] = Cd[2][26]; Md[344][344] = Cd[2][25]; Md[344][449] = Cd[2][12]; Md[344][459] = Cd[2][22]; Md[344][460] = Cd[2][28]; Md[344][461] = Cd[2][31]; 
   Md[345][49] = Cd[2][33]; Md[345][103] = Cd[2][14]; Md[345][158] = Cd[2][3]; Md[345][161] = Cd[2][12]; Md[345][184] = Cd[2][24]; Md[345][229] = Cd[2][1]; Md[345][235] = Cd[2][5]; Md[345][238] = Cd[2][7]; Md[345][243] = Cd[2][15]; Md[345][246] = Cd[2][17]; Md[345][248] = Cd[2][22]; Md[345][277] = Cd[2][30]; Md[345][327] = Cd[2][2]; Md[345][333] = Cd[2][6]; Md[345][334] = Cd[2][9]; Md[345][335] = Cd[2][10]; Md[345][342] = Cd[2][16]; Md[345][343] = Cd[2][19]; Md[345][344] = Cd[2][20]; Md[345][345] = Cd[2][25]; Md[345][346] = Cd[2][26]; Md[345][347] = Cd[2][28]; Md[345][462] = Cd[2][31]; 
   Md[346][49] = Cd[2][30]; Md[346][50] = Cd[2][33]; Md[346][104] = Cd[2][14]; Md[346][158] = Cd[2][2]; Md[346][159] = Cd[2][3]; Md[346][161] = Cd[2][10]; Md[346][185] = Cd[2][24]; Md[346][230] = Cd[2][1]; Md[346][236] = Cd[2][5]; Md[346][238] = Cd[2][6]; Md[346][239] = Cd[2][7]; Md[346][244] = Cd[2][15]; Md[346][246] = Cd[2][16]; Md[346][247] = Cd[2][17]; Md[346][248] = Cd[2][20]; Md[346][335] = Cd[2][9]; Md[346][344] = Cd[2][19]; Md[346][346] = Cd[2][25]; Md[346][347] = Cd[2][26]; Md[346][450] = Cd[2][12]; Md[346][460] = Cd[2][22]; Md[346][462] = Cd[2][28]; Md[346][463] = Cd[2][31]; 
   Md[347][50] = Cd[2][30]; Md[347][105] = Cd[2][14]; Md[347][159] = Cd[2][2]; Md[347][161] = Cd[2][9]; Md[347][186] = Cd[2][24]; Md[347][231] = Cd[2][1]; Md[347][237] = Cd[2][5]; Md[347][239] = Cd[2][6]; Md[347][245] = Cd[2][15]; Md[347][247] = Cd[2][16]; Md[347][248] = Cd[2][19]; Md[347][347] = Cd[2][25]; Md[347][388] = Cd[2][33]; Md[347][443] = Cd[2][3]; Md[347][449] = Cd[2][7]; Md[347][450] = Cd[2][10]; Md[347][451] = Cd[2][12]; Md[347][459] = Cd[2][17]; Md[347][460] = Cd[2][20]; Md[347][461] = Cd[2][22]; Md[347][462] = Cd[2][26]; Md[347][463] = Cd[2][28]; Md[347][464] = Cd[2][31]; 
   Md[348][274] = Cd[2][14]; Md[348][277] = Cd[2][24]; Md[348][278] = Cd[2][30]; Md[348][279] = Cd[2][33]; Md[348][327] = Cd[2][1]; Md[348][328] = Cd[2][2]; Md[348][329] = Cd[2][3]; Md[348][333] = Cd[2][5]; Md[348][334] = Cd[2][6]; Md[348][335] = Cd[2][7]; Md[348][336] = Cd[2][9]; Md[348][337] = Cd[2][10]; Md[348][338] = Cd[2][12]; Md[348][342] = Cd[2][15]; Md[348][343] = Cd[2][16]; Md[348][344] = Cd[2][17]; Md[348][345] = Cd[2][19]; Md[348][346] = Cd[2][20]; Md[348][347] = Cd[2][22]; Md[348][348] = Cd[2][25]; Md[348][349] = Cd[2][26]; Md[348][350] = Cd[2][28]; Md[348][465] = Cd[2][31]; 
   Md[349][49] = Cd[2][24]; Md[349][51] = Cd[2][33]; Md[349][106] = Cd[2][14]; Md[349][158] = Cd[2][1]; Md[349][160] = Cd[2][3]; Md[349][161] = Cd[2][7]; Md[349][238] = Cd[2][5]; Md[349][246] = Cd[2][15]; Md[349][248] = Cd[2][17]; Md[349][279] = Cd[2][30]; Md[349][329] = Cd[2][2]; Md[349][335] = Cd[2][6]; Md[349][337] = Cd[2][9]; Md[349][338] = Cd[2][10]; Md[349][344] = Cd[2][16]; Md[349][346] = Cd[2][19]; Md[349][347] = Cd[2][20]; Md[349][349] = Cd[2][25]; Md[349][350] = Cd[2][26]; Md[349][452] = Cd[2][12]; Md[349][462] = Cd[2][22]; Md[349][465] = Cd[2][28]; Md[349][466] = Cd[2][31]; 
   Md[350][50] = Cd[2][24]; Md[350][51] = Cd[2][30]; Md[350][107] = Cd[2][14]; Md[350][159] = Cd[2][1]; Md[350][160] = Cd[2][2]; Md[350][161] = Cd[2][6]; Md[350][239] = Cd[2][5]; Md[350][247] = Cd[2][15]; Md[350][248] = Cd[2][16]; Md[350][338] = Cd[2][9]; Md[350][347] = Cd[2][19]; Md[350][350] = Cd[2][25]; Md[350][389] = Cd[2][33]; Md[350][444] = Cd[2][3]; Md[350][450] = Cd[2][7]; Md[350][452] = Cd[2][10]; Md[350][453] = Cd[2][12]; Md[350][460] = Cd[2][17]; Md[350][462] = Cd[2][20]; Md[350][463] = Cd[2][22]; Md[350][465] = Cd[2][26]; Md[350][466] = Cd[2][28]; Md[350][467] = Cd[2][31]; 
   Md[351][275] = Cd[2][14]; Md[351][278] = Cd[2][24]; Md[351][280] = Cd[2][30]; Md[351][281] = Cd[2][33]; Md[351][328] = Cd[2][1]; Md[351][330] = Cd[2][2]; Md[351][331] = Cd[2][3]; Md[351][334] = Cd[2][5]; Md[351][336] = Cd[2][6]; Md[351][337] = Cd[2][7]; Md[351][339] = Cd[2][9]; Md[351][340] = Cd[2][10]; Md[351][341] = Cd[2][12]; Md[351][343] = Cd[2][15]; Md[351][345] = Cd[2][16]; Md[351][346] = Cd[2][17]; Md[351][348] = Cd[2][19]; Md[351][349] = Cd[2][20]; Md[351][350] = Cd[2][22]; Md[351][351] = Cd[2][25]; Md[351][352] = Cd[2][26]; Md[351][353] = Cd[2][28]; Md[351][469] = Cd[2][31]; 
   Md[352][276] = Cd[2][14]; Md[352][279] = Cd[2][24]; Md[352][281] = Cd[2][30]; Md[352][282] = Cd[2][33]; Md[352][329] = Cd[2][1]; Md[352][331] = Cd[2][2]; Md[352][332] = Cd[2][3]; Md[352][335] = Cd[2][5]; Md[352][337] = Cd[2][6]; Md[352][338] = Cd[2][7]; Md[352][340] = Cd[2][9]; Md[352][341] = Cd[2][10]; Md[352][344] = Cd[2][15]; Md[352][346] = Cd[2][16]; Md[352][347] = Cd[2][17]; Md[352][349] = Cd[2][19]; Md[352][350] = Cd[2][20]; Md[352][352] = Cd[2][25]; Md[352][353] = Cd[2][26]; Md[352][455] = Cd[2][12]; Md[352][465] = Cd[2][22]; Md[352][469] = Cd[2][28]; Md[352][470] = Cd[2][31]; 
   Md[353][51] = Cd[2][24]; Md[353][108] = Cd[2][14]; Md[353][160] = Cd[2][1]; Md[353][161] = Cd[2][5]; Md[353][248] = Cd[2][15]; Md[353][282] = Cd[2][30]; Md[353][332] = Cd[2][2]; Md[353][338] = Cd[2][6]; Md[353][341] = Cd[2][9]; Md[353][347] = Cd[2][16]; Md[353][350] = Cd[2][19]; Md[353][353] = Cd[2][25]; Md[353][391] = Cd[2][33]; Md[353][446] = Cd[2][3]; Md[353][452] = Cd[2][7]; Md[353][455] = Cd[2][10]; Md[353][456] = Cd[2][12]; Md[353][462] = Cd[2][17]; Md[353][465] = Cd[2][20]; Md[353][466] = Cd[2][22]; Md[353][469] = Cd[2][26]; Md[353][470] = Cd[2][28]; Md[353][471] = Cd[2][31]; 
   Md[354][181] = Cd[2][14]; Md[354][187] = Cd[2][24]; Md[354][188] = Cd[2][30]; Md[354][189] = Cd[2][33]; Md[354][232] = Cd[2][1]; Md[354][233] = Cd[2][2]; Md[354][234] = Cd[2][3]; Md[354][240] = Cd[2][5]; Md[354][241] = Cd[2][6]; Md[354][242] = Cd[2][7]; Md[354][243] = Cd[2][9]; Md[354][244] = Cd[2][10]; Md[354][245] = Cd[2][12]; Md[354][249] = Cd[2][15]; Md[354][250] = Cd[2][16]; Md[354][251] = Cd[2][17]; Md[354][252] = Cd[2][19]; Md[354][253] = Cd[2][20]; Md[354][254] = Cd[2][22]; Md[354][255] = Cd[2][26]; Md[354][256] = Cd[2][28]; Md[354][354] = Cd[2][25]; Md[354][474] = Cd[2][31]; 
   Md[355][182] = Cd[2][14]; Md[355][188] = Cd[2][24]; Md[355][190] = Cd[2][30]; Md[355][191] = Cd[2][33]; Md[355][233] = Cd[2][1]; Md[355][235] = Cd[2][2]; Md[355][236] = Cd[2][3]; Md[355][241] = Cd[2][5]; Md[355][243] = Cd[2][6]; Md[355][244] = Cd[2][7]; Md[355][246] = Cd[2][10]; Md[355][247] = Cd[2][12]; Md[355][250] = Cd[2][15]; Md[355][252] = Cd[2][16]; Md[355][253] = Cd[2][17]; Md[355][255] = Cd[2][20]; Md[355][256] = Cd[2][22]; Md[355][257] = Cd[2][28]; Md[355][342] = Cd[2][9]; Md[355][354] = Cd[2][19]; Md[355][355] = Cd[2][25]; Md[355][356] = Cd[2][26]; Md[355][475] = Cd[2][31]; 
   Md[356][183] = Cd[2][14]; Md[356][189] = Cd[2][24]; Md[356][191] = Cd[2][30]; Md[356][192] = Cd[2][33]; Md[356][234] = Cd[2][1]; Md[356][236] = Cd[2][2]; Md[356][237] = Cd[2][3]; Md[356][242] = Cd[2][5]; Md[356][244] = Cd[2][6]; Md[356][245] = Cd[2][7]; Md[356][246] = Cd[2][9]; Md[356][247] = Cd[2][10]; Md[356][251] = Cd[2][15]; Md[356][253] = Cd[2][16]; Md[356][254] = Cd[2][17]; Md[356][255] = Cd[2][19]; Md[356][256] = Cd[2][20]; Md[356][257] = Cd[2][26]; Md[356][356] = Cd[2][25]; Md[356][459] = Cd[2][12]; Md[356][474] = Cd[2][22]; Md[356][475] = Cd[2][28]; Md[356][476] = Cd[2][31]; 
   Md[357][184] = Cd[2][14]; Md[357][190] = Cd[2][24]; Md[357][193] = Cd[2][33]; Md[357][235] = Cd[2][1]; Md[357][238] = Cd[2][3]; Md[357][243] = Cd[2][5]; Md[357][246] = Cd[2][7]; Md[357][248] = Cd[2][12]; Md[357][252] = Cd[2][15]; Md[357][255] = Cd[2][17]; Md[357][257] = Cd[2][22]; Md[357][283] = Cd[2][30]; Md[357][333] = Cd[2][2]; Md[357][342] = Cd[2][6]; Md[357][343] = Cd[2][9]; Md[357][344] = Cd[2][10]; Md[357][354] = Cd[2][16]; Md[357][355] = Cd[2][19]; Md[357][356] = Cd[2][20]; Md[357][357] = Cd[2][25]; Md[357][358] = Cd[2][26]; Md[357][359] = Cd[2][28]; Md[357][477] = Cd[2][31]; 
   Md[358][185] = Cd[2][14]; Md[358][191] = Cd[2][24]; Md[358][193] = Cd[2][30]; Md[358][194] = Cd[2][33]; Md[358][236] = Cd[2][1]; Md[358][238] = Cd[2][2]; Md[358][239] = Cd[2][3]; Md[358][244] = Cd[2][5]; Md[358][246] = Cd[2][6]; Md[358][247] = Cd[2][7]; Md[358][248] = Cd[2][10]; Md[358][253] = Cd[2][15]; Md[358][255] = Cd[2][16]; Md[358][256] = Cd[2][17]; Md[358][257] = Cd[2][20]; Md[358][344] = Cd[2][9]; Md[358][356] = Cd[2][19]; Md[358][358] = Cd[2][25]; Md[358][359] = Cd[2][26]; Md[358][460] = Cd[2][12]; Md[358][475] = Cd[2][22]; Md[358][477] = Cd[2][28]; Md[358][478] = Cd[2][31]; 
   Md[359][186] = Cd[2][14]; Md[359][192] = Cd[2][24]; Md[359][194] = Cd[2][30]; Md[359][237] = Cd[2][1]; Md[359][239] = Cd[2][2]; Md[359][245] = Cd[2][5]; Md[359][247] = Cd[2][6]; Md[359][248] = Cd[2][9]; Md[359][254] = Cd[2][15]; Md[359][256] = Cd[2][16]; Md[359][257] = Cd[2][19]; Md[359][359] = Cd[2][25]; Md[359][394] = Cd[2][33]; Md[359][449] = Cd[2][3]; Md[359][459] = Cd[2][7]; Md[359][460] = Cd[2][10]; Md[359][461] = Cd[2][12]; Md[359][474] = Cd[2][17]; Md[359][475] = Cd[2][20]; Md[359][476] = Cd[2][22]; Md[359][477] = Cd[2][26]; Md[359][478] = Cd[2][28]; Md[359][479] = Cd[2][31]; 
   Md[360][277] = Cd[2][14]; Md[360][283] = Cd[2][24]; Md[360][284] = Cd[2][30]; Md[360][285] = Cd[2][33]; Md[360][333] = Cd[2][1]; Md[360][334] = Cd[2][2]; Md[360][335] = Cd[2][3]; Md[360][342] = Cd[2][5]; Md[360][343] = Cd[2][6]; Md[360][344] = Cd[2][7]; Md[360][345] = Cd[2][9]; Md[360][346] = Cd[2][10]; Md[360][347] = Cd[2][12]; Md[360][354] = Cd[2][15]; Md[360][355] = Cd[2][16]; Md[360][356] = Cd[2][17]; Md[360][357] = Cd[2][19]; Md[360][358] = Cd[2][20]; Md[360][359] = Cd[2][22]; Md[360][360] = Cd[2][25]; Md[360][361] = Cd[2][26]; Md[360][362] = Cd[2][28]; Md[360][480] = Cd[2][31]; 
   Md[361][49] = Cd[2][14]; Md[361][52] = Cd[2][33]; Md[361][161] = Cd[2][3]; Md[361][193] = Cd[2][24]; Md[361][238] = Cd[2][1]; Md[361][246] = Cd[2][5]; Md[361][248] = Cd[2][7]; Md[361][255] = Cd[2][15]; Md[361][257] = Cd[2][17]; Md[361][285] = Cd[2][30]; Md[361][335] = Cd[2][2]; Md[361][344] = Cd[2][6]; Md[361][346] = Cd[2][9]; Md[361][347] = Cd[2][10]; Md[361][356] = Cd[2][16]; Md[361][358] = Cd[2][19]; Md[361][359] = Cd[2][20]; Md[361][361] = Cd[2][25]; Md[361][362] = Cd[2][26]; Md[361][462] = Cd[2][12]; Md[361][477] = Cd[2][22]; Md[361][480] = Cd[2][28]; Md[361][481] = Cd[2][31]; 
   Md[362][50] = Cd[2][14]; Md[362][52] = Cd[2][30]; Md[362][161] = Cd[2][2]; Md[362][194] = Cd[2][24]; Md[362][239] = Cd[2][1]; Md[362][247] = Cd[2][5]; Md[362][248] = Cd[2][6]; Md[362][256] = Cd[2][15]; Md[362][257] = Cd[2][16]; Md[362][347] = Cd[2][9]; Md[362][359] = Cd[2][19]; Md[362][362] = Cd[2][25]; Md[362][395] = Cd[2][33]; Md[362][450] = Cd[2][3]; Md[362][460] = Cd[2][7]; Md[362][462] = Cd[2][10]; Md[362][463] = Cd[2][12]; Md[362][475] = Cd[2][17]; Md[362][477] = Cd[2][20]; Md[362][478] = Cd[2][22]; Md[362][480] = Cd[2][26]; Md[362][481] = Cd[2][28]; Md[362][482] = Cd[2][31]; 
   Md[363][278] = Cd[2][14]; Md[363][284] = Cd[2][24]; Md[363][286] = Cd[2][30]; Md[363][287] = Cd[2][33]; Md[363][334] = Cd[2][1]; Md[363][336] = Cd[2][2]; Md[363][337] = Cd[2][3]; Md[363][343] = Cd[2][5]; Md[363][345] = Cd[2][6]; Md[363][346] = Cd[2][7]; Md[363][348] = Cd[2][9]; Md[363][349] = Cd[2][10]; Md[363][350] = Cd[2][12]; Md[363][355] = Cd[2][15]; Md[363][357] = Cd[2][16]; Md[363][358] = Cd[2][17]; Md[363][360] = Cd[2][19]; Md[363][361] = Cd[2][20]; Md[363][362] = Cd[2][22]; Md[363][363] = Cd[2][25]; Md[363][364] = Cd[2][26]; Md[363][365] = Cd[2][28]; Md[363][484] = Cd[2][31]; 
   Md[364][279] = Cd[2][14]; Md[364][285] = Cd[2][24]; Md[364][287] = Cd[2][30]; Md[364][288] = Cd[2][33]; Md[364][335] = Cd[2][1]; Md[364][337] = Cd[2][2]; Md[364][338] = Cd[2][3]; Md[364][344] = Cd[2][5]; Md[364][346] = Cd[2][6]; Md[364][347] = Cd[2][7]; Md[364][349] = Cd[2][9]; Md[364][350] = Cd[2][10]; Md[364][356] = Cd[2][15]; Md[364][358] = Cd[2][16]; Md[364][359] = Cd[2][17]; Md[364][361] = Cd[2][19]; Md[364][362] = Cd[2][20]; Md[364][364] = Cd[2][25]; Md[364][365] = Cd[2][26]; Md[364][465] = Cd[2][12]; Md[364][480] = Cd[2][22]; Md[364][484] = Cd[2][28]; Md[364][485] = Cd[2][31]; 
   Md[365][51] = Cd[2][14]; Md[365][52] = Cd[2][24]; Md[365][161] = Cd[2][1]; Md[365][248] = Cd[2][5]; Md[365][257] = Cd[2][15]; Md[365][288] = Cd[2][30]; Md[365][338] = Cd[2][2]; Md[365][347] = Cd[2][6]; Md[365][350] = Cd[2][9]; Md[365][359] = Cd[2][16]; Md[365][362] = Cd[2][19]; Md[365][365] = Cd[2][25]; Md[365][397] = Cd[2][33]; Md[365][452] = Cd[2][3]; Md[365][462] = Cd[2][7]; Md[365][465] = Cd[2][10]; Md[365][466] = Cd[2][12]; Md[365][477] = Cd[2][17]; Md[365][480] = Cd[2][20]; Md[365][481] = Cd[2][22]; Md[365][484] = Cd[2][26]; Md[365][485] = Cd[2][28]; Md[365][486] = Cd[2][31]; 
   Md[366][280] = Cd[2][14]; Md[366][286] = Cd[2][24]; Md[366][289] = Cd[2][30]; Md[366][290] = Cd[2][33]; Md[366][336] = Cd[2][1]; Md[366][339] = Cd[2][2]; Md[366][340] = Cd[2][3]; Md[366][345] = Cd[2][5]; Md[366][348] = Cd[2][6]; Md[366][349] = Cd[2][7]; Md[366][351] = Cd[2][9]; Md[366][352] = Cd[2][10]; Md[366][353] = Cd[2][12]; Md[366][357] = Cd[2][15]; Md[366][360] = Cd[2][16]; Md[366][361] = Cd[2][17]; Md[366][363] = Cd[2][19]; Md[366][364] = Cd[2][20]; Md[366][365] = Cd[2][22]; Md[366][366] = Cd[2][25]; Md[366][367] = Cd[2][26]; Md[366][368] = Cd[2][28]; Md[366][489] = Cd[2][31]; 
   Md[367][281] = Cd[2][14]; Md[367][287] = Cd[2][24]; Md[367][290] = Cd[2][30]; Md[367][291] = Cd[2][33]; Md[367][337] = Cd[2][1]; Md[367][340] = Cd[2][2]; Md[367][341] = Cd[2][3]; Md[367][346] = Cd[2][5]; Md[367][349] = Cd[2][6]; Md[367][350] = Cd[2][7]; Md[367][352] = Cd[2][9]; Md[367][353] = Cd[2][10]; Md[367][358] = Cd[2][15]; Md[367][361] = Cd[2][16]; Md[367][362] = Cd[2][17]; Md[367][364] = Cd[2][19]; Md[367][365] = Cd[2][20]; Md[367][367] = Cd[2][25]; Md[367][368] = Cd[2][26]; Md[367][469] = Cd[2][12]; Md[367][484] = Cd[2][22]; Md[367][489] = Cd[2][28]; Md[367][490] = Cd[2][31]; 
   Md[368][282] = Cd[2][14]; Md[368][288] = Cd[2][24]; Md[368][291] = Cd[2][30]; Md[368][338] = Cd[2][1]; Md[368][341] = Cd[2][2]; Md[368][347] = Cd[2][5]; Md[368][350] = Cd[2][6]; Md[368][353] = Cd[2][9]; Md[368][359] = Cd[2][15]; Md[368][362] = Cd[2][16]; Md[368][365] = Cd[2][19]; Md[368][368] = Cd[2][25]; Md[368][400] = Cd[2][33]; Md[368][455] = Cd[2][3]; Md[368][465] = Cd[2][7]; Md[368][469] = Cd[2][10]; Md[368][470] = Cd[2][12]; Md[368][480] = Cd[2][17]; Md[368][484] = Cd[2][20]; Md[368][485] = Cd[2][22]; Md[368][489] = Cd[2][26]; Md[368][490] = Cd[2][28]; Md[368][491] = Cd[2][31]; 
   Md[369][1] = Cd[3][14]; Md[369][2] = Cd[3][24]; Md[369][3] = Cd[3][30]; Md[369][4] = Cd[3][33]; Md[369][14] = Cd[3][5]; Md[369][15] = Cd[3][6]; Md[369][16] = Cd[3][7]; Md[369][17] = Cd[3][9]; Md[369][18] = Cd[3][10]; Md[369][19] = Cd[3][12]; Md[369][20] = Cd[3][16]; Md[369][21] = Cd[3][17]; Md[369][22] = Cd[3][19]; Md[369][23] = Cd[3][20]; Md[369][24] = Cd[3][22]; Md[369][25] = Cd[3][26]; Md[369][26] = Cd[3][28]; Md[369][56] = Cd[3][1]; Md[369][57] = Cd[3][2]; Md[369][58] = Cd[3][3]; Md[369][162] = Cd[3][15]; Md[369][258] = Cd[3][25]; Md[369][369] = Cd[3][31]; 
   Md[370][5] = Cd[3][24]; Md[370][6] = Cd[3][30]; Md[370][7] = Cd[3][33]; Md[370][27] = Cd[3][16]; Md[370][28] = Cd[3][17]; Md[370][29] = Cd[3][19]; Md[370][30] = Cd[3][20]; Md[370][31] = Cd[3][22]; Md[370][32] = Cd[3][26]; Md[370][33] = Cd[3][28]; Md[370][54] = Cd[3][14]; Md[370][60] = Cd[3][1]; Md[370][61] = Cd[3][2]; Md[370][62] = Cd[3][3]; Md[370][63] = Cd[3][5]; Md[370][64] = Cd[3][6]; Md[370][65] = Cd[3][7]; Md[370][66] = Cd[3][9]; Md[370][67] = Cd[3][10]; Md[370][68] = Cd[3][12]; Md[370][163] = Cd[3][15]; Md[370][259] = Cd[3][25]; Md[370][370] = Cd[3][31]; 
   Md[371][5] = Cd[3][14]; Md[371][8] = Cd[3][24]; Md[371][9] = Cd[3][30]; Md[371][10] = Cd[3][33]; Md[371][27] = Cd[3][6]; Md[371][28] = Cd[3][7]; Md[371][29] = Cd[3][9]; Md[371][30] = Cd[3][10]; Md[371][31] = Cd[3][12]; Md[371][34] = Cd[3][19]; Md[371][35] = Cd[3][20]; Md[371][36] = Cd[3][22]; Md[371][37] = Cd[3][26]; Md[371][38] = Cd[3][28]; Md[371][63] = Cd[3][1]; Md[371][64] = Cd[3][2]; Md[371][65] = Cd[3][3]; Md[371][163] = Cd[3][5]; Md[371][164] = Cd[3][15]; Md[371][165] = Cd[3][16]; Md[371][166] = Cd[3][17]; Md[371][260] = Cd[3][25]; Md[371][371] = Cd[3][31]; 
   Md[372][6] = Cd[3][14]; Md[372][9] = Cd[3][24]; Md[372][11] = Cd[3][30]; Md[372][12] = Cd[3][33]; Md[372][27] = Cd[3][5]; Md[372][29] = Cd[3][6]; Md[372][30] = Cd[3][7]; Md[372][32] = Cd[3][10]; Md[372][33] = Cd[3][12]; Md[372][34] = Cd[3][16]; Md[372][35] = Cd[3][17]; Md[372][37] = Cd[3][20]; Md[372][38] = Cd[3][22]; Md[372][39] = Cd[3][28]; Md[372][64] = Cd[3][1]; Md[372][66] = Cd[3][2]; Md[372][67] = Cd[3][3]; Md[372][165] = Cd[3][15]; Md[372][259] = Cd[3][9]; Md[372][260] = Cd[3][19]; Md[372][261] = Cd[3][25]; Md[372][262] = Cd[3][26]; Md[372][372] = Cd[3][31]; 
   Md[373][7] = Cd[3][14]; Md[373][10] = Cd[3][24]; Md[373][12] = Cd[3][30]; Md[373][13] = Cd[3][33]; Md[373][28] = Cd[3][5]; Md[373][30] = Cd[3][6]; Md[373][31] = Cd[3][7]; Md[373][32] = Cd[3][9]; Md[373][33] = Cd[3][10]; Md[373][35] = Cd[3][16]; Md[373][36] = Cd[3][17]; Md[373][37] = Cd[3][19]; Md[373][38] = Cd[3][20]; Md[373][39] = Cd[3][26]; Md[373][65] = Cd[3][1]; Md[373][67] = Cd[3][2]; Md[373][68] = Cd[3][3]; Md[373][166] = Cd[3][15]; Md[373][262] = Cd[3][25]; Md[373][370] = Cd[3][12]; Md[373][371] = Cd[3][22]; Md[373][372] = Cd[3][28]; Md[373][373] = Cd[3][31]; 
   Md[374][55] = Cd[3][14]; Md[374][56] = Cd[3][24]; Md[374][57] = Cd[3][30]; Md[374][58] = Cd[3][33]; Md[374][70] = Cd[3][1]; Md[374][71] = Cd[3][2]; Md[374][72] = Cd[3][3]; Md[374][73] = Cd[3][5]; Md[374][74] = Cd[3][6]; Md[374][75] = Cd[3][7]; Md[374][76] = Cd[3][9]; Md[374][77] = Cd[3][10]; Md[374][78] = Cd[3][12]; Md[374][79] = Cd[3][16]; Md[374][80] = Cd[3][17]; Md[374][81] = Cd[3][19]; Md[374][82] = Cd[3][20]; Md[374][83] = Cd[3][22]; Md[374][84] = Cd[3][26]; Md[374][85] = Cd[3][28]; Md[374][167] = Cd[3][15]; Md[374][263] = Cd[3][25]; Md[374][374] = Cd[3][31]; 
   Md[375][14] = Cd[3][24]; Md[375][15] = Cd[3][30]; Md[375][16] = Cd[3][33]; Md[375][40] = Cd[3][19]; Md[375][41] = Cd[3][20]; Md[375][42] = Cd[3][22]; Md[375][43] = Cd[3][26]; Md[375][44] = Cd[3][28]; Md[375][56] = Cd[3][14]; Md[375][73] = Cd[3][1]; Md[375][74] = Cd[3][2]; Md[375][75] = Cd[3][3]; Md[375][79] = Cd[3][6]; Md[375][80] = Cd[3][7]; Md[375][81] = Cd[3][9]; Md[375][82] = Cd[3][10]; Md[375][83] = Cd[3][12]; Md[375][167] = Cd[3][5]; Md[375][168] = Cd[3][15]; Md[375][169] = Cd[3][16]; Md[375][170] = Cd[3][17]; Md[375][264] = Cd[3][25]; Md[375][375] = Cd[3][31]; 
   Md[376][15] = Cd[3][24]; Md[376][17] = Cd[3][30]; Md[376][18] = Cd[3][33]; Md[376][40] = Cd[3][16]; Md[376][41] = Cd[3][17]; Md[376][43] = Cd[3][20]; Md[376][44] = Cd[3][22]; Md[376][45] = Cd[3][28]; Md[376][57] = Cd[3][14]; Md[376][74] = Cd[3][1]; Md[376][76] = Cd[3][2]; Md[376][77] = Cd[3][3]; Md[376][79] = Cd[3][5]; Md[376][81] = Cd[3][6]; Md[376][82] = Cd[3][7]; Md[376][84] = Cd[3][10]; Md[376][85] = Cd[3][12]; Md[376][169] = Cd[3][15]; Md[376][263] = Cd[3][9]; Md[376][264] = Cd[3][19]; Md[376][265] = Cd[3][25]; Md[376][266] = Cd[3][26]; Md[376][376] = Cd[3][31]; 
   Md[377][16] = Cd[3][24]; Md[377][18] = Cd[3][30]; Md[377][19] = Cd[3][33]; Md[377][41] = Cd[3][16]; Md[377][42] = Cd[3][17]; Md[377][43] = Cd[3][19]; Md[377][44] = Cd[3][20]; Md[377][45] = Cd[3][26]; Md[377][58] = Cd[3][14]; Md[377][75] = Cd[3][1]; Md[377][77] = Cd[3][2]; Md[377][78] = Cd[3][3]; Md[377][80] = Cd[3][5]; Md[377][82] = Cd[3][6]; Md[377][83] = Cd[3][7]; Md[377][84] = Cd[3][9]; Md[377][85] = Cd[3][10]; Md[377][170] = Cd[3][15]; Md[377][266] = Cd[3][25]; Md[377][374] = Cd[3][12]; Md[377][375] = Cd[3][22]; Md[377][376] = Cd[3][28]; Md[377][377] = Cd[3][31]; 
   Md[378][14] = Cd[3][14]; Md[378][20] = Cd[3][30]; Md[378][21] = Cd[3][33]; Md[378][40] = Cd[3][9]; Md[378][41] = Cd[3][10]; Md[378][42] = Cd[3][12]; Md[378][46] = Cd[3][26]; Md[378][47] = Cd[3][28]; Md[378][79] = Cd[3][2]; Md[378][80] = Cd[3][3]; Md[378][162] = Cd[3][24]; Md[378][167] = Cd[3][1]; Md[378][168] = Cd[3][5]; Md[378][169] = Cd[3][6]; Md[378][170] = Cd[3][7]; Md[378][171] = Cd[3][15]; Md[378][172] = Cd[3][16]; Md[378][173] = Cd[3][17]; Md[378][174] = Cd[3][19]; Md[378][175] = Cd[3][20]; Md[378][176] = Cd[3][22]; Md[378][267] = Cd[3][25]; Md[378][378] = Cd[3][31]; 
   Md[379][15] = Cd[3][14]; Md[379][20] = Cd[3][24]; Md[379][22] = Cd[3][30]; Md[379][23] = Cd[3][33]; Md[379][40] = Cd[3][6]; Md[379][41] = Cd[3][7]; Md[379][43] = Cd[3][10]; Md[379][44] = Cd[3][12]; Md[379][46] = Cd[3][20]; Md[379][47] = Cd[3][22]; Md[379][48] = Cd[3][28]; Md[379][79] = Cd[3][1]; Md[379][81] = Cd[3][2]; Md[379][82] = Cd[3][3]; Md[379][169] = Cd[3][5]; Md[379][172] = Cd[3][15]; Md[379][174] = Cd[3][16]; Md[379][175] = Cd[3][17]; Md[379][264] = Cd[3][9]; Md[379][267] = Cd[3][19]; Md[379][268] = Cd[3][25]; Md[379][269] = Cd[3][26]; Md[379][379] = Cd[3][31]; 
   Md[380][16] = Cd[3][14]; Md[380][21] = Cd[3][24]; Md[380][23] = Cd[3][30]; Md[380][24] = Cd[3][33]; Md[380][41] = Cd[3][6]; Md[380][42] = Cd[3][7]; Md[380][43] = Cd[3][9]; Md[380][44] = Cd[3][10]; Md[380][46] = Cd[3][19]; Md[380][47] = Cd[3][20]; Md[380][48] = Cd[3][26]; Md[380][80] = Cd[3][1]; Md[380][82] = Cd[3][2]; Md[380][83] = Cd[3][3]; Md[380][170] = Cd[3][5]; Md[380][173] = Cd[3][15]; Md[380][175] = Cd[3][16]; Md[380][176] = Cd[3][17]; Md[380][269] = Cd[3][25]; Md[380][375] = Cd[3][12]; Md[380][378] = Cd[3][22]; Md[380][379] = Cd[3][28]; Md[380][380] = Cd[3][31]; 
   Md[381][17] = Cd[3][14]; Md[381][22] = Cd[3][24]; Md[381][25] = Cd[3][33]; Md[381][40] = Cd[3][5]; Md[381][43] = Cd[3][7]; Md[381][45] = Cd[3][12]; Md[381][46] = Cd[3][17]; Md[381][48] = Cd[3][22]; Md[381][81] = Cd[3][1]; Md[381][84] = Cd[3][3]; Md[381][174] = Cd[3][15]; Md[381][258] = Cd[3][30]; Md[381][263] = Cd[3][2]; Md[381][264] = Cd[3][6]; Md[381][265] = Cd[3][9]; Md[381][266] = Cd[3][10]; Md[381][267] = Cd[3][16]; Md[381][268] = Cd[3][19]; Md[381][269] = Cd[3][20]; Md[381][270] = Cd[3][25]; Md[381][271] = Cd[3][26]; Md[381][272] = Cd[3][28]; Md[381][381] = Cd[3][31]; 
   Md[382][18] = Cd[3][14]; Md[382][23] = Cd[3][24]; Md[382][25] = Cd[3][30]; Md[382][26] = Cd[3][33]; Md[382][41] = Cd[3][5]; Md[382][43] = Cd[3][6]; Md[382][44] = Cd[3][7]; Md[382][45] = Cd[3][10]; Md[382][46] = Cd[3][16]; Md[382][47] = Cd[3][17]; Md[382][48] = Cd[3][20]; Md[382][82] = Cd[3][1]; Md[382][84] = Cd[3][2]; Md[382][85] = Cd[3][3]; Md[382][175] = Cd[3][15]; Md[382][266] = Cd[3][9]; Md[382][269] = Cd[3][19]; Md[382][271] = Cd[3][25]; Md[382][272] = Cd[3][26]; Md[382][376] = Cd[3][12]; Md[382][379] = Cd[3][22]; Md[382][381] = Cd[3][28]; Md[382][382] = Cd[3][31]; 
   Md[383][19] = Cd[3][14]; Md[383][24] = Cd[3][24]; Md[383][26] = Cd[3][30]; Md[383][42] = Cd[3][5]; Md[383][44] = Cd[3][6]; Md[383][45] = Cd[3][9]; Md[383][47] = Cd[3][16]; Md[383][48] = Cd[3][19]; Md[383][83] = Cd[3][1]; Md[383][85] = Cd[3][2]; Md[383][176] = Cd[3][15]; Md[383][272] = Cd[3][25]; Md[383][369] = Cd[3][33]; Md[383][374] = Cd[3][3]; Md[383][375] = Cd[3][7]; Md[383][376] = Cd[3][10]; Md[383][377] = Cd[3][12]; Md[383][378] = Cd[3][17]; Md[383][379] = Cd[3][20]; Md[383][380] = Cd[3][22]; Md[383][381] = Cd[3][26]; Md[383][382] = Cd[3][28]; Md[383][383] = Cd[3][31]; 
   Md[384][59] = Cd[3][14]; Md[384][60] = Cd[3][24]; Md[384][61] = Cd[3][30]; Md[384][62] = Cd[3][33]; Md[384][87] = Cd[3][1]; Md[384][88] = Cd[3][2]; Md[384][89] = Cd[3][3]; Md[384][90] = Cd[3][5]; Md[384][91] = Cd[3][6]; Md[384][92] = Cd[3][7]; Md[384][93] = Cd[3][9]; Md[384][94] = Cd[3][10]; Md[384][95] = Cd[3][12]; Md[384][96] = Cd[3][16]; Md[384][97] = Cd[3][17]; Md[384][98] = Cd[3][19]; Md[384][99] = Cd[3][20]; Md[384][100] = Cd[3][22]; Md[384][101] = Cd[3][26]; Md[384][102] = Cd[3][28]; Md[384][177] = Cd[3][15]; Md[384][273] = Cd[3][25]; Md[384][384] = Cd[3][31]; 
   Md[385][60] = Cd[3][14]; Md[385][63] = Cd[3][24]; Md[385][64] = Cd[3][30]; Md[385][65] = Cd[3][33]; Md[385][90] = Cd[3][1]; Md[385][91] = Cd[3][2]; Md[385][92] = Cd[3][3]; Md[385][96] = Cd[3][6]; Md[385][97] = Cd[3][7]; Md[385][98] = Cd[3][9]; Md[385][99] = Cd[3][10]; Md[385][100] = Cd[3][12]; Md[385][103] = Cd[3][19]; Md[385][104] = Cd[3][20]; Md[385][105] = Cd[3][22]; Md[385][106] = Cd[3][26]; Md[385][107] = Cd[3][28]; Md[385][177] = Cd[3][5]; Md[385][178] = Cd[3][15]; Md[385][179] = Cd[3][16]; Md[385][180] = Cd[3][17]; Md[385][274] = Cd[3][25]; Md[385][385] = Cd[3][31]; 
   Md[386][61] = Cd[3][14]; Md[386][64] = Cd[3][24]; Md[386][66] = Cd[3][30]; Md[386][67] = Cd[3][33]; Md[386][91] = Cd[3][1]; Md[386][93] = Cd[3][2]; Md[386][94] = Cd[3][3]; Md[386][96] = Cd[3][5]; Md[386][98] = Cd[3][6]; Md[386][99] = Cd[3][7]; Md[386][101] = Cd[3][10]; Md[386][102] = Cd[3][12]; Md[386][103] = Cd[3][16]; Md[386][104] = Cd[3][17]; Md[386][106] = Cd[3][20]; Md[386][107] = Cd[3][22]; Md[386][108] = Cd[3][28]; Md[386][179] = Cd[3][15]; Md[386][273] = Cd[3][9]; Md[386][274] = Cd[3][19]; Md[386][275] = Cd[3][25]; Md[386][276] = Cd[3][26]; Md[386][386] = Cd[3][31]; 
   Md[387][62] = Cd[3][14]; Md[387][65] = Cd[3][24]; Md[387][67] = Cd[3][30]; Md[387][68] = Cd[3][33]; Md[387][92] = Cd[3][1]; Md[387][94] = Cd[3][2]; Md[387][95] = Cd[3][3]; Md[387][97] = Cd[3][5]; Md[387][99] = Cd[3][6]; Md[387][100] = Cd[3][7]; Md[387][101] = Cd[3][9]; Md[387][102] = Cd[3][10]; Md[387][104] = Cd[3][16]; Md[387][105] = Cd[3][17]; Md[387][106] = Cd[3][19]; Md[387][107] = Cd[3][20]; Md[387][108] = Cd[3][26]; Md[387][180] = Cd[3][15]; Md[387][276] = Cd[3][25]; Md[387][384] = Cd[3][12]; Md[387][385] = Cd[3][22]; Md[387][386] = Cd[3][28]; Md[387][387] = Cd[3][31]; 
   Md[388][27] = Cd[3][30]; Md[388][28] = Cd[3][33]; Md[388][49] = Cd[3][26]; Md[388][50] = Cd[3][28]; Md[388][63] = Cd[3][14]; Md[388][96] = Cd[3][2]; Md[388][97] = Cd[3][3]; Md[388][103] = Cd[3][9]; Md[388][104] = Cd[3][10]; Md[388][105] = Cd[3][12]; Md[388][163] = Cd[3][24]; Md[388][177] = Cd[3][1]; Md[388][178] = Cd[3][5]; Md[388][179] = Cd[3][6]; Md[388][180] = Cd[3][7]; Md[388][181] = Cd[3][15]; Md[388][182] = Cd[3][16]; Md[388][183] = Cd[3][17]; Md[388][184] = Cd[3][19]; Md[388][185] = Cd[3][20]; Md[388][186] = Cd[3][22]; Md[388][277] = Cd[3][25]; Md[388][388] = Cd[3][31]; 
   Md[389][27] = Cd[3][24]; Md[389][29] = Cd[3][30]; Md[389][30] = Cd[3][33]; Md[389][49] = Cd[3][20]; Md[389][50] = Cd[3][22]; Md[389][51] = Cd[3][28]; Md[389][64] = Cd[3][14]; Md[389][96] = Cd[3][1]; Md[389][98] = Cd[3][2]; Md[389][99] = Cd[3][3]; Md[389][103] = Cd[3][6]; Md[389][104] = Cd[3][7]; Md[389][106] = Cd[3][10]; Md[389][107] = Cd[3][12]; Md[389][179] = Cd[3][5]; Md[389][182] = Cd[3][15]; Md[389][184] = Cd[3][16]; Md[389][185] = Cd[3][17]; Md[389][274] = Cd[3][9]; Md[389][277] = Cd[3][19]; Md[389][278] = Cd[3][25]; Md[389][279] = Cd[3][26]; Md[389][389] = Cd[3][31]; 
   Md[390][28] = Cd[3][24]; Md[390][30] = Cd[3][30]; Md[390][31] = Cd[3][33]; Md[390][49] = Cd[3][19]; Md[390][50] = Cd[3][20]; Md[390][51] = Cd[3][26]; Md[390][65] = Cd[3][14]; Md[390][97] = Cd[3][1]; Md[390][99] = Cd[3][2]; Md[390][100] = Cd[3][3]; Md[390][104] = Cd[3][6]; Md[390][105] = Cd[3][7]; Md[390][106] = Cd[3][9]; Md[390][107] = Cd[3][10]; Md[390][180] = Cd[3][5]; Md[390][183] = Cd[3][15]; Md[390][185] = Cd[3][16]; Md[390][186] = Cd[3][17]; Md[390][279] = Cd[3][25]; Md[390][385] = Cd[3][12]; Md[390][388] = Cd[3][22]; Md[390][389] = Cd[3][28]; Md[390][390] = Cd[3][31]; 
   Md[391][29] = Cd[3][24]; Md[391][32] = Cd[3][33]; Md[391][49] = Cd[3][17]; Md[391][51] = Cd[3][22]; Md[391][66] = Cd[3][14]; Md[391][98] = Cd[3][1]; Md[391][101] = Cd[3][3]; Md[391][103] = Cd[3][5]; Md[391][106] = Cd[3][7]; Md[391][108] = Cd[3][12]; Md[391][184] = Cd[3][15]; Md[391][259] = Cd[3][30]; Md[391][273] = Cd[3][2]; Md[391][274] = Cd[3][6]; Md[391][275] = Cd[3][9]; Md[391][276] = Cd[3][10]; Md[391][277] = Cd[3][16]; Md[391][278] = Cd[3][19]; Md[391][279] = Cd[3][20]; Md[391][280] = Cd[3][25]; Md[391][281] = Cd[3][26]; Md[391][282] = Cd[3][28]; Md[391][391] = Cd[3][31]; 
   Md[392][30] = Cd[3][24]; Md[392][32] = Cd[3][30]; Md[392][33] = Cd[3][33]; Md[392][49] = Cd[3][16]; Md[392][50] = Cd[3][17]; Md[392][51] = Cd[3][20]; Md[392][67] = Cd[3][14]; Md[392][99] = Cd[3][1]; Md[392][101] = Cd[3][2]; Md[392][102] = Cd[3][3]; Md[392][104] = Cd[3][5]; Md[392][106] = Cd[3][6]; Md[392][107] = Cd[3][7]; Md[392][108] = Cd[3][10]; Md[392][185] = Cd[3][15]; Md[392][276] = Cd[3][9]; Md[392][279] = Cd[3][19]; Md[392][281] = Cd[3][25]; Md[392][282] = Cd[3][26]; Md[392][386] = Cd[3][12]; Md[392][389] = Cd[3][22]; Md[392][391] = Cd[3][28]; Md[392][392] = Cd[3][31]; 
   Md[393][31] = Cd[3][24]; Md[393][33] = Cd[3][30]; Md[393][50] = Cd[3][16]; Md[393][51] = Cd[3][19]; Md[393][68] = Cd[3][14]; Md[393][100] = Cd[3][1]; Md[393][102] = Cd[3][2]; Md[393][105] = Cd[3][5]; Md[393][107] = Cd[3][6]; Md[393][108] = Cd[3][9]; Md[393][186] = Cd[3][15]; Md[393][282] = Cd[3][25]; Md[393][370] = Cd[3][33]; Md[393][384] = Cd[3][3]; Md[393][385] = Cd[3][7]; Md[393][386] = Cd[3][10]; Md[393][387] = Cd[3][12]; Md[393][388] = Cd[3][17]; Md[393][389] = Cd[3][20]; Md[393][390] = Cd[3][22]; Md[393][391] = Cd[3][26]; Md[393][392] = Cd[3][28]; Md[393][393] = Cd[3][31]; 
   Md[394][163] = Cd[3][14]; Md[394][164] = Cd[3][24]; Md[394][165] = Cd[3][30]; Md[394][166] = Cd[3][33]; Md[394][178] = Cd[3][1]; Md[394][179] = Cd[3][2]; Md[394][180] = Cd[3][3]; Md[394][181] = Cd[3][5]; Md[394][182] = Cd[3][6]; Md[394][183] = Cd[3][7]; Md[394][184] = Cd[3][9]; Md[394][185] = Cd[3][10]; Md[394][186] = Cd[3][12]; Md[394][187] = Cd[3][15]; Md[394][188] = Cd[3][16]; Md[394][189] = Cd[3][17]; Md[394][190] = Cd[3][19]; Md[394][191] = Cd[3][20]; Md[394][192] = Cd[3][22]; Md[394][193] = Cd[3][26]; Md[394][194] = Cd[3][28]; Md[394][283] = Cd[3][25]; Md[394][394] = Cd[3][31]; 
   Md[395][27] = Cd[3][14]; Md[395][34] = Cd[3][30]; Md[395][35] = Cd[3][33]; Md[395][49] = Cd[3][10]; Md[395][50] = Cd[3][12]; Md[395][52] = Cd[3][28]; Md[395][103] = Cd[3][2]; Md[395][104] = Cd[3][3]; Md[395][165] = Cd[3][24]; Md[395][179] = Cd[3][1]; Md[395][182] = Cd[3][5]; Md[395][184] = Cd[3][6]; Md[395][185] = Cd[3][7]; Md[395][188] = Cd[3][15]; Md[395][190] = Cd[3][16]; Md[395][191] = Cd[3][17]; Md[395][193] = Cd[3][20]; Md[395][194] = Cd[3][22]; Md[395][277] = Cd[3][9]; Md[395][283] = Cd[3][19]; Md[395][284] = Cd[3][25]; Md[395][285] = Cd[3][26]; Md[395][395] = Cd[3][31]; 
   Md[396][28] = Cd[3][14]; Md[396][35] = Cd[3][30]; Md[396][36] = Cd[3][33]; Md[396][49] = Cd[3][9]; Md[396][50] = Cd[3][10]; Md[396][52] = Cd[3][26]; Md[396][104] = Cd[3][2]; Md[396][105] = Cd[3][3]; Md[396][166] = Cd[3][24]; Md[396][180] = Cd[3][1]; Md[396][183] = Cd[3][5]; Md[396][185] = Cd[3][6]; Md[396][186] = Cd[3][7]; Md[396][189] = Cd[3][15]; Md[396][191] = Cd[3][16]; Md[396][192] = Cd[3][17]; Md[396][193] = Cd[3][19]; Md[396][194] = Cd[3][20]; Md[396][285] = Cd[3][25]; Md[396][388] = Cd[3][12]; Md[396][394] = Cd[3][22]; Md[396][395] = Cd[3][28]; Md[396][396] = Cd[3][31]; 
   Md[397][29] = Cd[3][14]; Md[397][34] = Cd[3][24]; Md[397][37] = Cd[3][33]; Md[397][49] = Cd[3][7]; Md[397][51] = Cd[3][12]; Md[397][52] = Cd[3][22]; Md[397][103] = Cd[3][1]; Md[397][106] = Cd[3][3]; Md[397][184] = Cd[3][5]; Md[397][190] = Cd[3][15]; Md[397][193] = Cd[3][17]; Md[397][260] = Cd[3][30]; Md[397][274] = Cd[3][2]; Md[397][277] = Cd[3][6]; Md[397][278] = Cd[3][9]; Md[397][279] = Cd[3][10]; Md[397][283] = Cd[3][16]; Md[397][284] = Cd[3][19]; Md[397][285] = Cd[3][20]; Md[397][286] = Cd[3][25]; Md[397][287] = Cd[3][26]; Md[397][288] = Cd[3][28]; Md[397][397] = Cd[3][31]; 
   Md[398][30] = Cd[3][14]; Md[398][35] = Cd[3][24]; Md[398][37] = Cd[3][30]; Md[398][38] = Cd[3][33]; Md[398][49] = Cd[3][6]; Md[398][50] = Cd[3][7]; Md[398][51] = Cd[3][10]; Md[398][52] = Cd[3][20]; Md[398][104] = Cd[3][1]; Md[398][106] = Cd[3][2]; Md[398][107] = Cd[3][3]; Md[398][185] = Cd[3][5]; Md[398][191] = Cd[3][15]; Md[398][193] = Cd[3][16]; Md[398][194] = Cd[3][17]; Md[398][279] = Cd[3][9]; Md[398][285] = Cd[3][19]; Md[398][287] = Cd[3][25]; Md[398][288] = Cd[3][26]; Md[398][389] = Cd[3][12]; Md[398][395] = Cd[3][22]; Md[398][397] = Cd[3][28]; Md[398][398] = Cd[3][31]; 
   Md[399][31] = Cd[3][14]; Md[399][36] = Cd[3][24]; Md[399][38] = Cd[3][30]; Md[399][50] = Cd[3][6]; Md[399][51] = Cd[3][9]; Md[399][52] = Cd[3][19]; Md[399][105] = Cd[3][1]; Md[399][107] = Cd[3][2]; Md[399][186] = Cd[3][5]; Md[399][192] = Cd[3][15]; Md[399][194] = Cd[3][16]; Md[399][288] = Cd[3][25]; Md[399][371] = Cd[3][33]; Md[399][385] = Cd[3][3]; Md[399][388] = Cd[3][7]; Md[399][389] = Cd[3][10]; Md[399][390] = Cd[3][12]; Md[399][394] = Cd[3][17]; Md[399][395] = Cd[3][20]; Md[399][396] = Cd[3][22]; Md[399][397] = Cd[3][26]; Md[399][398] = Cd[3][28]; Md[399][399] = Cd[3][31]; 
   Md[400][259] = Cd[3][14]; Md[400][260] = Cd[3][24]; Md[400][261] = Cd[3][30]; Md[400][262] = Cd[3][33]; Md[400][274] = Cd[3][1]; Md[400][275] = Cd[3][2]; Md[400][276] = Cd[3][3]; Md[400][277] = Cd[3][5]; Md[400][278] = Cd[3][6]; Md[400][279] = Cd[3][7]; Md[400][280] = Cd[3][9]; Md[400][281] = Cd[3][10]; Md[400][282] = Cd[3][12]; Md[400][283] = Cd[3][15]; Md[400][284] = Cd[3][16]; Md[400][285] = Cd[3][17]; Md[400][286] = Cd[3][19]; Md[400][287] = Cd[3][20]; Md[400][288] = Cd[3][22]; Md[400][289] = Cd[3][25]; Md[400][290] = Cd[3][26]; Md[400][291] = Cd[3][28]; Md[400][400] = Cd[3][31]; 
   Md[401][32] = Cd[3][14]; Md[401][37] = Cd[3][24]; Md[401][39] = Cd[3][33]; Md[401][49] = Cd[3][5]; Md[401][51] = Cd[3][7]; Md[401][52] = Cd[3][17]; Md[401][106] = Cd[3][1]; Md[401][108] = Cd[3][3]; Md[401][193] = Cd[3][15]; Md[401][262] = Cd[3][30]; Md[401][276] = Cd[3][2]; Md[401][279] = Cd[3][6]; Md[401][281] = Cd[3][9]; Md[401][282] = Cd[3][10]; Md[401][285] = Cd[3][16]; Md[401][287] = Cd[3][19]; Md[401][288] = Cd[3][20]; Md[401][290] = Cd[3][25]; Md[401][291] = Cd[3][26]; Md[401][391] = Cd[3][12]; Md[401][397] = Cd[3][22]; Md[401][400] = Cd[3][28]; Md[401][401] = Cd[3][31]; 
   Md[402][33] = Cd[3][14]; Md[402][38] = Cd[3][24]; Md[402][39] = Cd[3][30]; Md[402][50] = Cd[3][5]; Md[402][51] = Cd[3][6]; Md[402][52] = Cd[3][16]; Md[402][107] = Cd[3][1]; Md[402][108] = Cd[3][2]; Md[402][194] = Cd[3][15]; Md[402][282] = Cd[3][9]; Md[402][288] = Cd[3][19]; Md[402][291] = Cd[3][25]; Md[402][372] = Cd[3][33]; Md[402][386] = Cd[3][3]; Md[402][389] = Cd[3][7]; Md[402][391] = Cd[3][10]; Md[402][392] = Cd[3][12]; Md[402][395] = Cd[3][17]; Md[402][397] = Cd[3][20]; Md[402][398] = Cd[3][22]; Md[402][400] = Cd[3][26]; Md[402][401] = Cd[3][28]; Md[402][402] = Cd[3][31]; 
   Md[403][370] = Cd[3][14]; Md[403][371] = Cd[3][24]; Md[403][372] = Cd[3][30]; Md[403][373] = Cd[3][33]; Md[403][385] = Cd[3][1]; Md[403][386] = Cd[3][2]; Md[403][387] = Cd[3][3]; Md[403][388] = Cd[3][5]; Md[403][389] = Cd[3][6]; Md[403][390] = Cd[3][7]; Md[403][391] = Cd[3][9]; Md[403][392] = Cd[3][10]; Md[403][393] = Cd[3][12]; Md[403][394] = Cd[3][15]; Md[403][395] = Cd[3][16]; Md[403][396] = Cd[3][17]; Md[403][397] = Cd[3][19]; Md[403][398] = Cd[3][20]; Md[403][399] = Cd[3][22]; Md[403][400] = Cd[3][25]; Md[403][401] = Cd[3][26]; Md[403][402] = Cd[3][28]; Md[403][403] = Cd[3][31]; 
   Md[404][69] = Cd[3][14]; Md[404][70] = Cd[3][24]; Md[404][71] = Cd[3][30]; Md[404][72] = Cd[3][33]; Md[404][110] = Cd[3][1]; Md[404][111] = Cd[3][2]; Md[404][112] = Cd[3][3]; Md[404][113] = Cd[3][5]; Md[404][114] = Cd[3][6]; Md[404][115] = Cd[3][7]; Md[404][116] = Cd[3][9]; Md[404][117] = Cd[3][10]; Md[404][118] = Cd[3][12]; Md[404][119] = Cd[3][16]; Md[404][120] = Cd[3][17]; Md[404][121] = Cd[3][19]; Md[404][122] = Cd[3][20]; Md[404][123] = Cd[3][22]; Md[404][124] = Cd[3][26]; Md[404][125] = Cd[3][28]; Md[404][195] = Cd[3][15]; Md[404][292] = Cd[3][25]; Md[404][404] = Cd[3][31]; 
   Md[405][70] = Cd[3][14]; Md[405][73] = Cd[3][24]; Md[405][74] = Cd[3][30]; Md[405][75] = Cd[3][33]; Md[405][113] = Cd[3][1]; Md[405][114] = Cd[3][2]; Md[405][115] = Cd[3][3]; Md[405][119] = Cd[3][6]; Md[405][120] = Cd[3][7]; Md[405][121] = Cd[3][9]; Md[405][122] = Cd[3][10]; Md[405][123] = Cd[3][12]; Md[405][126] = Cd[3][19]; Md[405][127] = Cd[3][20]; Md[405][128] = Cd[3][22]; Md[405][129] = Cd[3][26]; Md[405][130] = Cd[3][28]; Md[405][195] = Cd[3][5]; Md[405][196] = Cd[3][15]; Md[405][197] = Cd[3][16]; Md[405][198] = Cd[3][17]; Md[405][293] = Cd[3][25]; Md[405][405] = Cd[3][31]; 
   Md[406][71] = Cd[3][14]; Md[406][74] = Cd[3][24]; Md[406][76] = Cd[3][30]; Md[406][77] = Cd[3][33]; Md[406][114] = Cd[3][1]; Md[406][116] = Cd[3][2]; Md[406][117] = Cd[3][3]; Md[406][119] = Cd[3][5]; Md[406][121] = Cd[3][6]; Md[406][122] = Cd[3][7]; Md[406][124] = Cd[3][10]; Md[406][125] = Cd[3][12]; Md[406][126] = Cd[3][16]; Md[406][127] = Cd[3][17]; Md[406][129] = Cd[3][20]; Md[406][130] = Cd[3][22]; Md[406][131] = Cd[3][28]; Md[406][197] = Cd[3][15]; Md[406][292] = Cd[3][9]; Md[406][293] = Cd[3][19]; Md[406][294] = Cd[3][25]; Md[406][295] = Cd[3][26]; Md[406][406] = Cd[3][31]; 
   Md[407][72] = Cd[3][14]; Md[407][75] = Cd[3][24]; Md[407][77] = Cd[3][30]; Md[407][78] = Cd[3][33]; Md[407][115] = Cd[3][1]; Md[407][117] = Cd[3][2]; Md[407][118] = Cd[3][3]; Md[407][120] = Cd[3][5]; Md[407][122] = Cd[3][6]; Md[407][123] = Cd[3][7]; Md[407][124] = Cd[3][9]; Md[407][125] = Cd[3][10]; Md[407][127] = Cd[3][16]; Md[407][128] = Cd[3][17]; Md[407][129] = Cd[3][19]; Md[407][130] = Cd[3][20]; Md[407][131] = Cd[3][26]; Md[407][198] = Cd[3][15]; Md[407][295] = Cd[3][25]; Md[407][404] = Cd[3][12]; Md[407][405] = Cd[3][22]; Md[407][406] = Cd[3][28]; Md[407][407] = Cd[3][31]; 
   Md[408][73] = Cd[3][14]; Md[408][79] = Cd[3][30]; Md[408][80] = Cd[3][33]; Md[408][119] = Cd[3][2]; Md[408][120] = Cd[3][3]; Md[408][126] = Cd[3][9]; Md[408][127] = Cd[3][10]; Md[408][128] = Cd[3][12]; Md[408][132] = Cd[3][26]; Md[408][133] = Cd[3][28]; Md[408][167] = Cd[3][24]; Md[408][195] = Cd[3][1]; Md[408][196] = Cd[3][5]; Md[408][197] = Cd[3][6]; Md[408][198] = Cd[3][7]; Md[408][199] = Cd[3][15]; Md[408][200] = Cd[3][16]; Md[408][201] = Cd[3][17]; Md[408][202] = Cd[3][19]; Md[408][203] = Cd[3][20]; Md[408][204] = Cd[3][22]; Md[408][296] = Cd[3][25]; Md[408][408] = Cd[3][31]; 
   Md[409][74] = Cd[3][14]; Md[409][79] = Cd[3][24]; Md[409][81] = Cd[3][30]; Md[409][82] = Cd[3][33]; Md[409][119] = Cd[3][1]; Md[409][121] = Cd[3][2]; Md[409][122] = Cd[3][3]; Md[409][126] = Cd[3][6]; Md[409][127] = Cd[3][7]; Md[409][129] = Cd[3][10]; Md[409][130] = Cd[3][12]; Md[409][132] = Cd[3][20]; Md[409][133] = Cd[3][22]; Md[409][134] = Cd[3][28]; Md[409][197] = Cd[3][5]; Md[409][200] = Cd[3][15]; Md[409][202] = Cd[3][16]; Md[409][203] = Cd[3][17]; Md[409][293] = Cd[3][9]; Md[409][296] = Cd[3][19]; Md[409][297] = Cd[3][25]; Md[409][298] = Cd[3][26]; Md[409][409] = Cd[3][31]; 
   Md[410][75] = Cd[3][14]; Md[410][80] = Cd[3][24]; Md[410][82] = Cd[3][30]; Md[410][83] = Cd[3][33]; Md[410][120] = Cd[3][1]; Md[410][122] = Cd[3][2]; Md[410][123] = Cd[3][3]; Md[410][127] = Cd[3][6]; Md[410][128] = Cd[3][7]; Md[410][129] = Cd[3][9]; Md[410][130] = Cd[3][10]; Md[410][132] = Cd[3][19]; Md[410][133] = Cd[3][20]; Md[410][134] = Cd[3][26]; Md[410][198] = Cd[3][5]; Md[410][201] = Cd[3][15]; Md[410][203] = Cd[3][16]; Md[410][204] = Cd[3][17]; Md[410][298] = Cd[3][25]; Md[410][405] = Cd[3][12]; Md[410][408] = Cd[3][22]; Md[410][409] = Cd[3][28]; Md[410][410] = Cd[3][31]; 
   Md[411][76] = Cd[3][14]; Md[411][81] = Cd[3][24]; Md[411][84] = Cd[3][33]; Md[411][121] = Cd[3][1]; Md[411][124] = Cd[3][3]; Md[411][126] = Cd[3][5]; Md[411][129] = Cd[3][7]; Md[411][131] = Cd[3][12]; Md[411][132] = Cd[3][17]; Md[411][134] = Cd[3][22]; Md[411][202] = Cd[3][15]; Md[411][263] = Cd[3][30]; Md[411][292] = Cd[3][2]; Md[411][293] = Cd[3][6]; Md[411][294] = Cd[3][9]; Md[411][295] = Cd[3][10]; Md[411][296] = Cd[3][16]; Md[411][297] = Cd[3][19]; Md[411][298] = Cd[3][20]; Md[411][299] = Cd[3][25]; Md[411][300] = Cd[3][26]; Md[411][301] = Cd[3][28]; Md[411][411] = Cd[3][31]; 
   Md[412][77] = Cd[3][14]; Md[412][82] = Cd[3][24]; Md[412][84] = Cd[3][30]; Md[412][85] = Cd[3][33]; Md[412][122] = Cd[3][1]; Md[412][124] = Cd[3][2]; Md[412][125] = Cd[3][3]; Md[412][127] = Cd[3][5]; Md[412][129] = Cd[3][6]; Md[412][130] = Cd[3][7]; Md[412][131] = Cd[3][10]; Md[412][132] = Cd[3][16]; Md[412][133] = Cd[3][17]; Md[412][134] = Cd[3][20]; Md[412][203] = Cd[3][15]; Md[412][295] = Cd[3][9]; Md[412][298] = Cd[3][19]; Md[412][300] = Cd[3][25]; Md[412][301] = Cd[3][26]; Md[412][406] = Cd[3][12]; Md[412][409] = Cd[3][22]; Md[412][411] = Cd[3][28]; Md[412][412] = Cd[3][31]; 
   Md[413][78] = Cd[3][14]; Md[413][83] = Cd[3][24]; Md[413][85] = Cd[3][30]; Md[413][123] = Cd[3][1]; Md[413][125] = Cd[3][2]; Md[413][128] = Cd[3][5]; Md[413][130] = Cd[3][6]; Md[413][131] = Cd[3][9]; Md[413][133] = Cd[3][16]; Md[413][134] = Cd[3][19]; Md[413][204] = Cd[3][15]; Md[413][301] = Cd[3][25]; Md[413][374] = Cd[3][33]; Md[413][404] = Cd[3][3]; Md[413][405] = Cd[3][7]; Md[413][406] = Cd[3][10]; Md[413][407] = Cd[3][12]; Md[413][408] = Cd[3][17]; Md[413][409] = Cd[3][20]; Md[413][410] = Cd[3][22]; Md[413][411] = Cd[3][26]; Md[413][412] = Cd[3][28]; Md[413][413] = Cd[3][31]; 
   Md[414][167] = Cd[3][14]; Md[414][168] = Cd[3][24]; Md[414][169] = Cd[3][30]; Md[414][170] = Cd[3][33]; Md[414][196] = Cd[3][1]; Md[414][197] = Cd[3][2]; Md[414][198] = Cd[3][3]; Md[414][199] = Cd[3][5]; Md[414][200] = Cd[3][6]; Md[414][201] = Cd[3][7]; Md[414][202] = Cd[3][9]; Md[414][203] = Cd[3][10]; Md[414][204] = Cd[3][12]; Md[414][205] = Cd[3][15]; Md[414][206] = Cd[3][16]; Md[414][207] = Cd[3][17]; Md[414][208] = Cd[3][19]; Md[414][209] = Cd[3][20]; Md[414][210] = Cd[3][22]; Md[414][211] = Cd[3][26]; Md[414][212] = Cd[3][28]; Md[414][302] = Cd[3][25]; Md[414][414] = Cd[3][31]; 
   Md[415][40] = Cd[3][30]; Md[415][41] = Cd[3][33]; Md[415][53] = Cd[3][28]; Md[415][79] = Cd[3][14]; Md[415][126] = Cd[3][2]; Md[415][127] = Cd[3][3]; Md[415][132] = Cd[3][10]; Md[415][133] = Cd[3][12]; Md[415][169] = Cd[3][24]; Md[415][197] = Cd[3][1]; Md[415][200] = Cd[3][5]; Md[415][202] = Cd[3][6]; Md[415][203] = Cd[3][7]; Md[415][206] = Cd[3][15]; Md[415][208] = Cd[3][16]; Md[415][209] = Cd[3][17]; Md[415][211] = Cd[3][20]; Md[415][212] = Cd[3][22]; Md[415][296] = Cd[3][9]; Md[415][302] = Cd[3][19]; Md[415][303] = Cd[3][25]; Md[415][304] = Cd[3][26]; Md[415][415] = Cd[3][31]; 
   Md[416][41] = Cd[3][30]; Md[416][42] = Cd[3][33]; Md[416][53] = Cd[3][26]; Md[416][80] = Cd[3][14]; Md[416][127] = Cd[3][2]; Md[416][128] = Cd[3][3]; Md[416][132] = Cd[3][9]; Md[416][133] = Cd[3][10]; Md[416][170] = Cd[3][24]; Md[416][198] = Cd[3][1]; Md[416][201] = Cd[3][5]; Md[416][203] = Cd[3][6]; Md[416][204] = Cd[3][7]; Md[416][207] = Cd[3][15]; Md[416][209] = Cd[3][16]; Md[416][210] = Cd[3][17]; Md[416][211] = Cd[3][19]; Md[416][212] = Cd[3][20]; Md[416][304] = Cd[3][25]; Md[416][408] = Cd[3][12]; Md[416][414] = Cd[3][22]; Md[416][415] = Cd[3][28]; Md[416][416] = Cd[3][31]; 
   Md[417][40] = Cd[3][24]; Md[417][43] = Cd[3][33]; Md[417][53] = Cd[3][22]; Md[417][81] = Cd[3][14]; Md[417][126] = Cd[3][1]; Md[417][129] = Cd[3][3]; Md[417][132] = Cd[3][7]; Md[417][134] = Cd[3][12]; Md[417][202] = Cd[3][5]; Md[417][208] = Cd[3][15]; Md[417][211] = Cd[3][17]; Md[417][264] = Cd[3][30]; Md[417][293] = Cd[3][2]; Md[417][296] = Cd[3][6]; Md[417][297] = Cd[3][9]; Md[417][298] = Cd[3][10]; Md[417][302] = Cd[3][16]; Md[417][303] = Cd[3][19]; Md[417][304] = Cd[3][20]; Md[417][305] = Cd[3][25]; Md[417][306] = Cd[3][26]; Md[417][307] = Cd[3][28]; Md[417][417] = Cd[3][31]; 
   Md[418][41] = Cd[3][24]; Md[418][43] = Cd[3][30]; Md[418][44] = Cd[3][33]; Md[418][53] = Cd[3][20]; Md[418][82] = Cd[3][14]; Md[418][127] = Cd[3][1]; Md[418][129] = Cd[3][2]; Md[418][130] = Cd[3][3]; Md[418][132] = Cd[3][6]; Md[418][133] = Cd[3][7]; Md[418][134] = Cd[3][10]; Md[418][203] = Cd[3][5]; Md[418][209] = Cd[3][15]; Md[418][211] = Cd[3][16]; Md[418][212] = Cd[3][17]; Md[418][298] = Cd[3][9]; Md[418][304] = Cd[3][19]; Md[418][306] = Cd[3][25]; Md[418][307] = Cd[3][26]; Md[418][409] = Cd[3][12]; Md[418][415] = Cd[3][22]; Md[418][417] = Cd[3][28]; Md[418][418] = Cd[3][31]; 
   Md[419][42] = Cd[3][24]; Md[419][44] = Cd[3][30]; Md[419][53] = Cd[3][19]; Md[419][83] = Cd[3][14]; Md[419][128] = Cd[3][1]; Md[419][130] = Cd[3][2]; Md[419][133] = Cd[3][6]; Md[419][134] = Cd[3][9]; Md[419][204] = Cd[3][5]; Md[419][210] = Cd[3][15]; Md[419][212] = Cd[3][16]; Md[419][307] = Cd[3][25]; Md[419][375] = Cd[3][33]; Md[419][405] = Cd[3][3]; Md[419][408] = Cd[3][7]; Md[419][409] = Cd[3][10]; Md[419][410] = Cd[3][12]; Md[419][414] = Cd[3][17]; Md[419][415] = Cd[3][20]; Md[419][416] = Cd[3][22]; Md[419][417] = Cd[3][26]; Md[419][418] = Cd[3][28]; Md[419][419] = Cd[3][31]; 
   Md[420][263] = Cd[3][14]; Md[420][264] = Cd[3][24]; Md[420][265] = Cd[3][30]; Md[420][266] = Cd[3][33]; Md[420][293] = Cd[3][1]; Md[420][294] = Cd[3][2]; Md[420][295] = Cd[3][3]; Md[420][296] = Cd[3][5]; Md[420][297] = Cd[3][6]; Md[420][298] = Cd[3][7]; Md[420][299] = Cd[3][9]; Md[420][300] = Cd[3][10]; Md[420][301] = Cd[3][12]; Md[420][302] = Cd[3][15]; Md[420][303] = Cd[3][16]; Md[420][304] = Cd[3][17]; Md[420][305] = Cd[3][19]; Md[420][306] = Cd[3][20]; Md[420][307] = Cd[3][22]; Md[420][308] = Cd[3][25]; Md[420][309] = Cd[3][26]; Md[420][310] = Cd[3][28]; Md[420][420] = Cd[3][31]; 
   Md[421][43] = Cd[3][24]; Md[421][45] = Cd[3][33]; Md[421][53] = Cd[3][17]; Md[421][84] = Cd[3][14]; Md[421][129] = Cd[3][1]; Md[421][131] = Cd[3][3]; Md[421][132] = Cd[3][5]; Md[421][134] = Cd[3][7]; Md[421][211] = Cd[3][15]; Md[421][266] = Cd[3][30]; Md[421][295] = Cd[3][2]; Md[421][298] = Cd[3][6]; Md[421][300] = Cd[3][9]; Md[421][301] = Cd[3][10]; Md[421][304] = Cd[3][16]; Md[421][306] = Cd[3][19]; Md[421][307] = Cd[3][20]; Md[421][309] = Cd[3][25]; Md[421][310] = Cd[3][26]; Md[421][411] = Cd[3][12]; Md[421][417] = Cd[3][22]; Md[421][420] = Cd[3][28]; Md[421][421] = Cd[3][31]; 
   Md[422][44] = Cd[3][24]; Md[422][45] = Cd[3][30]; Md[422][53] = Cd[3][16]; Md[422][85] = Cd[3][14]; Md[422][130] = Cd[3][1]; Md[422][131] = Cd[3][2]; Md[422][133] = Cd[3][5]; Md[422][134] = Cd[3][6]; Md[422][212] = Cd[3][15]; Md[422][301] = Cd[3][9]; Md[422][307] = Cd[3][19]; Md[422][310] = Cd[3][25]; Md[422][376] = Cd[3][33]; Md[422][406] = Cd[3][3]; Md[422][409] = Cd[3][7]; Md[422][411] = Cd[3][10]; Md[422][412] = Cd[3][12]; Md[422][415] = Cd[3][17]; Md[422][417] = Cd[3][20]; Md[422][418] = Cd[3][22]; Md[422][420] = Cd[3][26]; Md[422][421] = Cd[3][28]; Md[422][422] = Cd[3][31]; 
   Md[423][374] = Cd[3][14]; Md[423][375] = Cd[3][24]; Md[423][376] = Cd[3][30]; Md[423][377] = Cd[3][33]; Md[423][405] = Cd[3][1]; Md[423][406] = Cd[3][2]; Md[423][407] = Cd[3][3]; Md[423][408] = Cd[3][5]; Md[423][409] = Cd[3][6]; Md[423][410] = Cd[3][7]; Md[423][411] = Cd[3][9]; Md[423][412] = Cd[3][10]; Md[423][413] = Cd[3][12]; Md[423][414] = Cd[3][15]; Md[423][415] = Cd[3][16]; Md[423][416] = Cd[3][17]; Md[423][417] = Cd[3][19]; Md[423][418] = Cd[3][20]; Md[423][419] = Cd[3][22]; Md[423][420] = Cd[3][25]; Md[423][421] = Cd[3][26]; Md[423][422] = Cd[3][28]; Md[423][423] = Cd[3][31]; 
   Md[424][168] = Cd[3][14]; Md[424][171] = Cd[3][24]; Md[424][172] = Cd[3][30]; Md[424][173] = Cd[3][33]; Md[424][199] = Cd[3][1]; Md[424][200] = Cd[3][2]; Md[424][201] = Cd[3][3]; Md[424][205] = Cd[3][5]; Md[424][206] = Cd[3][6]; Md[424][207] = Cd[3][7]; Md[424][208] = Cd[3][9]; Md[424][209] = Cd[3][10]; Md[424][210] = Cd[3][12]; Md[424][213] = Cd[3][15]; Md[424][214] = Cd[3][16]; Md[424][215] = Cd[3][17]; Md[424][216] = Cd[3][19]; Md[424][217] = Cd[3][20]; Md[424][218] = Cd[3][22]; Md[424][219] = Cd[3][26]; Md[424][220] = Cd[3][28]; Md[424][311] = Cd[3][25]; Md[424][424] = Cd[3][31]; 
   Md[425][169] = Cd[3][14]; Md[425][172] = Cd[3][24]; Md[425][174] = Cd[3][30]; Md[425][175] = Cd[3][33]; Md[425][200] = Cd[3][1]; Md[425][202] = Cd[3][2]; Md[425][203] = Cd[3][3]; Md[425][206] = Cd[3][5]; Md[425][208] = Cd[3][6]; Md[425][209] = Cd[3][7]; Md[425][211] = Cd[3][10]; Md[425][212] = Cd[3][12]; Md[425][214] = Cd[3][15]; Md[425][216] = Cd[3][16]; Md[425][217] = Cd[3][17]; Md[425][219] = Cd[3][20]; Md[425][220] = Cd[3][22]; Md[425][221] = Cd[3][28]; Md[425][302] = Cd[3][9]; Md[425][311] = Cd[3][19]; Md[425][312] = Cd[3][25]; Md[425][313] = Cd[3][26]; Md[425][425] = Cd[3][31]; 
   Md[426][170] = Cd[3][14]; Md[426][173] = Cd[3][24]; Md[426][175] = Cd[3][30]; Md[426][176] = Cd[3][33]; Md[426][201] = Cd[3][1]; Md[426][203] = Cd[3][2]; Md[426][204] = Cd[3][3]; Md[426][207] = Cd[3][5]; Md[426][209] = Cd[3][6]; Md[426][210] = Cd[3][7]; Md[426][211] = Cd[3][9]; Md[426][212] = Cd[3][10]; Md[426][215] = Cd[3][15]; Md[426][217] = Cd[3][16]; Md[426][218] = Cd[3][17]; Md[426][219] = Cd[3][19]; Md[426][220] = Cd[3][20]; Md[426][221] = Cd[3][26]; Md[426][313] = Cd[3][25]; Md[426][414] = Cd[3][12]; Md[426][424] = Cd[3][22]; Md[426][425] = Cd[3][28]; Md[426][426] = Cd[3][31]; 
   Md[427][40] = Cd[3][14]; Md[427][46] = Cd[3][33]; Md[427][53] = Cd[3][12]; Md[427][132] = Cd[3][3]; Md[427][174] = Cd[3][24]; Md[427][202] = Cd[3][1]; Md[427][208] = Cd[3][5]; Md[427][211] = Cd[3][7]; Md[427][216] = Cd[3][15]; Md[427][219] = Cd[3][17]; Md[427][221] = Cd[3][22]; Md[427][267] = Cd[3][30]; Md[427][296] = Cd[3][2]; Md[427][302] = Cd[3][6]; Md[427][303] = Cd[3][9]; Md[427][304] = Cd[3][10]; Md[427][311] = Cd[3][16]; Md[427][312] = Cd[3][19]; Md[427][313] = Cd[3][20]; Md[427][314] = Cd[3][25]; Md[427][315] = Cd[3][26]; Md[427][316] = Cd[3][28]; Md[427][427] = Cd[3][31]; 
   Md[428][41] = Cd[3][14]; Md[428][46] = Cd[3][30]; Md[428][47] = Cd[3][33]; Md[428][53] = Cd[3][10]; Md[428][132] = Cd[3][2]; Md[428][133] = Cd[3][3]; Md[428][175] = Cd[3][24]; Md[428][203] = Cd[3][1]; Md[428][209] = Cd[3][5]; Md[428][211] = Cd[3][6]; Md[428][212] = Cd[3][7]; Md[428][217] = Cd[3][15]; Md[428][219] = Cd[3][16]; Md[428][220] = Cd[3][17]; Md[428][221] = Cd[3][20]; Md[428][304] = Cd[3][9]; Md[428][313] = Cd[3][19]; Md[428][315] = Cd[3][25]; Md[428][316] = Cd[3][26]; Md[428][415] = Cd[3][12]; Md[428][425] = Cd[3][22]; Md[428][427] = Cd[3][28]; Md[428][428] = Cd[3][31]; 
   Md[429][42] = Cd[3][14]; Md[429][47] = Cd[3][30]; Md[429][53] = Cd[3][9]; Md[429][133] = Cd[3][2]; Md[429][176] = Cd[3][24]; Md[429][204] = Cd[3][1]; Md[429][210] = Cd[3][5]; Md[429][212] = Cd[3][6]; Md[429][218] = Cd[3][15]; Md[429][220] = Cd[3][16]; Md[429][221] = Cd[3][19]; Md[429][316] = Cd[3][25]; Md[429][378] = Cd[3][33]; Md[429][408] = Cd[3][3]; Md[429][414] = Cd[3][7]; Md[429][415] = Cd[3][10]; Md[429][416] = Cd[3][12]; Md[429][424] = Cd[3][17]; Md[429][425] = Cd[3][20]; Md[429][426] = Cd[3][22]; Md[429][427] = Cd[3][26]; Md[429][428] = Cd[3][28]; Md[429][429] = Cd[3][31]; 
   Md[430][264] = Cd[3][14]; Md[430][267] = Cd[3][24]; Md[430][268] = Cd[3][30]; Md[430][269] = Cd[3][33]; Md[430][296] = Cd[3][1]; Md[430][297] = Cd[3][2]; Md[430][298] = Cd[3][3]; Md[430][302] = Cd[3][5]; Md[430][303] = Cd[3][6]; Md[430][304] = Cd[3][7]; Md[430][305] = Cd[3][9]; Md[430][306] = Cd[3][10]; Md[430][307] = Cd[3][12]; Md[430][311] = Cd[3][15]; Md[430][312] = Cd[3][16]; Md[430][313] = Cd[3][17]; Md[430][314] = Cd[3][19]; Md[430][315] = Cd[3][20]; Md[430][316] = Cd[3][22]; Md[430][317] = Cd[3][25]; Md[430][318] = Cd[3][26]; Md[430][319] = Cd[3][28]; Md[430][430] = Cd[3][31]; 
   Md[431][43] = Cd[3][14]; Md[431][46] = Cd[3][24]; Md[431][48] = Cd[3][33]; Md[431][53] = Cd[3][7]; Md[431][132] = Cd[3][1]; Md[431][134] = Cd[3][3]; Md[431][211] = Cd[3][5]; Md[431][219] = Cd[3][15]; Md[431][221] = Cd[3][17]; Md[431][269] = Cd[3][30]; Md[431][298] = Cd[3][2]; Md[431][304] = Cd[3][6]; Md[431][306] = Cd[3][9]; Md[431][307] = Cd[3][10]; Md[431][313] = Cd[3][16]; Md[431][315] = Cd[3][19]; Md[431][316] = Cd[3][20]; Md[431][318] = Cd[3][25]; Md[431][319] = Cd[3][26]; Md[431][417] = Cd[3][12]; Md[431][427] = Cd[3][22]; Md[431][430] = Cd[3][28]; Md[431][431] = Cd[3][31]; 
   Md[432][44] = Cd[3][14]; Md[432][47] = Cd[3][24]; Md[432][48] = Cd[3][30]; Md[432][53] = Cd[3][6]; Md[432][133] = Cd[3][1]; Md[432][134] = Cd[3][2]; Md[432][212] = Cd[3][5]; Md[432][220] = Cd[3][15]; Md[432][221] = Cd[3][16]; Md[432][307] = Cd[3][9]; Md[432][316] = Cd[3][19]; Md[432][319] = Cd[3][25]; Md[432][379] = Cd[3][33]; Md[432][409] = Cd[3][3]; Md[432][415] = Cd[3][7]; Md[432][417] = Cd[3][10]; Md[432][418] = Cd[3][12]; Md[432][425] = Cd[3][17]; Md[432][427] = Cd[3][20]; Md[432][428] = Cd[3][22]; Md[432][430] = Cd[3][26]; Md[432][431] = Cd[3][28]; Md[432][432] = Cd[3][31]; 
   Md[433][375] = Cd[3][14]; Md[433][378] = Cd[3][24]; Md[433][379] = Cd[3][30]; Md[433][380] = Cd[3][33]; Md[433][408] = Cd[3][1]; Md[433][409] = Cd[3][2]; Md[433][410] = Cd[3][3]; Md[433][414] = Cd[3][5]; Md[433][415] = Cd[3][6]; Md[433][416] = Cd[3][7]; Md[433][417] = Cd[3][9]; Md[433][418] = Cd[3][10]; Md[433][419] = Cd[3][12]; Md[433][424] = Cd[3][15]; Md[433][425] = Cd[3][16]; Md[433][426] = Cd[3][17]; Md[433][427] = Cd[3][19]; Md[433][428] = Cd[3][20]; Md[433][429] = Cd[3][22]; Md[433][430] = Cd[3][25]; Md[433][431] = Cd[3][26]; Md[433][432] = Cd[3][28]; Md[433][433] = Cd[3][31]; 
   Md[434][265] = Cd[3][14]; Md[434][268] = Cd[3][24]; Md[434][270] = Cd[3][30]; Md[434][271] = Cd[3][33]; Md[434][297] = Cd[3][1]; Md[434][299] = Cd[3][2]; Md[434][300] = Cd[3][3]; Md[434][303] = Cd[3][5]; Md[434][305] = Cd[3][6]; Md[434][306] = Cd[3][7]; Md[434][308] = Cd[3][9]; Md[434][309] = Cd[3][10]; Md[434][310] = Cd[3][12]; Md[434][312] = Cd[3][15]; Md[434][314] = Cd[3][16]; Md[434][315] = Cd[3][17]; Md[434][317] = Cd[3][19]; Md[434][318] = Cd[3][20]; Md[434][319] = Cd[3][22]; Md[434][320] = Cd[3][25]; Md[434][321] = Cd[3][26]; Md[434][322] = Cd[3][28]; Md[434][434] = Cd[3][31]; 
   Md[435][266] = Cd[3][14]; Md[435][269] = Cd[3][24]; Md[435][271] = Cd[3][30]; Md[435][272] = Cd[3][33]; Md[435][298] = Cd[3][1]; Md[435][300] = Cd[3][2]; Md[435][301] = Cd[3][3]; Md[435][304] = Cd[3][5]; Md[435][306] = Cd[3][6]; Md[435][307] = Cd[3][7]; Md[435][309] = Cd[3][9]; Md[435][310] = Cd[3][10]; Md[435][313] = Cd[3][15]; Md[435][315] = Cd[3][16]; Md[435][316] = Cd[3][17]; Md[435][318] = Cd[3][19]; Md[435][319] = Cd[3][20]; Md[435][321] = Cd[3][25]; Md[435][322] = Cd[3][26]; Md[435][420] = Cd[3][12]; Md[435][430] = Cd[3][22]; Md[435][434] = Cd[3][28]; Md[435][435] = Cd[3][31]; 
   Md[436][45] = Cd[3][14]; Md[436][48] = Cd[3][24]; Md[436][53] = Cd[3][5]; Md[436][134] = Cd[3][1]; Md[436][221] = Cd[3][15]; Md[436][272] = Cd[3][30]; Md[436][301] = Cd[3][2]; Md[436][307] = Cd[3][6]; Md[436][310] = Cd[3][9]; Md[436][316] = Cd[3][16]; Md[436][319] = Cd[3][19]; Md[436][322] = Cd[3][25]; Md[436][381] = Cd[3][33]; Md[436][411] = Cd[3][3]; Md[436][417] = Cd[3][7]; Md[436][420] = Cd[3][10]; Md[436][421] = Cd[3][12]; Md[436][427] = Cd[3][17]; Md[436][430] = Cd[3][20]; Md[436][431] = Cd[3][22]; Md[436][434] = Cd[3][26]; Md[436][435] = Cd[3][28]; Md[436][436] = Cd[3][31]; 
   Md[437][376] = Cd[3][14]; Md[437][379] = Cd[3][24]; Md[437][381] = Cd[3][30]; Md[437][382] = Cd[3][33]; Md[437][409] = Cd[3][1]; Md[437][411] = Cd[3][2]; Md[437][412] = Cd[3][3]; Md[437][415] = Cd[3][5]; Md[437][417] = Cd[3][6]; Md[437][418] = Cd[3][7]; Md[437][420] = Cd[3][9]; Md[437][421] = Cd[3][10]; Md[437][422] = Cd[3][12]; Md[437][425] = Cd[3][15]; Md[437][427] = Cd[3][16]; Md[437][428] = Cd[3][17]; Md[437][430] = Cd[3][19]; Md[437][431] = Cd[3][20]; Md[437][432] = Cd[3][22]; Md[437][434] = Cd[3][25]; Md[437][435] = Cd[3][26]; Md[437][436] = Cd[3][28]; Md[437][437] = Cd[3][31]; 
   Md[438][377] = Cd[3][14]; Md[438][380] = Cd[3][24]; Md[438][382] = Cd[3][30]; Md[438][383] = Cd[3][33]; Md[438][410] = Cd[3][1]; Md[438][412] = Cd[3][2]; Md[438][413] = Cd[3][3]; Md[438][416] = Cd[3][5]; Md[438][418] = Cd[3][6]; Md[438][419] = Cd[3][7]; Md[438][421] = Cd[3][9]; Md[438][422] = Cd[3][10]; Md[438][423] = Cd[3][12]; Md[438][426] = Cd[3][15]; Md[438][428] = Cd[3][16]; Md[438][429] = Cd[3][17]; Md[438][431] = Cd[3][19]; Md[438][432] = Cd[3][20]; Md[438][433] = Cd[3][22]; Md[438][435] = Cd[3][25]; Md[438][436] = Cd[3][26]; Md[438][437] = Cd[3][28]; Md[438][438] = Cd[3][31]; 
   Md[439][86] = Cd[3][14]; Md[439][87] = Cd[3][24]; Md[439][88] = Cd[3][30]; Md[439][89] = Cd[3][33]; Md[439][136] = Cd[3][1]; Md[439][137] = Cd[3][2]; Md[439][138] = Cd[3][3]; Md[439][139] = Cd[3][5]; Md[439][140] = Cd[3][6]; Md[439][141] = Cd[3][7]; Md[439][142] = Cd[3][9]; Md[439][143] = Cd[3][10]; Md[439][144] = Cd[3][12]; Md[439][145] = Cd[3][16]; Md[439][146] = Cd[3][17]; Md[439][147] = Cd[3][19]; Md[439][148] = Cd[3][20]; Md[439][149] = Cd[3][22]; Md[439][150] = Cd[3][26]; Md[439][151] = Cd[3][28]; Md[439][222] = Cd[3][15]; Md[439][323] = Cd[3][25]; Md[439][439] = Cd[3][31]; 
   Md[440][87] = Cd[3][14]; Md[440][90] = Cd[3][24]; Md[440][91] = Cd[3][30]; Md[440][92] = Cd[3][33]; Md[440][139] = Cd[3][1]; Md[440][140] = Cd[3][2]; Md[440][141] = Cd[3][3]; Md[440][145] = Cd[3][6]; Md[440][146] = Cd[3][7]; Md[440][147] = Cd[3][9]; Md[440][148] = Cd[3][10]; Md[440][149] = Cd[3][12]; Md[440][152] = Cd[3][19]; Md[440][153] = Cd[3][20]; Md[440][154] = Cd[3][22]; Md[440][155] = Cd[3][26]; Md[440][156] = Cd[3][28]; Md[440][222] = Cd[3][5]; Md[440][223] = Cd[3][15]; Md[440][224] = Cd[3][16]; Md[440][225] = Cd[3][17]; Md[440][324] = Cd[3][25]; Md[440][440] = Cd[3][31]; 
   Md[441][88] = Cd[3][14]; Md[441][91] = Cd[3][24]; Md[441][93] = Cd[3][30]; Md[441][94] = Cd[3][33]; Md[441][140] = Cd[3][1]; Md[441][142] = Cd[3][2]; Md[441][143] = Cd[3][3]; Md[441][145] = Cd[3][5]; Md[441][147] = Cd[3][6]; Md[441][148] = Cd[3][7]; Md[441][150] = Cd[3][10]; Md[441][151] = Cd[3][12]; Md[441][152] = Cd[3][16]; Md[441][153] = Cd[3][17]; Md[441][155] = Cd[3][20]; Md[441][156] = Cd[3][22]; Md[441][157] = Cd[3][28]; Md[441][224] = Cd[3][15]; Md[441][323] = Cd[3][9]; Md[441][324] = Cd[3][19]; Md[441][325] = Cd[3][25]; Md[441][326] = Cd[3][26]; Md[441][441] = Cd[3][31]; 
   Md[442][89] = Cd[3][14]; Md[442][92] = Cd[3][24]; Md[442][94] = Cd[3][30]; Md[442][95] = Cd[3][33]; Md[442][141] = Cd[3][1]; Md[442][143] = Cd[3][2]; Md[442][144] = Cd[3][3]; Md[442][146] = Cd[3][5]; Md[442][148] = Cd[3][6]; Md[442][149] = Cd[3][7]; Md[442][150] = Cd[3][9]; Md[442][151] = Cd[3][10]; Md[442][153] = Cd[3][16]; Md[442][154] = Cd[3][17]; Md[442][155] = Cd[3][19]; Md[442][156] = Cd[3][20]; Md[442][157] = Cd[3][26]; Md[442][225] = Cd[3][15]; Md[442][326] = Cd[3][25]; Md[442][439] = Cd[3][12]; Md[442][440] = Cd[3][22]; Md[442][441] = Cd[3][28]; Md[442][442] = Cd[3][31]; 
   Md[443][90] = Cd[3][14]; Md[443][96] = Cd[3][30]; Md[443][97] = Cd[3][33]; Md[443][145] = Cd[3][2]; Md[443][146] = Cd[3][3]; Md[443][152] = Cd[3][9]; Md[443][153] = Cd[3][10]; Md[443][154] = Cd[3][12]; Md[443][158] = Cd[3][26]; Md[443][159] = Cd[3][28]; Md[443][177] = Cd[3][24]; Md[443][222] = Cd[3][1]; Md[443][223] = Cd[3][5]; Md[443][224] = Cd[3][6]; Md[443][225] = Cd[3][7]; Md[443][226] = Cd[3][15]; Md[443][227] = Cd[3][16]; Md[443][228] = Cd[3][17]; Md[443][229] = Cd[3][19]; Md[443][230] = Cd[3][20]; Md[443][231] = Cd[3][22]; Md[443][327] = Cd[3][25]; Md[443][443] = Cd[3][31]; 
   Md[444][91] = Cd[3][14]; Md[444][96] = Cd[3][24]; Md[444][98] = Cd[3][30]; Md[444][99] = Cd[3][33]; Md[444][145] = Cd[3][1]; Md[444][147] = Cd[3][2]; Md[444][148] = Cd[3][3]; Md[444][152] = Cd[3][6]; Md[444][153] = Cd[3][7]; Md[444][155] = Cd[3][10]; Md[444][156] = Cd[3][12]; Md[444][158] = Cd[3][20]; Md[444][159] = Cd[3][22]; Md[444][160] = Cd[3][28]; Md[444][224] = Cd[3][5]; Md[444][227] = Cd[3][15]; Md[444][229] = Cd[3][16]; Md[444][230] = Cd[3][17]; Md[444][324] = Cd[3][9]; Md[444][327] = Cd[3][19]; Md[444][328] = Cd[3][25]; Md[444][329] = Cd[3][26]; Md[444][444] = Cd[3][31]; 
   Md[445][92] = Cd[3][14]; Md[445][97] = Cd[3][24]; Md[445][99] = Cd[3][30]; Md[445][100] = Cd[3][33]; Md[445][146] = Cd[3][1]; Md[445][148] = Cd[3][2]; Md[445][149] = Cd[3][3]; Md[445][153] = Cd[3][6]; Md[445][154] = Cd[3][7]; Md[445][155] = Cd[3][9]; Md[445][156] = Cd[3][10]; Md[445][158] = Cd[3][19]; Md[445][159] = Cd[3][20]; Md[445][160] = Cd[3][26]; Md[445][225] = Cd[3][5]; Md[445][228] = Cd[3][15]; Md[445][230] = Cd[3][16]; Md[445][231] = Cd[3][17]; Md[445][329] = Cd[3][25]; Md[445][440] = Cd[3][12]; Md[445][443] = Cd[3][22]; Md[445][444] = Cd[3][28]; Md[445][445] = Cd[3][31]; 
   Md[446][93] = Cd[3][14]; Md[446][98] = Cd[3][24]; Md[446][101] = Cd[3][33]; Md[446][147] = Cd[3][1]; Md[446][150] = Cd[3][3]; Md[446][152] = Cd[3][5]; Md[446][155] = Cd[3][7]; Md[446][157] = Cd[3][12]; Md[446][158] = Cd[3][17]; Md[446][160] = Cd[3][22]; Md[446][229] = Cd[3][15]; Md[446][273] = Cd[3][30]; Md[446][323] = Cd[3][2]; Md[446][324] = Cd[3][6]; Md[446][325] = Cd[3][9]; Md[446][326] = Cd[3][10]; Md[446][327] = Cd[3][16]; Md[446][328] = Cd[3][19]; Md[446][329] = Cd[3][20]; Md[446][330] = Cd[3][25]; Md[446][331] = Cd[3][26]; Md[446][332] = Cd[3][28]; Md[446][446] = Cd[3][31]; 
   Md[447][94] = Cd[3][14]; Md[447][99] = Cd[3][24]; Md[447][101] = Cd[3][30]; Md[447][102] = Cd[3][33]; Md[447][148] = Cd[3][1]; Md[447][150] = Cd[3][2]; Md[447][151] = Cd[3][3]; Md[447][153] = Cd[3][5]; Md[447][155] = Cd[3][6]; Md[447][156] = Cd[3][7]; Md[447][157] = Cd[3][10]; Md[447][158] = Cd[3][16]; Md[447][159] = Cd[3][17]; Md[447][160] = Cd[3][20]; Md[447][230] = Cd[3][15]; Md[447][326] = Cd[3][9]; Md[447][329] = Cd[3][19]; Md[447][331] = Cd[3][25]; Md[447][332] = Cd[3][26]; Md[447][441] = Cd[3][12]; Md[447][444] = Cd[3][22]; Md[447][446] = Cd[3][28]; Md[447][447] = Cd[3][31]; 
   Md[448][95] = Cd[3][14]; Md[448][100] = Cd[3][24]; Md[448][102] = Cd[3][30]; Md[448][149] = Cd[3][1]; Md[448][151] = Cd[3][2]; Md[448][154] = Cd[3][5]; Md[448][156] = Cd[3][6]; Md[448][157] = Cd[3][9]; Md[448][159] = Cd[3][16]; Md[448][160] = Cd[3][19]; Md[448][231] = Cd[3][15]; Md[448][332] = Cd[3][25]; Md[448][384] = Cd[3][33]; Md[448][439] = Cd[3][3]; Md[448][440] = Cd[3][7]; Md[448][441] = Cd[3][10]; Md[448][442] = Cd[3][12]; Md[448][443] = Cd[3][17]; Md[448][444] = Cd[3][20]; Md[448][445] = Cd[3][22]; Md[448][446] = Cd[3][26]; Md[448][447] = Cd[3][28]; Md[448][448] = Cd[3][31]; 
   Md[449][177] = Cd[3][14]; Md[449][178] = Cd[3][24]; Md[449][179] = Cd[3][30]; Md[449][180] = Cd[3][33]; Md[449][223] = Cd[3][1]; Md[449][224] = Cd[3][2]; Md[449][225] = Cd[3][3]; Md[449][226] = Cd[3][5]; Md[449][227] = Cd[3][6]; Md[449][228] = Cd[3][7]; Md[449][229] = Cd[3][9]; Md[449][230] = Cd[3][10]; Md[449][231] = Cd[3][12]; Md[449][232] = Cd[3][15]; Md[449][233] = Cd[3][16]; Md[449][234] = Cd[3][17]; Md[449][235] = Cd[3][19]; Md[449][236] = Cd[3][20]; Md[449][237] = Cd[3][22]; Md[449][238] = Cd[3][26]; Md[449][239] = Cd[3][28]; Md[449][333] = Cd[3][25]; Md[449][449] = Cd[3][31]; 
   Md[450][96] = Cd[3][14]; Md[450][103] = Cd[3][30]; Md[450][104] = Cd[3][33]; Md[450][152] = Cd[3][2]; Md[450][153] = Cd[3][3]; Md[450][158] = Cd[3][10]; Md[450][159] = Cd[3][12]; Md[450][161] = Cd[3][28]; Md[450][179] = Cd[3][24]; Md[450][224] = Cd[3][1]; Md[450][227] = Cd[3][5]; Md[450][229] = Cd[3][6]; Md[450][230] = Cd[3][7]; Md[450][233] = Cd[3][15]; Md[450][235] = Cd[3][16]; Md[450][236] = Cd[3][17]; Md[450][238] = Cd[3][20]; Md[450][239] = Cd[3][22]; Md[450][327] = Cd[3][9]; Md[450][333] = Cd[3][19]; Md[450][334] = Cd[3][25]; Md[450][335] = Cd[3][26]; Md[450][450] = Cd[3][31]; 
   Md[451][97] = Cd[3][14]; Md[451][104] = Cd[3][30]; Md[451][105] = Cd[3][33]; Md[451][153] = Cd[3][2]; Md[451][154] = Cd[3][3]; Md[451][158] = Cd[3][9]; Md[451][159] = Cd[3][10]; Md[451][161] = Cd[3][26]; Md[451][180] = Cd[3][24]; Md[451][225] = Cd[3][1]; Md[451][228] = Cd[3][5]; Md[451][230] = Cd[3][6]; Md[451][231] = Cd[3][7]; Md[451][234] = Cd[3][15]; Md[451][236] = Cd[3][16]; Md[451][237] = Cd[3][17]; Md[451][238] = Cd[3][19]; Md[451][239] = Cd[3][20]; Md[451][335] = Cd[3][25]; Md[451][443] = Cd[3][12]; Md[451][449] = Cd[3][22]; Md[451][450] = Cd[3][28]; Md[451][451] = Cd[3][31]; 
   Md[452][98] = Cd[3][14]; Md[452][103] = Cd[3][24]; Md[452][106] = Cd[3][33]; Md[452][152] = Cd[3][1]; Md[452][155] = Cd[3][3]; Md[452][158] = Cd[3][7]; Md[452][160] = Cd[3][12]; Md[452][161] = Cd[3][22]; Md[452][229] = Cd[3][5]; Md[452][235] = Cd[3][15]; Md[452][238] = Cd[3][17]; Md[452][274] = Cd[3][30]; Md[452][324] = Cd[3][2]; Md[452][327] = Cd[3][6]; Md[452][328] = Cd[3][9]; Md[452][329] = Cd[3][10]; Md[452][333] = Cd[3][16]; Md[452][334] = Cd[3][19]; Md[452][335] = Cd[3][20]; Md[452][336] = Cd[3][25]; Md[452][337] = Cd[3][26]; Md[452][338] = Cd[3][28]; Md[452][452] = Cd[3][31]; 
   Md[453][99] = Cd[3][14]; Md[453][104] = Cd[3][24]; Md[453][106] = Cd[3][30]; Md[453][107] = Cd[3][33]; Md[453][153] = Cd[3][1]; Md[453][155] = Cd[3][2]; Md[453][156] = Cd[3][3]; Md[453][158] = Cd[3][6]; Md[453][159] = Cd[3][7]; Md[453][160] = Cd[3][10]; Md[453][161] = Cd[3][20]; Md[453][230] = Cd[3][5]; Md[453][236] = Cd[3][15]; Md[453][238] = Cd[3][16]; Md[453][239] = Cd[3][17]; Md[453][329] = Cd[3][9]; Md[453][335] = Cd[3][19]; Md[453][337] = Cd[3][25]; Md[453][338] = Cd[3][26]; Md[453][444] = Cd[3][12]; Md[453][450] = Cd[3][22]; Md[453][452] = Cd[3][28]; Md[453][453] = Cd[3][31]; 
   Md[454][100] = Cd[3][14]; Md[454][105] = Cd[3][24]; Md[454][107] = Cd[3][30]; Md[454][154] = Cd[3][1]; Md[454][156] = Cd[3][2]; Md[454][159] = Cd[3][6]; Md[454][160] = Cd[3][9]; Md[454][161] = Cd[3][19]; Md[454][231] = Cd[3][5]; Md[454][237] = Cd[3][15]; Md[454][239] = Cd[3][16]; Md[454][338] = Cd[3][25]; Md[454][385] = Cd[3][33]; Md[454][440] = Cd[3][3]; Md[454][443] = Cd[3][7]; Md[454][444] = Cd[3][10]; Md[454][445] = Cd[3][12]; Md[454][449] = Cd[3][17]; Md[454][450] = Cd[3][20]; Md[454][451] = Cd[3][22]; Md[454][452] = Cd[3][26]; Md[454][453] = Cd[3][28]; Md[454][454] = Cd[3][31]; 
   Md[455][273] = Cd[3][14]; Md[455][274] = Cd[3][24]; Md[455][275] = Cd[3][30]; Md[455][276] = Cd[3][33]; Md[455][324] = Cd[3][1]; Md[455][325] = Cd[3][2]; Md[455][326] = Cd[3][3]; Md[455][327] = Cd[3][5]; Md[455][328] = Cd[3][6]; Md[455][329] = Cd[3][7]; Md[455][330] = Cd[3][9]; Md[455][331] = Cd[3][10]; Md[455][332] = Cd[3][12]; Md[455][333] = Cd[3][15]; Md[455][334] = Cd[3][16]; Md[455][335] = Cd[3][17]; Md[455][336] = Cd[3][19]; Md[455][337] = Cd[3][20]; Md[455][338] = Cd[3][22]; Md[455][339] = Cd[3][25]; Md[455][340] = Cd[3][26]; Md[455][341] = Cd[3][28]; Md[455][455] = Cd[3][31]; 
   Md[456][101] = Cd[3][14]; Md[456][106] = Cd[3][24]; Md[456][108] = Cd[3][33]; Md[456][155] = Cd[3][1]; Md[456][157] = Cd[3][3]; Md[456][158] = Cd[3][5]; Md[456][160] = Cd[3][7]; Md[456][161] = Cd[3][17]; Md[456][238] = Cd[3][15]; Md[456][276] = Cd[3][30]; Md[456][326] = Cd[3][2]; Md[456][329] = Cd[3][6]; Md[456][331] = Cd[3][9]; Md[456][332] = Cd[3][10]; Md[456][335] = Cd[3][16]; Md[456][337] = Cd[3][19]; Md[456][338] = Cd[3][20]; Md[456][340] = Cd[3][25]; Md[456][341] = Cd[3][26]; Md[456][446] = Cd[3][12]; Md[456][452] = Cd[3][22]; Md[456][455] = Cd[3][28]; Md[456][456] = Cd[3][31]; 
   Md[457][102] = Cd[3][14]; Md[457][107] = Cd[3][24]; Md[457][108] = Cd[3][30]; Md[457][156] = Cd[3][1]; Md[457][157] = Cd[3][2]; Md[457][159] = Cd[3][5]; Md[457][160] = Cd[3][6]; Md[457][161] = Cd[3][16]; Md[457][239] = Cd[3][15]; Md[457][332] = Cd[3][9]; Md[457][338] = Cd[3][19]; Md[457][341] = Cd[3][25]; Md[457][386] = Cd[3][33]; Md[457][441] = Cd[3][3]; Md[457][444] = Cd[3][7]; Md[457][446] = Cd[3][10]; Md[457][447] = Cd[3][12]; Md[457][450] = Cd[3][17]; Md[457][452] = Cd[3][20]; Md[457][453] = Cd[3][22]; Md[457][455] = Cd[3][26]; Md[457][456] = Cd[3][28]; Md[457][457] = Cd[3][31]; 
   Md[458][384] = Cd[3][14]; Md[458][385] = Cd[3][24]; Md[458][386] = Cd[3][30]; Md[458][387] = Cd[3][33]; Md[458][440] = Cd[3][1]; Md[458][441] = Cd[3][2]; Md[458][442] = Cd[3][3]; Md[458][443] = Cd[3][5]; Md[458][444] = Cd[3][6]; Md[458][445] = Cd[3][7]; Md[458][446] = Cd[3][9]; Md[458][447] = Cd[3][10]; Md[458][448] = Cd[3][12]; Md[458][449] = Cd[3][15]; Md[458][450] = Cd[3][16]; Md[458][451] = Cd[3][17]; Md[458][452] = Cd[3][19]; Md[458][453] = Cd[3][20]; Md[458][454] = Cd[3][22]; Md[458][455] = Cd[3][25]; Md[458][456] = Cd[3][26]; Md[458][457] = Cd[3][28]; Md[458][458] = Cd[3][31]; 
   Md[459][178] = Cd[3][14]; Md[459][181] = Cd[3][24]; Md[459][182] = Cd[3][30]; Md[459][183] = Cd[3][33]; Md[459][226] = Cd[3][1]; Md[459][227] = Cd[3][2]; Md[459][228] = Cd[3][3]; Md[459][232] = Cd[3][5]; Md[459][233] = Cd[3][6]; Md[459][234] = Cd[3][7]; Md[459][235] = Cd[3][9]; Md[459][236] = Cd[3][10]; Md[459][237] = Cd[3][12]; Md[459][240] = Cd[3][15]; Md[459][241] = Cd[3][16]; Md[459][242] = Cd[3][17]; Md[459][243] = Cd[3][19]; Md[459][244] = Cd[3][20]; Md[459][245] = Cd[3][22]; Md[459][246] = Cd[3][26]; Md[459][247] = Cd[3][28]; Md[459][342] = Cd[3][25]; Md[459][459] = Cd[3][31]; 
   Md[460][179] = Cd[3][14]; Md[460][182] = Cd[3][24]; Md[460][184] = Cd[3][30]; Md[460][185] = Cd[3][33]; Md[460][227] = Cd[3][1]; Md[460][229] = Cd[3][2]; Md[460][230] = Cd[3][3]; Md[460][233] = Cd[3][5]; Md[460][235] = Cd[3][6]; Md[460][236] = Cd[3][7]; Md[460][238] = Cd[3][10]; Md[460][239] = Cd[3][12]; Md[460][241] = Cd[3][15]; Md[460][243] = Cd[3][16]; Md[460][244] = Cd[3][17]; Md[460][246] = Cd[3][20]; Md[460][247] = Cd[3][22]; Md[460][248] = Cd[3][28]; Md[460][333] = Cd[3][9]; Md[460][342] = Cd[3][19]; Md[460][343] = Cd[3][25]; Md[460][344] = Cd[3][26]; Md[460][460] = Cd[3][31]; 
   Md[461][180] = Cd[3][14]; Md[461][183] = Cd[3][24]; Md[461][185] = Cd[3][30]; Md[461][186] = Cd[3][33]; Md[461][228] = Cd[3][1]; Md[461][230] = Cd[3][2]; Md[461][231] = Cd[3][3]; Md[461][234] = Cd[3][5]; Md[461][236] = Cd[3][6]; Md[461][237] = Cd[3][7]; Md[461][238] = Cd[3][9]; Md[461][239] = Cd[3][10]; Md[461][242] = Cd[3][15]; Md[461][244] = Cd[3][16]; Md[461][245] = Cd[3][17]; Md[461][246] = Cd[3][19]; Md[461][247] = Cd[3][20]; Md[461][248] = Cd[3][26]; Md[461][344] = Cd[3][25]; Md[461][449] = Cd[3][12]; Md[461][459] = Cd[3][22]; Md[461][460] = Cd[3][28]; Md[461][461] = Cd[3][31]; 
   Md[462][49] = Cd[3][33]; Md[462][103] = Cd[3][14]; Md[462][158] = Cd[3][3]; Md[462][161] = Cd[3][12]; Md[462][184] = Cd[3][24]; Md[462][229] = Cd[3][1]; Md[462][235] = Cd[3][5]; Md[462][238] = Cd[3][7]; Md[462][243] = Cd[3][15]; Md[462][246] = Cd[3][17]; Md[462][248] = Cd[3][22]; Md[462][277] = Cd[3][30]; Md[462][327] = Cd[3][2]; Md[462][333] = Cd[3][6]; Md[462][334] = Cd[3][9]; Md[462][335] = Cd[3][10]; Md[462][342] = Cd[3][16]; Md[462][343] = Cd[3][19]; Md[462][344] = Cd[3][20]; Md[462][345] = Cd[3][25]; Md[462][346] = Cd[3][26]; Md[462][347] = Cd[3][28]; Md[462][462] = Cd[3][31]; 
   Md[463][49] = Cd[3][30]; Md[463][50] = Cd[3][33]; Md[463][104] = Cd[3][14]; Md[463][158] = Cd[3][2]; Md[463][159] = Cd[3][3]; Md[463][161] = Cd[3][10]; Md[463][185] = Cd[3][24]; Md[463][230] = Cd[3][1]; Md[463][236] = Cd[3][5]; Md[463][238] = Cd[3][6]; Md[463][239] = Cd[3][7]; Md[463][244] = Cd[3][15]; Md[463][246] = Cd[3][16]; Md[463][247] = Cd[3][17]; Md[463][248] = Cd[3][20]; Md[463][335] = Cd[3][9]; Md[463][344] = Cd[3][19]; Md[463][346] = Cd[3][25]; Md[463][347] = Cd[3][26]; Md[463][450] = Cd[3][12]; Md[463][460] = Cd[3][22]; Md[463][462] = Cd[3][28]; Md[463][463] = Cd[3][31]; 
   Md[464][50] = Cd[3][30]; Md[464][105] = Cd[3][14]; Md[464][159] = Cd[3][2]; Md[464][161] = Cd[3][9]; Md[464][186] = Cd[3][24]; Md[464][231] = Cd[3][1]; Md[464][237] = Cd[3][5]; Md[464][239] = Cd[3][6]; Md[464][245] = Cd[3][15]; Md[464][247] = Cd[3][16]; Md[464][248] = Cd[3][19]; Md[464][347] = Cd[3][25]; Md[464][388] = Cd[3][33]; Md[464][443] = Cd[3][3]; Md[464][449] = Cd[3][7]; Md[464][450] = Cd[3][10]; Md[464][451] = Cd[3][12]; Md[464][459] = Cd[3][17]; Md[464][460] = Cd[3][20]; Md[464][461] = Cd[3][22]; Md[464][462] = Cd[3][26]; Md[464][463] = Cd[3][28]; Md[464][464] = Cd[3][31]; 
   Md[465][274] = Cd[3][14]; Md[465][277] = Cd[3][24]; Md[465][278] = Cd[3][30]; Md[465][279] = Cd[3][33]; Md[465][327] = Cd[3][1]; Md[465][328] = Cd[3][2]; Md[465][329] = Cd[3][3]; Md[465][333] = Cd[3][5]; Md[465][334] = Cd[3][6]; Md[465][335] = Cd[3][7]; Md[465][336] = Cd[3][9]; Md[465][337] = Cd[3][10]; Md[465][338] = Cd[3][12]; Md[465][342] = Cd[3][15]; Md[465][343] = Cd[3][16]; Md[465][344] = Cd[3][17]; Md[465][345] = Cd[3][19]; Md[465][346] = Cd[3][20]; Md[465][347] = Cd[3][22]; Md[465][348] = Cd[3][25]; Md[465][349] = Cd[3][26]; Md[465][350] = Cd[3][28]; Md[465][465] = Cd[3][31]; 
   Md[466][49] = Cd[3][24]; Md[466][51] = Cd[3][33]; Md[466][106] = Cd[3][14]; Md[466][158] = Cd[3][1]; Md[466][160] = Cd[3][3]; Md[466][161] = Cd[3][7]; Md[466][238] = Cd[3][5]; Md[466][246] = Cd[3][15]; Md[466][248] = Cd[3][17]; Md[466][279] = Cd[3][30]; Md[466][329] = Cd[3][2]; Md[466][335] = Cd[3][6]; Md[466][337] = Cd[3][9]; Md[466][338] = Cd[3][10]; Md[466][344] = Cd[3][16]; Md[466][346] = Cd[3][19]; Md[466][347] = Cd[3][20]; Md[466][349] = Cd[3][25]; Md[466][350] = Cd[3][26]; Md[466][452] = Cd[3][12]; Md[466][462] = Cd[3][22]; Md[466][465] = Cd[3][28]; Md[466][466] = Cd[3][31]; 
   Md[467][50] = Cd[3][24]; Md[467][51] = Cd[3][30]; Md[467][107] = Cd[3][14]; Md[467][159] = Cd[3][1]; Md[467][160] = Cd[3][2]; Md[467][161] = Cd[3][6]; Md[467][239] = Cd[3][5]; Md[467][247] = Cd[3][15]; Md[467][248] = Cd[3][16]; Md[467][338] = Cd[3][9]; Md[467][347] = Cd[3][19]; Md[467][350] = Cd[3][25]; Md[467][389] = Cd[3][33]; Md[467][444] = Cd[3][3]; Md[467][450] = Cd[3][7]; Md[467][452] = Cd[3][10]; Md[467][453] = Cd[3][12]; Md[467][460] = Cd[3][17]; Md[467][462] = Cd[3][20]; Md[467][463] = Cd[3][22]; Md[467][465] = Cd[3][26]; Md[467][466] = Cd[3][28]; Md[467][467] = Cd[3][31]; 
   Md[468][385] = Cd[3][14]; Md[468][388] = Cd[3][24]; Md[468][389] = Cd[3][30]; Md[468][390] = Cd[3][33]; Md[468][443] = Cd[3][1]; Md[468][444] = Cd[3][2]; Md[468][445] = Cd[3][3]; Md[468][449] = Cd[3][5]; Md[468][450] = Cd[3][6]; Md[468][451] = Cd[3][7]; Md[468][452] = Cd[3][9]; Md[468][453] = Cd[3][10]; Md[468][454] = Cd[3][12]; Md[468][459] = Cd[3][15]; Md[468][460] = Cd[3][16]; Md[468][461] = Cd[3][17]; Md[468][462] = Cd[3][19]; Md[468][463] = Cd[3][20]; Md[468][464] = Cd[3][22]; Md[468][465] = Cd[3][25]; Md[468][466] = Cd[3][26]; Md[468][467] = Cd[3][28]; Md[468][468] = Cd[3][31]; 
   Md[469][275] = Cd[3][14]; Md[469][278] = Cd[3][24]; Md[469][280] = Cd[3][30]; Md[469][281] = Cd[3][33]; Md[469][328] = Cd[3][1]; Md[469][330] = Cd[3][2]; Md[469][331] = Cd[3][3]; Md[469][334] = Cd[3][5]; Md[469][336] = Cd[3][6]; Md[469][337] = Cd[3][7]; Md[469][339] = Cd[3][9]; Md[469][340] = Cd[3][10]; Md[469][341] = Cd[3][12]; Md[469][343] = Cd[3][15]; Md[469][345] = Cd[3][16]; Md[469][346] = Cd[3][17]; Md[469][348] = Cd[3][19]; Md[469][349] = Cd[3][20]; Md[469][350] = Cd[3][22]; Md[469][351] = Cd[3][25]; Md[469][352] = Cd[3][26]; Md[469][353] = Cd[3][28]; Md[469][469] = Cd[3][31]; 
   Md[470][276] = Cd[3][14]; Md[470][279] = Cd[3][24]; Md[470][281] = Cd[3][30]; Md[470][282] = Cd[3][33]; Md[470][329] = Cd[3][1]; Md[470][331] = Cd[3][2]; Md[470][332] = Cd[3][3]; Md[470][335] = Cd[3][5]; Md[470][337] = Cd[3][6]; Md[470][338] = Cd[3][7]; Md[470][340] = Cd[3][9]; Md[470][341] = Cd[3][10]; Md[470][344] = Cd[3][15]; Md[470][346] = Cd[3][16]; Md[470][347] = Cd[3][17]; Md[470][349] = Cd[3][19]; Md[470][350] = Cd[3][20]; Md[470][352] = Cd[3][25]; Md[470][353] = Cd[3][26]; Md[470][455] = Cd[3][12]; Md[470][465] = Cd[3][22]; Md[470][469] = Cd[3][28]; Md[470][470] = Cd[3][31]; 
   Md[471][51] = Cd[3][24]; Md[471][108] = Cd[3][14]; Md[471][160] = Cd[3][1]; Md[471][161] = Cd[3][5]; Md[471][248] = Cd[3][15]; Md[471][282] = Cd[3][30]; Md[471][332] = Cd[3][2]; Md[471][338] = Cd[3][6]; Md[471][341] = Cd[3][9]; Md[471][347] = Cd[3][16]; Md[471][350] = Cd[3][19]; Md[471][353] = Cd[3][25]; Md[471][391] = Cd[3][33]; Md[471][446] = Cd[3][3]; Md[471][452] = Cd[3][7]; Md[471][455] = Cd[3][10]; Md[471][456] = Cd[3][12]; Md[471][462] = Cd[3][17]; Md[471][465] = Cd[3][20]; Md[471][466] = Cd[3][22]; Md[471][469] = Cd[3][26]; Md[471][470] = Cd[3][28]; Md[471][471] = Cd[3][31]; 
   Md[472][386] = Cd[3][14]; Md[472][389] = Cd[3][24]; Md[472][391] = Cd[3][30]; Md[472][392] = Cd[3][33]; Md[472][444] = Cd[3][1]; Md[472][446] = Cd[3][2]; Md[472][447] = Cd[3][3]; Md[472][450] = Cd[3][5]; Md[472][452] = Cd[3][6]; Md[472][453] = Cd[3][7]; Md[472][455] = Cd[3][9]; Md[472][456] = Cd[3][10]; Md[472][457] = Cd[3][12]; Md[472][460] = Cd[3][15]; Md[472][462] = Cd[3][16]; Md[472][463] = Cd[3][17]; Md[472][465] = Cd[3][19]; Md[472][466] = Cd[3][20]; Md[472][467] = Cd[3][22]; Md[472][469] = Cd[3][25]; Md[472][470] = Cd[3][26]; Md[472][471] = Cd[3][28]; Md[472][472] = Cd[3][31]; 
   Md[473][387] = Cd[3][14]; Md[473][390] = Cd[3][24]; Md[473][392] = Cd[3][30]; Md[473][393] = Cd[3][33]; Md[473][445] = Cd[3][1]; Md[473][447] = Cd[3][2]; Md[473][448] = Cd[3][3]; Md[473][451] = Cd[3][5]; Md[473][453] = Cd[3][6]; Md[473][454] = Cd[3][7]; Md[473][456] = Cd[3][9]; Md[473][457] = Cd[3][10]; Md[473][458] = Cd[3][12]; Md[473][461] = Cd[3][15]; Md[473][463] = Cd[3][16]; Md[473][464] = Cd[3][17]; Md[473][466] = Cd[3][19]; Md[473][467] = Cd[3][20]; Md[473][468] = Cd[3][22]; Md[473][470] = Cd[3][25]; Md[473][471] = Cd[3][26]; Md[473][472] = Cd[3][28]; Md[473][473] = Cd[3][31]; 
   Md[474][181] = Cd[3][14]; Md[474][187] = Cd[3][24]; Md[474][188] = Cd[3][30]; Md[474][189] = Cd[3][33]; Md[474][232] = Cd[3][1]; Md[474][233] = Cd[3][2]; Md[474][234] = Cd[3][3]; Md[474][240] = Cd[3][5]; Md[474][241] = Cd[3][6]; Md[474][242] = Cd[3][7]; Md[474][243] = Cd[3][9]; Md[474][244] = Cd[3][10]; Md[474][245] = Cd[3][12]; Md[474][249] = Cd[3][15]; Md[474][250] = Cd[3][16]; Md[474][251] = Cd[3][17]; Md[474][252] = Cd[3][19]; Md[474][253] = Cd[3][20]; Md[474][254] = Cd[3][22]; Md[474][255] = Cd[3][26]; Md[474][256] = Cd[3][28]; Md[474][354] = Cd[3][25]; Md[474][474] = Cd[3][31]; 
   Md[475][182] = Cd[3][14]; Md[475][188] = Cd[3][24]; Md[475][190] = Cd[3][30]; Md[475][191] = Cd[3][33]; Md[475][233] = Cd[3][1]; Md[475][235] = Cd[3][2]; Md[475][236] = Cd[3][3]; Md[475][241] = Cd[3][5]; Md[475][243] = Cd[3][6]; Md[475][244] = Cd[3][7]; Md[475][246] = Cd[3][10]; Md[475][247] = Cd[3][12]; Md[475][250] = Cd[3][15]; Md[475][252] = Cd[3][16]; Md[475][253] = Cd[3][17]; Md[475][255] = Cd[3][20]; Md[475][256] = Cd[3][22]; Md[475][257] = Cd[3][28]; Md[475][342] = Cd[3][9]; Md[475][354] = Cd[3][19]; Md[475][355] = Cd[3][25]; Md[475][356] = Cd[3][26]; Md[475][475] = Cd[3][31]; 
   Md[476][183] = Cd[3][14]; Md[476][189] = Cd[3][24]; Md[476][191] = Cd[3][30]; Md[476][192] = Cd[3][33]; Md[476][234] = Cd[3][1]; Md[476][236] = Cd[3][2]; Md[476][237] = Cd[3][3]; Md[476][242] = Cd[3][5]; Md[476][244] = Cd[3][6]; Md[476][245] = Cd[3][7]; Md[476][246] = Cd[3][9]; Md[476][247] = Cd[3][10]; Md[476][251] = Cd[3][15]; Md[476][253] = Cd[3][16]; Md[476][254] = Cd[3][17]; Md[476][255] = Cd[3][19]; Md[476][256] = Cd[3][20]; Md[476][257] = Cd[3][26]; Md[476][356] = Cd[3][25]; Md[476][459] = Cd[3][12]; Md[476][474] = Cd[3][22]; Md[476][475] = Cd[3][28]; Md[476][476] = Cd[3][31]; 
   Md[477][184] = Cd[3][14]; Md[477][190] = Cd[3][24]; Md[477][193] = Cd[3][33]; Md[477][235] = Cd[3][1]; Md[477][238] = Cd[3][3]; Md[477][243] = Cd[3][5]; Md[477][246] = Cd[3][7]; Md[477][248] = Cd[3][12]; Md[477][252] = Cd[3][15]; Md[477][255] = Cd[3][17]; Md[477][257] = Cd[3][22]; Md[477][283] = Cd[3][30]; Md[477][333] = Cd[3][2]; Md[477][342] = Cd[3][6]; Md[477][343] = Cd[3][9]; Md[477][344] = Cd[3][10]; Md[477][354] = Cd[3][16]; Md[477][355] = Cd[3][19]; Md[477][356] = Cd[3][20]; Md[477][357] = Cd[3][25]; Md[477][358] = Cd[3][26]; Md[477][359] = Cd[3][28]; Md[477][477] = Cd[3][31]; 
   Md[478][185] = Cd[3][14]; Md[478][191] = Cd[3][24]; Md[478][193] = Cd[3][30]; Md[478][194] = Cd[3][33]; Md[478][236] = Cd[3][1]; Md[478][238] = Cd[3][2]; Md[478][239] = Cd[3][3]; Md[478][244] = Cd[3][5]; Md[478][246] = Cd[3][6]; Md[478][247] = Cd[3][7]; Md[478][248] = Cd[3][10]; Md[478][253] = Cd[3][15]; Md[478][255] = Cd[3][16]; Md[478][256] = Cd[3][17]; Md[478][257] = Cd[3][20]; Md[478][344] = Cd[3][9]; Md[478][356] = Cd[3][19]; Md[478][358] = Cd[3][25]; Md[478][359] = Cd[3][26]; Md[478][460] = Cd[3][12]; Md[478][475] = Cd[3][22]; Md[478][477] = Cd[3][28]; Md[478][478] = Cd[3][31]; 
   Md[479][186] = Cd[3][14]; Md[479][192] = Cd[3][24]; Md[479][194] = Cd[3][30]; Md[479][237] = Cd[3][1]; Md[479][239] = Cd[3][2]; Md[479][245] = Cd[3][5]; Md[479][247] = Cd[3][6]; Md[479][248] = Cd[3][9]; Md[479][254] = Cd[3][15]; Md[479][256] = Cd[3][16]; Md[479][257] = Cd[3][19]; Md[479][359] = Cd[3][25]; Md[479][394] = Cd[3][33]; Md[479][449] = Cd[3][3]; Md[479][459] = Cd[3][7]; Md[479][460] = Cd[3][10]; Md[479][461] = Cd[3][12]; Md[479][474] = Cd[3][17]; Md[479][475] = Cd[3][20]; Md[479][476] = Cd[3][22]; Md[479][477] = Cd[3][26]; Md[479][478] = Cd[3][28]; Md[479][479] = Cd[3][31]; 
   Md[480][277] = Cd[3][14]; Md[480][283] = Cd[3][24]; Md[480][284] = Cd[3][30]; Md[480][285] = Cd[3][33]; Md[480][333] = Cd[3][1]; Md[480][334] = Cd[3][2]; Md[480][335] = Cd[3][3]; Md[480][342] = Cd[3][5]; Md[480][343] = Cd[3][6]; Md[480][344] = Cd[3][7]; Md[480][345] = Cd[3][9]; Md[480][346] = Cd[3][10]; Md[480][347] = Cd[3][12]; Md[480][354] = Cd[3][15]; Md[480][355] = Cd[3][16]; Md[480][356] = Cd[3][17]; Md[480][357] = Cd[3][19]; Md[480][358] = Cd[3][20]; Md[480][359] = Cd[3][22]; Md[480][360] = Cd[3][25]; Md[480][361] = Cd[3][26]; Md[480][362] = Cd[3][28]; Md[480][480] = Cd[3][31]; 
   Md[481][49] = Cd[3][14]; Md[481][52] = Cd[3][33]; Md[481][161] = Cd[3][3]; Md[481][193] = Cd[3][24]; Md[481][238] = Cd[3][1]; Md[481][246] = Cd[3][5]; Md[481][248] = Cd[3][7]; Md[481][255] = Cd[3][15]; Md[481][257] = Cd[3][17]; Md[481][285] = Cd[3][30]; Md[481][335] = Cd[3][2]; Md[481][344] = Cd[3][6]; Md[481][346] = Cd[3][9]; Md[481][347] = Cd[3][10]; Md[481][356] = Cd[3][16]; Md[481][358] = Cd[3][19]; Md[481][359] = Cd[3][20]; Md[481][361] = Cd[3][25]; Md[481][362] = Cd[3][26]; Md[481][462] = Cd[3][12]; Md[481][477] = Cd[3][22]; Md[481][480] = Cd[3][28]; Md[481][481] = Cd[3][31]; 
   Md[482][50] = Cd[3][14]; Md[482][52] = Cd[3][30]; Md[482][161] = Cd[3][2]; Md[482][194] = Cd[3][24]; Md[482][239] = Cd[3][1]; Md[482][247] = Cd[3][5]; Md[482][248] = Cd[3][6]; Md[482][256] = Cd[3][15]; Md[482][257] = Cd[3][16]; Md[482][347] = Cd[3][9]; Md[482][359] = Cd[3][19]; Md[482][362] = Cd[3][25]; Md[482][395] = Cd[3][33]; Md[482][450] = Cd[3][3]; Md[482][460] = Cd[3][7]; Md[482][462] = Cd[3][10]; Md[482][463] = Cd[3][12]; Md[482][475] = Cd[3][17]; Md[482][477] = Cd[3][20]; Md[482][478] = Cd[3][22]; Md[482][480] = Cd[3][26]; Md[482][481] = Cd[3][28]; Md[482][482] = Cd[3][31]; 
   Md[483][388] = Cd[3][14]; Md[483][394] = Cd[3][24]; Md[483][395] = Cd[3][30]; Md[483][396] = Cd[3][33]; Md[483][449] = Cd[3][1]; Md[483][450] = Cd[3][2]; Md[483][451] = Cd[3][3]; Md[483][459] = Cd[3][5]; Md[483][460] = Cd[3][6]; Md[483][461] = Cd[3][7]; Md[483][462] = Cd[3][9]; Md[483][463] = Cd[3][10]; Md[483][464] = Cd[3][12]; Md[483][474] = Cd[3][15]; Md[483][475] = Cd[3][16]; Md[483][476] = Cd[3][17]; Md[483][477] = Cd[3][19]; Md[483][478] = Cd[3][20]; Md[483][479] = Cd[3][22]; Md[483][480] = Cd[3][25]; Md[483][481] = Cd[3][26]; Md[483][482] = Cd[3][28]; Md[483][483] = Cd[3][31]; 
   Md[484][278] = Cd[3][14]; Md[484][284] = Cd[3][24]; Md[484][286] = Cd[3][30]; Md[484][287] = Cd[3][33]; Md[484][334] = Cd[3][1]; Md[484][336] = Cd[3][2]; Md[484][337] = Cd[3][3]; Md[484][343] = Cd[3][5]; Md[484][345] = Cd[3][6]; Md[484][346] = Cd[3][7]; Md[484][348] = Cd[3][9]; Md[484][349] = Cd[3][10]; Md[484][350] = Cd[3][12]; Md[484][355] = Cd[3][15]; Md[484][357] = Cd[3][16]; Md[484][358] = Cd[3][17]; Md[484][360] = Cd[3][19]; Md[484][361] = Cd[3][20]; Md[484][362] = Cd[3][22]; Md[484][363] = Cd[3][25]; Md[484][364] = Cd[3][26]; Md[484][365] = Cd[3][28]; Md[484][484] = Cd[3][31]; 
   Md[485][279] = Cd[3][14]; Md[485][285] = Cd[3][24]; Md[485][287] = Cd[3][30]; Md[485][288] = Cd[3][33]; Md[485][335] = Cd[3][1]; Md[485][337] = Cd[3][2]; Md[485][338] = Cd[3][3]; Md[485][344] = Cd[3][5]; Md[485][346] = Cd[3][6]; Md[485][347] = Cd[3][7]; Md[485][349] = Cd[3][9]; Md[485][350] = Cd[3][10]; Md[485][356] = Cd[3][15]; Md[485][358] = Cd[3][16]; Md[485][359] = Cd[3][17]; Md[485][361] = Cd[3][19]; Md[485][362] = Cd[3][20]; Md[485][364] = Cd[3][25]; Md[485][365] = Cd[3][26]; Md[485][465] = Cd[3][12]; Md[485][480] = Cd[3][22]; Md[485][484] = Cd[3][28]; Md[485][485] = Cd[3][31]; 
   Md[486][51] = Cd[3][14]; Md[486][52] = Cd[3][24]; Md[486][161] = Cd[3][1]; Md[486][248] = Cd[3][5]; Md[486][257] = Cd[3][15]; Md[486][288] = Cd[3][30]; Md[486][338] = Cd[3][2]; Md[486][347] = Cd[3][6]; Md[486][350] = Cd[3][9]; Md[486][359] = Cd[3][16]; Md[486][362] = Cd[3][19]; Md[486][365] = Cd[3][25]; Md[486][397] = Cd[3][33]; Md[486][452] = Cd[3][3]; Md[486][462] = Cd[3][7]; Md[486][465] = Cd[3][10]; Md[486][466] = Cd[3][12]; Md[486][477] = Cd[3][17]; Md[486][480] = Cd[3][20]; Md[486][481] = Cd[3][22]; Md[486][484] = Cd[3][26]; Md[486][485] = Cd[3][28]; Md[486][486] = Cd[3][31]; 
   Md[487][389] = Cd[3][14]; Md[487][395] = Cd[3][24]; Md[487][397] = Cd[3][30]; Md[487][398] = Cd[3][33]; Md[487][450] = Cd[3][1]; Md[487][452] = Cd[3][2]; Md[487][453] = Cd[3][3]; Md[487][460] = Cd[3][5]; Md[487][462] = Cd[3][6]; Md[487][463] = Cd[3][7]; Md[487][465] = Cd[3][9]; Md[487][466] = Cd[3][10]; Md[487][467] = Cd[3][12]; Md[487][475] = Cd[3][15]; Md[487][477] = Cd[3][16]; Md[487][478] = Cd[3][17]; Md[487][480] = Cd[3][19]; Md[487][481] = Cd[3][20]; Md[487][482] = Cd[3][22]; Md[487][484] = Cd[3][25]; Md[487][485] = Cd[3][26]; Md[487][486] = Cd[3][28]; Md[487][487] = Cd[3][31]; 
   Md[488][390] = Cd[3][14]; Md[488][396] = Cd[3][24]; Md[488][398] = Cd[3][30]; Md[488][399] = Cd[3][33]; Md[488][451] = Cd[3][1]; Md[488][453] = Cd[3][2]; Md[488][454] = Cd[3][3]; Md[488][461] = Cd[3][5]; Md[488][463] = Cd[3][6]; Md[488][464] = Cd[3][7]; Md[488][466] = Cd[3][9]; Md[488][467] = Cd[3][10]; Md[488][468] = Cd[3][12]; Md[488][476] = Cd[3][15]; Md[488][478] = Cd[3][16]; Md[488][479] = Cd[3][17]; Md[488][481] = Cd[3][19]; Md[488][482] = Cd[3][20]; Md[488][483] = Cd[3][22]; Md[488][485] = Cd[3][25]; Md[488][486] = Cd[3][26]; Md[488][487] = Cd[3][28]; Md[488][488] = Cd[3][31]; 
   Md[489][280] = Cd[3][14]; Md[489][286] = Cd[3][24]; Md[489][289] = Cd[3][30]; Md[489][290] = Cd[3][33]; Md[489][336] = Cd[3][1]; Md[489][339] = Cd[3][2]; Md[489][340] = Cd[3][3]; Md[489][345] = Cd[3][5]; Md[489][348] = Cd[3][6]; Md[489][349] = Cd[3][7]; Md[489][351] = Cd[3][9]; Md[489][352] = Cd[3][10]; Md[489][353] = Cd[3][12]; Md[489][357] = Cd[3][15]; Md[489][360] = Cd[3][16]; Md[489][361] = Cd[3][17]; Md[489][363] = Cd[3][19]; Md[489][364] = Cd[3][20]; Md[489][365] = Cd[3][22]; Md[489][366] = Cd[3][25]; Md[489][367] = Cd[3][26]; Md[489][368] = Cd[3][28]; Md[489][489] = Cd[3][31]; 
   Md[490][281] = Cd[3][14]; Md[490][287] = Cd[3][24]; Md[490][290] = Cd[3][30]; Md[490][291] = Cd[3][33]; Md[490][337] = Cd[3][1]; Md[490][340] = Cd[3][2]; Md[490][341] = Cd[3][3]; Md[490][346] = Cd[3][5]; Md[490][349] = Cd[3][6]; Md[490][350] = Cd[3][7]; Md[490][352] = Cd[3][9]; Md[490][353] = Cd[3][10]; Md[490][358] = Cd[3][15]; Md[490][361] = Cd[3][16]; Md[490][362] = Cd[3][17]; Md[490][364] = Cd[3][19]; Md[490][365] = Cd[3][20]; Md[490][367] = Cd[3][25]; Md[490][368] = Cd[3][26]; Md[490][469] = Cd[3][12]; Md[490][484] = Cd[3][22]; Md[490][489] = Cd[3][28]; Md[490][490] = Cd[3][31]; 
   Md[491][282] = Cd[3][14]; Md[491][288] = Cd[3][24]; Md[491][291] = Cd[3][30]; Md[491][338] = Cd[3][1]; Md[491][341] = Cd[3][2]; Md[491][347] = Cd[3][5]; Md[491][350] = Cd[3][6]; Md[491][353] = Cd[3][9]; Md[491][359] = Cd[3][15]; Md[491][362] = Cd[3][16]; Md[491][365] = Cd[3][19]; Md[491][368] = Cd[3][25]; Md[491][400] = Cd[3][33]; Md[491][455] = Cd[3][3]; Md[491][465] = Cd[3][7]; Md[491][469] = Cd[3][10]; Md[491][470] = Cd[3][12]; Md[491][480] = Cd[3][17]; Md[491][484] = Cd[3][20]; Md[491][485] = Cd[3][22]; Md[491][489] = Cd[3][26]; Md[491][490] = Cd[3][28]; Md[491][491] = Cd[3][31]; 
   Md[492][391] = Cd[3][14]; Md[492][397] = Cd[3][24]; Md[492][400] = Cd[3][30]; Md[492][401] = Cd[3][33]; Md[492][452] = Cd[3][1]; Md[492][455] = Cd[3][2]; Md[492][456] = Cd[3][3]; Md[492][462] = Cd[3][5]; Md[492][465] = Cd[3][6]; Md[492][466] = Cd[3][7]; Md[492][469] = Cd[3][9]; Md[492][470] = Cd[3][10]; Md[492][471] = Cd[3][12]; Md[492][477] = Cd[3][15]; Md[492][480] = Cd[3][16]; Md[492][481] = Cd[3][17]; Md[492][484] = Cd[3][19]; Md[492][485] = Cd[3][20]; Md[492][486] = Cd[3][22]; Md[492][489] = Cd[3][25]; Md[492][490] = Cd[3][26]; Md[492][491] = Cd[3][28]; Md[492][492] = Cd[3][31]; 
   Md[493][392] = Cd[3][14]; Md[493][398] = Cd[3][24]; Md[493][401] = Cd[3][30]; Md[493][402] = Cd[3][33]; Md[493][453] = Cd[3][1]; Md[493][456] = Cd[3][2]; Md[493][457] = Cd[3][3]; Md[493][463] = Cd[3][5]; Md[493][466] = Cd[3][6]; Md[493][467] = Cd[3][7]; Md[493][470] = Cd[3][9]; Md[493][471] = Cd[3][10]; Md[493][472] = Cd[3][12]; Md[493][478] = Cd[3][15]; Md[493][481] = Cd[3][16]; Md[493][482] = Cd[3][17]; Md[493][485] = Cd[3][19]; Md[493][486] = Cd[3][20]; Md[493][487] = Cd[3][22]; Md[493][490] = Cd[3][25]; Md[493][491] = Cd[3][26]; Md[493][492] = Cd[3][28]; Md[493][493] = Cd[3][31]; 
   Md[494][393] = Cd[3][14]; Md[494][399] = Cd[3][24]; Md[494][402] = Cd[3][30]; Md[494][403] = Cd[3][33]; Md[494][454] = Cd[3][1]; Md[494][457] = Cd[3][2]; Md[494][458] = Cd[3][3]; Md[494][464] = Cd[3][5]; Md[494][467] = Cd[3][6]; Md[494][468] = Cd[3][7]; Md[494][471] = Cd[3][9]; Md[494][472] = Cd[3][10]; Md[494][473] = Cd[3][12]; Md[494][479] = Cd[3][15]; Md[494][482] = Cd[3][16]; Md[494][483] = Cd[3][17]; Md[494][486] = Cd[3][19]; Md[494][487] = Cd[3][20]; Md[494][488] = Cd[3][22]; Md[494][491] = Cd[3][25]; Md[494][492] = Cd[3][26]; Md[494][493] = Cd[3][28]; Md[494][494] = Cd[3][31]; 

   // error = rox_array2d_double_save("C.txt", C);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_array2d_double_save("M.txt", M);
   // ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   return error;
}

Rox_ErrorCode rox_solve_gibbs_parametrization_numeric_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double Ar, const Rox_Array2D_Double br)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double C = NULL;

   if (!Ar || !br || !Re ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&C, 3, 35);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_compute_coefficients_gibbs_parametrization_points3d_to_planes3d(C, Ar, br);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_normalize_rows(C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_solve_gibbs_equations_system_macaulay_resultant_numeric_points3d_to_planes3d(Re, C);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&C);

   return error;
}

Rox_ErrorCode rox_array2d_double_normalize_rows(Rox_Array2D_Double C)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint rows = 0;
   Rox_Sint cols = 0;

   if ( !C ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** Cd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Cd, C );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_size(&rows, &cols, C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint r = 0; r < rows; r++)
   {
      Rox_Double norm2 = 0;

      for (Rox_Sint c = 0; c < cols; c++)
      {
         norm2 += Cd[r][c]*Cd[r][c];
      }

      Rox_Double norm = sqrt(norm2);

      for (Rox_Sint c = 0; c < cols; c++)
      {
         Cd[r][c] = Cd[r][c]/norm;
      }
   }

function_terminate:
   // rox_array2d_double_del(&C);

   return error;
}

Rox_ErrorCode rox_compute_coefficients_gibbs_parametrization_points3d_to_planes3d(Rox_Array2D_Double C, const Rox_Array2D_Double Ar, Rox_Array2D_Double br)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!Ar || !br || !C ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** Cd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Cd, C );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** Ad = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &Ad, Ar );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** bd = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &bd, br );
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

Rox_ErrorCode rox_solve_gibbs_equations_system_macaulay_resultant_numeric_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double C)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double u = NULL;
   Rox_Array2D_Double M = NULL;

   if (!Re || !C ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&u, 1, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&M, 120, 120);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matrix_macaulay_gibbs_points3d_to_planes3d(M, u, C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_extract_real_gibbs_solutions_macaulay_points3d_to_planes3d(Re, M);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_array2d_double_del(&M);
   rox_array2d_double_del(&u);

   return error;
}

Rox_ErrorCode rox_matrix_macaulay_gibbs_points3d_to_planes3d(Rox_Array2D_Double M, const Rox_Array2D_Double u, const Rox_Array2D_Double C)
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

   Md[ 0][ 0] = 1.0; Md[ 0][ 1] = 1.0; Md[ 0][ 2] = 1.0; Md[ 0][ 3] = 1.0;
   Md[ 1][ 1] = 1.0; Md[ 1][ 4] = 1.0; Md[ 1][ 5] = 1.0; Md[ 1][ 6] = 1.0;
   Md[ 2][ 2] = 1.0; Md[ 2][ 5] = 1.0; Md[ 2][ 7] = 1.0; Md[ 2][ 8] = 1.0;
   Md[ 3][ 3] = 1.0; Md[ 3][ 6] = 1.0; Md[ 3][ 8] = 1.0; Md[ 3][ 9] = 1.0;
   Md[ 4][ 4] = 1.0; Md[ 4][10] = 1.0; Md[ 4][11] = 1.0; Md[ 4][27] = 1.0;
   Md[ 5][ 5] = 1.0; Md[ 5][10] = 1.0; Md[ 5][12] = 1.0; Md[ 5][13] = 1.0;
   Md[ 6][ 6] = 1.0; Md[ 6][11] = 1.0; Md[ 6][13] = 1.0; Md[ 6][14] = 1.0;
   Md[ 7][ 7] = 1.0; Md[ 7][12] = 1.0; Md[ 7][15] = 1.0; Md[ 7][54] = 1.0;
   Md[ 8][ 8] = 1.0; Md[ 8][13] = 1.0; Md[ 8][15] = 1.0; Md[ 8][16] = 1.0;
   Md[ 9][ 9] = 1.0; Md[ 9][14] = 1.0; Md[ 9][16] = 1.0; Md[ 9][85] = 1.0; 
   Md[10][10] = 1.0; Md[10][17] = 1.0; Md[10][18] = 1.0; Md[10][29] = 1.0;
   Md[11][11] = 1.0; Md[11][18] = 1.0; Md[11][19] = 1.0; Md[11][30] = 1.0;
   Md[12][12] = 1.0; Md[12][17] = 1.0; Md[12][20] = 1.0; Md[12][55] = 1.0;
   Md[13][13] = 1.0; Md[13][18] = 1.0; Md[13][20] = 1.0; Md[13][21] = 1.0;
   Md[14][14] = 1.0; Md[14][19] = 1.0; Md[14][21] = 1.0; Md[14][86] = 1.0;
   Md[15][15] = 1.0; Md[15][20] = 1.0; Md[15][22] = 1.0; Md[15][57] = 1.0;
   Md[16][16] = 1.0; Md[16][21] = 1.0; Md[16][22] = 1.0; Md[16][87] = 1.0;
   Md[17][17] = 1.0; Md[17][23] = 1.0; Md[17][34] = 1.0; Md[17][58] = 1.0;
   Md[18][18] = 1.0; Md[18][23] = 1.0; Md[18][24] = 1.0; Md[18][35] = 1.0;
   Md[19][19] = 1.0; Md[19][24] = 1.0; Md[19][36] = 1.0; Md[19][89] = 1.0;
   Md[20][20] = 1.0; Md[20][23] = 1.0; Md[20][25] = 1.0; Md[20][60] = 1.0;
   Md[21][21] = 1.0; Md[21][24] = 1.0; Md[21][25] = 1.0; Md[21][90] = 1.0;
   Md[22][22] = 1.0; Md[22][25] = 1.0; Md[22][63] = 1.0; Md[22][92] = 1.0;
   Md[23][23] = 1.0; Md[23][26] = 1.0; Md[23][43] = 1.0; Md[23][66] = 1.0;
   Md[24][24] = 1.0; Md[24][26] = 1.0; Md[24][44] = 1.0; Md[24][96] = 1.0;
   Md[25][25] = 1.0; Md[25][26] = 1.0; Md[25][69] = 1.0; Md[25][98] = 1.0;
   Md[26][26] = 1.0; Md[26][53] = 1.0; Md[26][78] = 1.0; Md[26][108] = 1.0;

   Md[27][0] = Cd[0][19]; Md[27][1] = Cd[0][9]; Md[27][2] = Cd[0][15]; Md[27][3] = Cd[0][18]; Md[27][4] = Cd[0][3]; Md[27][5] = Cd[0][6]; Md[27][6] = Cd[0][8]; Md[27][7] = Cd[0][12]; Md[27][8] = Cd[0][14]; Md[27][9] = Cd[0][17]; Md[27][10] = Cd[0][1]; Md[27][11] = Cd[0][2]; Md[27][12] = Cd[0][4]; Md[27][13] = Cd[0][5]; Md[27][14] = Cd[0][7]; Md[27][15] = Cd[0][11]; Md[27][16] = Cd[0][13]; Md[27][27] = Cd[0][0]; Md[27][54] = Cd[0][10]; Md[27][85] = Cd[0][16];

   Md[28][1] = Cd[0][19]; Md[28][4] = Cd[0][9]; Md[28][5] = Cd[0][15]; Md[28][6] = Cd[0][18]; Md[28][10] = Cd[0][6]; Md[28][11] = Cd[0][8]; Md[28][12] = Cd[0][12]; Md[28][13] = Cd[0][14]; Md[28][14] = Cd[0][17]; Md[28][17] = Cd[0][4]; Md[28][18] = Cd[0][5]; Md[28][19] = Cd[0][7]; Md[28][20] = Cd[0][11]; Md[28][21] = Cd[0][13]; Md[28][27] = Cd[0][3]; Md[28][28] = Cd[0][0]; Md[28][29] = Cd[0][1]; Md[28][30] = Cd[0][2]; Md[28][55] = Cd[0][10]; Md[28][86] = Cd[0][16]; Md[29][2] = Cd[0][19]; Md[29][5] = Cd[0][9]; Md[29][7] = Cd[0][15]; Md[29][8] = Cd[0][18]; Md[29][10] = Cd[0][3]; Md[29][12] = Cd[0][6]; Md[29][13] = Cd[0][8]; Md[29][15] = Cd[0][14]; Md[29][16] = Cd[0][17]; Md[29][17] = Cd[0][1]; Md[29][18] = Cd[0][2]; Md[29][20] = Cd[0][5];
   Md[29][21] = Cd[0][7]; Md[29][22] = Cd[0][13]; Md[29][29] = Cd[0][0]; Md[29][54] = Cd[0][12]; Md[29][55] = Cd[0][4]; Md[29][56] = Cd[0][10]; Md[29][57] = Cd[0][11]; Md[29][87] = Cd[0][16];  
   Md[30][3] = Cd[0][19]; Md[30][6] = Cd[0][9]; Md[30][8] = Cd[0][15]; Md[30][9] = Cd[0][18];
   Md[30][11] = Cd[0][3]; Md[30][13] = Cd[0][6]; Md[30][14] = Cd[0][8]; Md[30][15] = Cd[0][12];
   Md[30][16] = Cd[0][14]; Md[30][18] = Cd[0][1]; Md[30][19] = Cd[0][2]; Md[30][20] = Cd[0][4];
   Md[30][21] = Cd[0][5]; Md[30][22] = Cd[0][11]; Md[30][30] = Cd[0][0]; Md[30][57] = Cd[0][10];
   Md[30][85] = Cd[0][17]; Md[30][86] = Cd[0][7]; Md[30][87] = Cd[0][13]; Md[30][88] = Cd[0][16];

   Md[31][4] = Cd[0][19]; Md[31][10] = Cd[0][15]; Md[31][11] = Cd[0][18]; Md[31][17] = Cd[0][12];
   Md[31][18] = Cd[0][14]; Md[31][19] = Cd[0][17]; Md[31][23] = Cd[0][11]; Md[31][24] = Cd[0][13];
   Md[31][27] = Cd[0][9]; Md[31][28] = Cd[0][3]; Md[31][29] = Cd[0][6]; Md[31][30] = Cd[0][8];
   Md[31][31] = Cd[0][0]; Md[31][32] = Cd[0][1]; Md[31][33] = Cd[0][2]; Md[31][34] = Cd[0][4];
   Md[31][35] = Cd[0][5]; Md[31][36] = Cd[0][7]; Md[31][58] = Cd[0][10]; Md[31][89] = Cd[0][16];

   Md[32][5] = Cd[0][19]; Md[32][10] = Cd[0][9]; Md[32][12] = Cd[0][15]; Md[32][13] = Cd[0][18];
   Md[32][17] = Cd[0][6]; Md[32][18] = Cd[0][8]; Md[32][20] = Cd[0][14]; Md[32][21] = Cd[0][17];
   Md[32][23] = Cd[0][5]; Md[32][24] = Cd[0][7]; Md[32][25] = Cd[0][13]; Md[32][29] = Cd[0][3];
   Md[32][32] = Cd[0][0]; Md[32][34] = Cd[0][1]; Md[32][35] = Cd[0][2]; Md[32][55] = Cd[0][12];
   Md[32][58] = Cd[0][4]; Md[32][59] = Cd[0][10]; Md[32][60] = Cd[0][11]; Md[32][90] = Cd[0][16];

   Md[33][6] = Cd[0][19]; Md[33][11] = Cd[0][9]; Md[33][13] = Cd[0][15]; Md[33][14] = Cd[0][18];
   Md[33][18] = Cd[0][6]; Md[33][19] = Cd[0][8]; Md[33][20] = Cd[0][12]; Md[33][21] = Cd[0][14];
   Md[33][23] = Cd[0][4]; Md[33][24] = Cd[0][5]; Md[33][25] = Cd[0][11]; Md[33][30] = Cd[0][3];
  Md[33][33] = Cd[0][0]; Md[33][35] = Cd[0][1]; Md[33][36] = Cd[0][2]; Md[33][60] = Cd[0][10];
  Md[33][86] = Cd[0][17]; Md[33][89] = Cd[0][7]; Md[33][90] = Cd[0][13]; Md[33][91] = Cd[0][16];

  Md[34][7] = Cd[0][19]; Md[34][12] = Cd[0][9]; Md[34][15] = Cd[0][18]; Md[34][17] = Cd[0][3];
  Md[34][20] = Cd[0][8]; Md[34][22] = Cd[0][17]; Md[34][23] = Cd[0][2]; Md[34][25] = Cd[0][7];
  Md[34][34] = Cd[0][0]; Md[34][54] = Cd[0][15]; Md[34][55] = Cd[0][6]; Md[34][56] = Cd[0][12];
  Md[34][57] = Cd[0][14]; Md[34][58] = Cd[0][1]; Md[34][59] = Cd[0][4]; Md[34][60] = Cd[0][5];
  Md[34][61] = Cd[0][10]; Md[34][62] = Cd[0][11]; Md[34][63] = Cd[0][13]; Md[34][92] = Cd[0][16];

  Md[35][8] = Cd[0][19]; Md[35][13] = Cd[0][9]; Md[35][15] = Cd[0][15]; Md[35][16] = Cd[0][18];
  Md[35][18] = Cd[0][3]; Md[35][20] = Cd[0][6]; Md[35][21] = Cd[0][8]; Md[35][22] = Cd[0][14];
  Md[35][23] = Cd[0][1]; Md[35][24] = Cd[0][2]; Md[35][25] = Cd[0][5]; Md[35][35] = Cd[0][0];
  Md[35][57] = Cd[0][12]; Md[35][60] = Cd[0][4]; Md[35][62] = Cd[0][10]; Md[35][63] = Cd[0][11];
  Md[35][87] = Cd[0][17]; Md[35][90] = Cd[0][7]; Md[35][92] = Cd[0][13]; Md[35][93] = Cd[0][16];

  Md[36][9] = Cd[0][19]; Md[36][14] = Cd[0][9]; Md[36][16] = Cd[0][15]; Md[36][19] = Cd[0][3];
  Md[36][21] = Cd[0][6]; Md[36][22] = Cd[0][12]; Md[36][24] = Cd[0][1]; Md[36][25] = Cd[0][4];
  Md[36][36] = Cd[0][0]; Md[36][63] = Cd[0][10]; Md[36][85] = Cd[0][18]; Md[36][86] = Cd[0][8];
  Md[36][87] = Cd[0][14]; Md[36][88] = Cd[0][17]; Md[36][89] = Cd[0][2]; Md[36][90] = Cd[0][5];
  Md[36][91] = Cd[0][7]; Md[36][92] = Cd[0][11]; Md[36][93] = Cd[0][13]; Md[36][94] = Cd[0][16];

  Md[37][27] = Cd[0][19]; Md[37][28] = Cd[0][9]; Md[37][29] = Cd[0][15]; Md[37][30] = Cd[0][18];
  Md[37][31] = Cd[0][3]; Md[37][32] = Cd[0][6]; Md[37][33] = Cd[0][8]; Md[37][34] = Cd[0][12];
  Md[37][35] = Cd[0][14]; Md[37][36] = Cd[0][17]; Md[37][37] = Cd[0][0]; Md[37][38] = Cd[0][1];
  Md[37][39] = Cd[0][2]; Md[37][40] = Cd[0][4]; Md[37][41] = Cd[0][5]; Md[37][42] = Cd[0][7];
  Md[37][43] = Cd[0][11]; Md[37][44] = Cd[0][13]; Md[37][64] = Cd[0][10]; Md[37][95] = Cd[0][16];

  Md[38][10] = Cd[0][19]; Md[38][17] = Cd[0][15]; Md[38][18] = Cd[0][18]; Md[38][23] = Cd[0][14];
  Md[38][24] = Cd[0][17]; Md[38][26] = Cd[0][13]; Md[38][29] = Cd[0][9]; Md[38][32] = Cd[0][3];
  Md[38][34] = Cd[0][6]; Md[38][35] = Cd[0][8]; Md[38][38] = Cd[0][0]; Md[38][40] = Cd[0][1];
  Md[38][41] = Cd[0][2]; Md[38][43] = Cd[0][5]; Md[38][44] = Cd[0][7]; Md[38][58] = Cd[0][12];
  Md[38][64] = Cd[0][4]; Md[38][65] = Cd[0][10]; Md[38][66] = Cd[0][11]; Md[38][96] = Cd[0][16];

  Md[39][11] = Cd[0][19]; Md[39][18] = Cd[0][15]; Md[39][19] = Cd[0][18]; Md[39][23] = Cd[0][12];
  Md[39][24] = Cd[0][14]; Md[39][26] = Cd[0][11]; Md[39][30] = Cd[0][9]; Md[39][33] = Cd[0][3];
  Md[39][35] = Cd[0][6]; Md[39][36] = Cd[0][8]; Md[39][39] = Cd[0][0]; Md[39][41] = Cd[0][1];
  Md[39][42] = Cd[0][2]; Md[39][43] = Cd[0][4]; Md[39][44] = Cd[0][5]; Md[39][66] = Cd[0][10];
  Md[39][89] = Cd[0][17]; Md[39][95] = Cd[0][7]; Md[39][96] = Cd[0][13]; Md[39][97] = Cd[0][16];

  Md[40][12] = Cd[0][19]; Md[40][17] = Cd[0][9]; Md[40][20] = Cd[0][18]; Md[40][23] = Cd[0][8];
  Md[40][25] = Cd[0][17]; Md[40][26] = Cd[0][7]; Md[40][34] = Cd[0][3]; Md[40][40] = Cd[0][0];
  Md[40][43] = Cd[0][2]; Md[40][55] = Cd[0][15]; Md[40][58] = Cd[0][6]; Md[40][59] = Cd[0][12];
  Md[40][60] = Cd[0][14]; Md[40][64] = Cd[0][1]; Md[40][65] = Cd[0][4]; Md[40][66] = Cd[0][5];
  Md[40][67] = Cd[0][10]; Md[40][68] = Cd[0][11]; Md[40][69] = Cd[0][13]; Md[40][98] = Cd[0][16];

  Md[41][13] = Cd[0][19]; Md[41][18] = Cd[0][9]; Md[41][20] = Cd[0][15]; Md[41][21] = Cd[0][18];
  Md[41][23] = Cd[0][6]; Md[41][24] = Cd[0][8]; Md[41][25] = Cd[0][14]; Md[41][26] = Cd[0][5];
  Md[41][35] = Cd[0][3]; Md[41][41] = Cd[0][0]; Md[41][43] = Cd[0][1]; Md[41][44] = Cd[0][2];
  Md[41][60] = Cd[0][12]; Md[41][66] = Cd[0][4]; Md[41][68] = Cd[0][10]; Md[41][69] = Cd[0][11];
  Md[41][90] = Cd[0][17]; Md[41][96] = Cd[0][7]; Md[41][98] = Cd[0][13]; Md[41][99] = Cd[0][16];

  Md[42][14] = Cd[0][19]; Md[42][19] = Cd[0][9]; Md[42][21] = Cd[0][15]; Md[42][24] = Cd[0][6];
  Md[42][25] = Cd[0][12]; Md[42][26] = Cd[0][4]; Md[42][36] = Cd[0][3]; Md[42][42] = Cd[0][0];
  Md[42][44] = Cd[0][1]; Md[42][69] = Cd[0][10]; Md[42][86] = Cd[0][18]; Md[42][89] = Cd[0][8];
  Md[42][90] = Cd[0][14]; Md[42][91] = Cd[0][17]; Md[42][95] = Cd[0][2]; Md[42][96] = Cd[0][5];
  Md[42][97] = Cd[0][7]; Md[42][98] = Cd[0][11]; Md[42][99] = Cd[0][13]; Md[42][100] = Cd[0][16];
  
  Md[43][15] = Cd[0][19]; Md[43][20] = Cd[0][9]; Md[43][22] = Cd[0][18]; Md[43][23] = Cd[0][3];
  Md[43][25] = Cd[0][8]; Md[43][26] = Cd[0][2]; Md[43][43] = Cd[0][0]; Md[43][57] = Cd[0][15];
  Md[43][60] = Cd[0][6]; Md[43][62] = Cd[0][12]; Md[43][63] = Cd[0][14]; Md[43][66] = Cd[0][1];
  Md[43][68] = Cd[0][4]; Md[43][69] = Cd[0][5]; Md[43][71] = Cd[0][10]; Md[43][72] = Cd[0][11];
  Md[43][92] = Cd[0][17]; Md[43][98] = Cd[0][7]; Md[43][101] = Cd[0][13]; Md[43][102] = Cd[0][16];
  
  Md[44][16] = Cd[0][19]; Md[44][21] = Cd[0][9]; Md[44][22] = Cd[0][15]; Md[44][24] = Cd[0][3];
  Md[44][25] = Cd[0][6]; Md[44][26] = Cd[0][1]; Md[44][44] = Cd[0][0]; Md[44][63] = Cd[0][12];
  Md[44][69] = Cd[0][4]; Md[44][72] = Cd[0][10]; Md[44][87] = Cd[0][18]; Md[44][90] = Cd[0][8];
  Md[44][92] = Cd[0][14]; Md[44][93] = Cd[0][17]; Md[44][96] = Cd[0][2]; Md[44][98] = Cd[0][5];
  Md[44][99] = Cd[0][7]; Md[44][101] = Cd[0][11]; Md[44][102] = Cd[0][13]; Md[44][103] = Cd[0][16];
  Md[45][28] = Cd[0][19]; Md[45][31] = Cd[0][9]; Md[45][32] = Cd[0][15]; Md[45][33] = Cd[0][18];
  Md[45][37] = Cd[0][3]; Md[45][38] = Cd[0][6]; Md[45][39] = Cd[0][8]; Md[45][40] = Cd[0][12];
  Md[45][41] = Cd[0][14]; Md[45][42] = Cd[0][17]; Md[45][45] = Cd[0][0]; Md[45][46] = Cd[0][1];
  Md[45][47] = Cd[0][2]; Md[45][48] = Cd[0][4]; Md[45][49] = Cd[0][5]; Md[45][50] = Cd[0][7];
  Md[45][51] = Cd[0][11]; Md[45][52] = Cd[0][13]; Md[45][73] = Cd[0][10]; Md[45][105] = Cd[0][16];
  Md[46][29] = Cd[0][19]; Md[46][32] = Cd[0][9]; Md[46][34] = Cd[0][15]; Md[46][35] = Cd[0][18];
  Md[46][38] = Cd[0][3]; Md[46][40] = Cd[0][6]; Md[46][41] = Cd[0][8]; Md[46][43] = Cd[0][14];
  Md[46][44] = Cd[0][17]; Md[46][46] = Cd[0][0]; Md[46][48] = Cd[0][1]; Md[46][49] = Cd[0][2];
  Md[46][51] = Cd[0][5]; Md[46][52] = Cd[0][7]; Md[46][53] = Cd[0][13]; Md[46][64] = Cd[0][12];
  Md[46][73] = Cd[0][4]; Md[46][74] = Cd[0][10]; Md[46][75] = Cd[0][11]; Md[46][106] = Cd[0][16];
  Md[47][30] = Cd[0][19]; Md[47][33] = Cd[0][9]; Md[47][35] = Cd[0][15]; Md[47][36] = Cd[0][18];
  Md[47][39] = Cd[0][3]; Md[47][41] = Cd[0][6]; Md[47][42] = Cd[0][8]; Md[47][43] = Cd[0][12];
  Md[47][44] = Cd[0][14]; Md[47][47] = Cd[0][0]; Md[47][49] = Cd[0][1]; Md[47][50] = Cd[0][2];
  Md[47][51] = Cd[0][4]; Md[47][52] = Cd[0][5]; Md[47][53] = Cd[0][11]; Md[47][75] = Cd[0][10];
  Md[47][95] = Cd[0][17]; Md[47][105] = Cd[0][7]; Md[47][106] = Cd[0][13]; Md[47][107] = Cd[0][16];
  Md[48][17] = Cd[0][19]; Md[48][23] = Cd[0][18]; Md[48][26] = Cd[0][17]; Md[48][34] = Cd[0][9];
  Md[48][40] = Cd[0][3]; Md[48][43] = Cd[0][8]; Md[48][48] = Cd[0][0]; Md[48][51] = Cd[0][2];
  Md[48][53] = Cd[0][7]; Md[48][58] = Cd[0][15]; Md[48][64] = Cd[0][6]; Md[48][65] = Cd[0][12];
  Md[48][66] = Cd[0][14]; Md[48][73] = Cd[0][1]; Md[48][74] = Cd[0][4]; Md[48][75] = Cd[0][5];
  Md[48][76] = Cd[0][10]; Md[48][77] = Cd[0][11]; Md[48][78] = Cd[0][13]; Md[48][108] = Cd[0][16];
  Md[49][18] = Cd[0][19]; Md[49][23] = Cd[0][15]; Md[49][24] = Cd[0][18]; Md[49][26] = Cd[0][14];
  Md[49][35] = Cd[0][9]; Md[49][41] = Cd[0][3]; Md[49][43] = Cd[0][6]; Md[49][44] = Cd[0][8];
  Md[49][49] = Cd[0][0]; Md[49][51] = Cd[0][1]; Md[49][52] = Cd[0][2]; Md[49][53] = Cd[0][5];
  Md[49][66] = Cd[0][12]; Md[49][75] = Cd[0][4]; Md[49][77] = Cd[0][10]; Md[49][78] = Cd[0][11];
  Md[49][96] = Cd[0][17]; Md[49][106] = Cd[0][7]; Md[49][108] = Cd[0][13]; Md[49][109] = Cd[0][16];
  Md[50][19] = Cd[0][19]; Md[50][24] = Cd[0][15]; Md[50][26] = Cd[0][12]; Md[50][36] = Cd[0][9];
  Md[50][42] = Cd[0][3]; Md[50][44] = Cd[0][6]; Md[50][50] = Cd[0][0]; Md[50][52] = Cd[0][1];
  Md[50][53] = Cd[0][4]; Md[50][78] = Cd[0][10]; Md[50][89] = Cd[0][18]; Md[50][95] = Cd[0][8];
  Md[50][96] = Cd[0][14]; Md[50][97] = Cd[0][17]; Md[50][105] = Cd[0][2]; Md[50][106] = Cd[0][5];
  Md[50][107] = Cd[0][7]; Md[50][108] = Cd[0][11];
  Md[50][109] = Cd[0][13]; Md[50][110] = Cd[0][16];
  Md[51][20] = Cd[0][19]; Md[51][23] = Cd[0][9];
  Md[51][25] = Cd[0][18]; Md[51][26] = Cd[0][8];
  Md[51][43] = Cd[0][3]; Md[51][51] = Cd[0][0];
  Md[51][53] = Cd[0][2]; Md[51][60] = Cd[0][15];
  Md[51][66] = Cd[0][6]; Md[51][68] = Cd[0][12];
  Md[51][69] = Cd[0][14]; Md[51][75] = Cd[0][1];
  Md[51][77] = Cd[0][4]; Md[51][78] = Cd[0][5];
  Md[51][80] = Cd[0][10]; Md[51][81] = Cd[0][11];
  Md[51][98] = Cd[0][17]; Md[51][108] = Cd[0][7];
  Md[51][111] = Cd[0][13]; Md[51][112] = Cd[0][16];
  Md[52][21] = Cd[0][19]; Md[52][24] = Cd[0][9];
  Md[52][25] = Cd[0][15]; Md[52][26] = Cd[0][6];
  Md[52][44] = Cd[0][3]; Md[52][52] = Cd[0][0];
  Md[52][53] = Cd[0][1]; Md[52][69] = Cd[0][12];
  Md[52][78] = Cd[0][4]; Md[52][81] = Cd[0][10];
  Md[52][90] = Cd[0][18]; Md[52][96] = Cd[0][8];
  Md[52][98] = Cd[0][14]; Md[52][99] = Cd[0][17];
  Md[52][106] = Cd[0][2]; Md[52][108] = Cd[0][5];
  Md[52][109] = Cd[0][7]; Md[52][111] = Cd[0][11];
  Md[52][112] = Cd[0][13]; Md[52][113] = Cd[0][16];
  Md[53][22] = Cd[0][19]; Md[53][25] = Cd[0][9];
  Md[53][26] = Cd[0][3]; Md[53][53] = Cd[0][0];
  Md[53][63] = Cd[0][15]; Md[53][69] = Cd[0][6];
  Md[53][72] = Cd[0][12]; Md[53][78] = Cd[0][1];
  Md[53][81] = Cd[0][4]; Md[53][84] = Cd[0][10];
  Md[53][92] = Cd[0][18]; Md[53][98] = Cd[0][8];
  Md[53][101] = Cd[0][14]; Md[53][102] = Cd[0][17];
  Md[53][108] = Cd[0][2]; Md[53][111] = Cd[0][5];
  Md[53][112] = Cd[0][7]; Md[53][115] = Cd[0][11];
  Md[53][116] = Cd[0][13]; Md[53][117] = Cd[0][16];
  Md[54][0] = Cd[1][19]; Md[54][1] = Cd[1][9];
  Md[54][2] = Cd[1][15]; Md[54][3] = Cd[1][18];
  Md[54][4] = Cd[1][3]; Md[54][5] = Cd[1][6];
  Md[54][6] = Cd[1][8]; Md[54][7] = Cd[1][12];
  Md[54][8] = Cd[1][14]; Md[54][9] = Cd[1][17];
  Md[54][10] = Cd[1][1]; Md[54][11] = Cd[1][2];
  Md[54][12] = Cd[1][4]; Md[54][13] = Cd[1][5];
  Md[54][14] = Cd[1][7]; Md[54][15] = Cd[1][11];
  Md[54][16] = Cd[1][13]; Md[54][27] = Cd[1][0];
  Md[54][54] = Cd[1][10]; Md[54][85] = Cd[1][16];
  Md[55][1] = Cd[1][19]; Md[55][4] = Cd[1][9];
  Md[55][5] = Cd[1][15]; Md[55][6] = Cd[1][18];
  Md[55][10] = Cd[1][6]; Md[55][11] = Cd[1][8];
  Md[55][12] = Cd[1][12]; Md[55][13] = Cd[1][14];
  Md[55][14] = Cd[1][17]; Md[55][17] = Cd[1][4];
  Md[55][18] = Cd[1][5]; Md[55][19] = Cd[1][7];
  Md[55][20] = Cd[1][11]; Md[55][21] = Cd[1][13];
  Md[55][27] = Cd[1][3]; Md[55][28] = Cd[1][0];
  Md[55][29] = Cd[1][1]; Md[55][30] = Cd[1][2];
  Md[55][55] = Cd[1][10]; Md[55][86] = Cd[1][16];
  Md[56][2] = Cd[1][19]; Md[56][5] = Cd[1][9];
  Md[56][7] = Cd[1][15]; Md[56][8] = Cd[1][18];
  Md[56][10] = Cd[1][3]; Md[56][12] = Cd[1][6];
  Md[56][13] = Cd[1][8]; Md[56][15] = Cd[1][14];
  Md[56][16] = Cd[1][17]; Md[56][17] = Cd[1][1];
  Md[56][18] = Cd[1][2]; Md[56][20] = Cd[1][5];
  Md[56][21] = Cd[1][7]; Md[56][22] = Cd[1][13];
  Md[56][29] = Cd[1][0]; Md[56][54] = Cd[1][12];
  Md[56][55] = Cd[1][4]; Md[56][56] = Cd[1][10];
  Md[56][57] = Cd[1][11]; Md[56][87] = Cd[1][16];
  Md[57][3] = Cd[1][19]; Md[57][6] = Cd[1][9];
  Md[57][8] = Cd[1][15]; Md[57][9] = Cd[1][18];
  Md[57][11] = Cd[1][3]; Md[57][13] = Cd[1][6];
  Md[57][14] = Cd[1][8]; Md[57][15] = Cd[1][12];
  Md[57][16] = Cd[1][14]; Md[57][18] = Cd[1][1];
  Md[57][19] = Cd[1][2]; Md[57][20] = Cd[1][4];
  Md[57][21] = Cd[1][5]; Md[57][22] = Cd[1][11];
  Md[57][30] = Cd[1][0]; Md[57][57] = Cd[1][10];
  Md[57][85] = Cd[1][17]; Md[57][86] = Cd[1][7];
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

Rox_ErrorCode rox_extract_real_gibbs_solutions_macaulay_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double M)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Define timer to measure performances
   // Rox_Timer timer = NULL;
   // Rox_Double time = 0.0;

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

   // Init new timer
   // error = rox_timer_new(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

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

   // rox_timer_start(timer);

   //error = rox_array2d_double_svdinverse(Di, D);
   //ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_inverse_lu(Di, D);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);
   // rox_log("mean time to solve the problem = %f (ms)\n", time);

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

Rox_ErrorCode rox_extract_real_quaternion_solutions_macaulay_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double M)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   // Define timer to measure performances
   // Rox_Timer timer = NULL;
   // Rox_Double time = 0.0;
   Rox_Sint new_solution = 0;

   Rox_Array2D_Double Q = NULL;

   Rox_Array2D_Double A = NULL;
   Rox_Array2D_Double B = NULL;
   Rox_Array2D_Double C = NULL;
   Rox_Array2D_Double D = NULL;
   Rox_Array2D_Double E = NULL;
   Rox_Array2D_Double Di = NULL;

   Rox_Array2D_Double rek = NULL;
   Rox_MatSO3 Rek = NULL;

   // Compute the n real eigenvalues d and eigenvectors V
   // of the Q matrix of size m x m
   Rox_ObjSet_Array2D_Double V = NULL; // size V will be m x n
   Rox_DynVec_Double e = NULL; // size e will be 1 x n
   Rox_ObjSet_Array2D_Double re = NULL; // size s will be 4 x n

   // Init new timer
   // error = rox_timer_new(&timer);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   if (!Re || !M ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Extract submatrices

   error = rox_array2d_double_new(&Q , 54, 54);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&Di, 441, 441);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&E , 54, 441);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &A, M,  0,  0, 54, 54 );            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &B, M,  0, 54, 54, 441 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &C, M, 54,  0, 441, 54 );            
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d( &D, M, 54, 54, 441, 441 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // error = rox_array2d_double_save("D.txt", D);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   // rox_timer_start(timer);

   //error = rox_array2d_double_svdinverse(Di, D);
   //ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_inverse_lu(Di, D);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Display elapsed time
   // rox_timer_stop(timer);
   // rox_timer_get_elapsed_ms(&time, timer);
   // rox_log("mean time to invert D = %f (ms)\n", time);

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

   error = rox_objset_array2d_double_new(&re, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //rox_timer_start(timer);

   error = rox_real_eigenvalues_eigenvectors(e, V, Q);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Display elapsed time
   //rox_timer_stop(timer);
   //rox_timer_get_elapsed_ms(&time, timer);
   //rox_log("mean time to compute eigenvectors = %f (ms)\n", time);

   ////////////////////////////////////////////////////////////////

   Rox_Uint nb_used = 0;
   error = rox_dynvec_double_get_used(&nb_used, e);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Array2D_Double * Vd = NULL;
   error = rox_objset_array2d_double_get_data_pointer ( &Vd, V );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k=0; k <nb_used; k++)
   {
      Rox_Double v0 = 0;
      Rox_Double v1 = 0;
      Rox_Double v2 = 0;
      Rox_Double v3 = 0;
      Rox_Double v4 = 0;

      Rox_Double wn = 0;
      Rox_Double xn = 0;
      Rox_Double yn = 0;
      Rox_Double zn = 0;

      // rox_matrix_print(Vd[k]);

      error = rox_array2d_double_get_value(&v0, Vd[k], 0, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&v1, Vd[k], 1, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&v2, Vd[k], 2, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&v3, Vd[k], 3, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_get_value(&v4, Vd[k], 4, 0);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      if (fabs(v0) > 10e-8)
      {
         wn = v1/v0;
      }
      else
      {
         wn = 0;
      }

      if (fabs(v0) > 10e-8)
      {
         xn = v2/v0;
      }
      else
      {
         xn = 0;
      }

      if (fabs(v0) > 10e-8)
      {
         yn = v3/v0;
      }
      else
      {
         yn = 0;
      }

      if (fabs(v0) > 10e-8)
      {
         zn = v4/v0;
      }
      else
      {
         zn = 0;
      }

      error = rox_array2d_double_new(&rek, 4, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_set_value(rek, 0, 0, wn);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_set_value(rek, 1, 0, xn);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_set_value(rek, 2, 0, yn);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_set_value(rek, 3, 0, zn);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // test if rek is already in re (twofold or duplicate solutions)
      error = rox_check_twofold_solutions (&new_solution, re, rek); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (new_solution)
      {
         error = rox_objset_array2d_double_append(re, rek);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matso3_new(&Rek);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_matso3_from_vector_quaternion(Rek, rek);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_double_append(Re, Rek);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
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

   rox_array2d_double_del(&rek);

   return error;
}


Rox_ErrorCode rox_check_twofold_solutions ( Rox_Sint * new_solution, Rox_ObjSet_Array2D_Double re, Rox_Array2D_Double rek)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nb_used = 0;

   Rox_Array2D_Double e = NULL;

   if (!re || !rek ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&e, 4, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_array2d_double_get_used(&nb_used, re);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Array2D_Double * red = NULL;
   error = rox_objset_array2d_double_get_data_pointer ( &red, re );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *new_solution = 1;

   for (Rox_Uint k=0; k <nb_used; k++)
   {
      Rox_Double norm2 = 1.0;

      error = rox_array2d_double_add(e, red[k], rek); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_norm2 ( &norm2, e );
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (norm2 < FLT_EPSILON)
      {
         *new_solution = 0;
         break;
      }
   }

function_terminate:

   return error;
}


Rox_ErrorCode rox_compute_valid_solutions_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Te, Rox_DynVec_Double ce, const Rox_ObjSet_Array2D_Double Re, const Rox_Array2D_Double At, const Rox_Array2D_Double bt, const Rox_Array2D_Double Ar, const Rox_Array2D_Double br, const Rox_Array2D_Double cr, const Rox_Plane3D_Double p, const Rox_Point3D_Double m, const Rox_Uint nbp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double vR = NULL;
   Rox_Array2D_Double R = NULL;
   Rox_Array2D_Double t = NULL;
   Rox_Array2D_Double g = NULL;

   Rox_Uint max_real_solutions = 0;
   Rox_Uint good_solution = 0;
   Rox_Uint count_sols = 0;

   if (!Te || !ce || !Re || !Ar || !br || !cr || !p || !m) 
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

      // Compute translation t = At * vR + bt;
      error = rox_array2d_double_mulmatmat(t, At, vR);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_add(t, t, bt);
      ROX_ERROR_CHECK_TERMINATE ( error );

      good_solution = 1;
      // Test of Hessian not needed, just take the min of cost
      //for (Rox_Uint i = 0; i < nbp; i++)
      //{
         // The Hessian ? must be definite positive
         // if (Hc[i] > 0.0)
         //{
         //   good_solution = 1;
         //   break;
         //}
      //}

      if (good_solution == 1)
      {
         Rox_Double cost = 0;
         Rox_MatSE3 T = NULL;

         error = rox_matse3_new(&T);
         ROX_ERROR_CHECK_TERMINATE ( error );

         //T(:,:,count_sols) = [R, t; 0, 0, 0, 1];
         error = rox_matse3_set_matso3_r3(T, R, t);
         ROX_ERROR_CHECK_TERMINATE ( error );

         count_sols = count_sols + 1;

         // compute cost final_solutions
         error = rox_cost_function_points3d_to_planes3d(&cost, T, p, m, nbp);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_objset_array2d_double_append(Te, T);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_dynvec_double_append(ce, &cost);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

   error = rox_sort_solutions_ascending_cost_points3d_to_planes3d(Te, ce);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matrix_del(&vR);
   rox_matrix_del(&g);
   rox_matrix_del(&t);
   rox_matrix_del(&R);

   return error;
}


Rox_ErrorCode rox_sort_solutions_ascending_cost_points3d_to_planes3d(Rox_ObjSet_Array2D_Double Te, Rox_DynVec_Double ce)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!Te || !ce) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   /////////////////////////////////////////////////////////////////////////////////
   // Sort in ascending order
   Rox_Array2D_Double cost_ascending = NULL;

   Rox_ObjSet_Array2D_Double Te_sort = NULL;

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

   for (Rox_Uint k=0; k<nsol; k++)
   {
      for (Rox_Uint i=0; i<nsol; i++)
      {
         if ((ce_data[i]>0) && (fabs(ce_data[i]-cost_ascending_data[0][k]) < 1e-15))
         {
            Rox_Array2D_Double Te_copy = NULL;

            error = rox_array2d_double_new_copy(&Te_copy, Te->data[i]);
            ROX_ERROR_CHECK_TERMINATE ( error );

            error = rox_objset_array2d_double_append(Te_sort, Te_copy);
            ROX_ERROR_CHECK_TERMINATE ( error );

            // Set the current cost to a negative value
            ce_data[i] = -1;
         }
      }
   }

   for (Rox_Sint k=0; k<nsol; k++)
   {
      error = rox_array2d_double_copy(Te->data[k], Te_sort->data[k]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      ce_data[k] = cost_ascending_data[0][k];
   }

   // error = rox_objset_array2d_double_clone(Te, Te_sort);
   // ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_objset_array2d_double_del(&Te_sort);
   rox_array2d_double_del(&cost_ascending);
   return error;
}

Rox_ErrorCode rox_cost_function_points3d_to_planes3d(Rox_Double * cost, Rox_MatSE3 T, Rox_Plane3D_Double p, const Rox_Point3D_Double m, const Rox_Uint nbp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cost || !T || !p || !m ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // R = T(1:3,1:3);
   // t = T(1:3,4:4);
   Rox_Double ** Td = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &Td, T);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *cost = 0.0;
   for (Rox_Uint k = 0; k < nbp; k++)
   {

      Rox_Double nx = p[k].a;
      Rox_Double ny = p[k].b;
      Rox_Double nz = p[k].c;
      Rox_Double  d = p[k].d;

      Rox_Double Xm = m[k].X;
      Rox_Double Ym = m[k].Y;
      Rox_Double Zm = m[k].Z;

      Rox_Double Xc = Td[0][0]*Xm+Td[0][1]*Ym+Td[0][2]*Zm+Td[0][3];
      Rox_Double Yc = Td[1][0]*Xm+Td[1][1]*Ym+Td[1][2]*Zm+Td[1][3];
      Rox_Double Zc = Td[2][0]*Xm+Td[2][1]*Ym+Td[2][2]*Zm+Td[2][3];

      Rox_Double distance = nx*Xc+ny*Yc+nz*Zc+d;

      // *cost += ( n(:,k)' * (R * m(1:3,k) + t) + d )^2;
      *cost += distance*distance;
   }

function_terminate:

   return error;
}
