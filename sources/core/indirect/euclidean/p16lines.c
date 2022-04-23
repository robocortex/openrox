//==============================================================================
//
//    OPENROX   : File p16lines.c
//
//    Contents  : Implementation of p16lines module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "p16lines.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/determinant/detgl3.h>
#include <baseproc/array/decomposition/qr.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/add/add.h>
#include <baseproc/maths/nonlin/polynomials.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/maths_macros.h>

Rox_ErrorCode rox_pose_from_16_lines(Rox_Array2D_Double_Collection poses, Rox_Uint * validposes, Rox_DynVec_Line3D_Plucker reflines, Rox_DynVec_Line3D_Plucker curlines)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double U = NULL, S = NULL, V = NULL, Q = NULL, R = NULL, P = NULL;
   Rox_Array2D_Double PU = NULL, PS = NULL, PV = NULL, TU = NULL, TS = NULL, TV = NULL;
   Rox_Array2D_Double buf53_1 = NULL, buf53_2 = NULL, buf33_1 = NULL;
   Rox_Array2D_Double A_trans = NULL, Ainv_trans = NULL, b_trans = NULL;
   Rox_Array2D_Double C = NULL, C2 = NULL, S0_beta = NULL, S1_beta = NULL, S2_beta = NULL;
   Rox_Array2D_Double R_algsol1 = NULL, R_algsol2 = NULL, R_sol1 = NULL, R_sol2 = NULL, M_poly = NULL;
   Rox_Double ** dc, **dr1, **dr2, **dv, **dp, **ds0, **ds1, **ds2, *ds, *db, **da, **dr, **dt;
   Rox_Uint idline, idcol, i,j,pos, countvalids, countinvalids, isvalidset;
   Rox_Double u[9], v[9], w[9];
   Rox_Double polys[5][3];
   Rox_Double * x, *y;
   Rox_Double normeq1, normeq2, normeq3, normeq4, normeq5, det, scale, n1, n2, norm;
   Rox_Double coefs[3];
   Rox_Complex roots[2];
   Rox_Array2D_Double R_est[2] = {NULL, NULL}, T_est[2] = {NULL, NULL};

   //Check input
   if (!poses || !validposes || !reflines || !curlines) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (reflines->used != curlines->used) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   if (reflines->used < 16) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   if (rox_array2d_double_collection_get_count(poses) != 2) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   
   error = rox_array2d_double_check_size(rox_array2d_double_collection_get(poses,0), 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Initialize output
   *validposes = NULL;
   
   //Create buffers
   error = rox_array2d_double_new(&C, curlines->used, 18); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&C2, 18, 18); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Q, 18, 18); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&R, 18, 18); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&P, 18, 18); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&U, 18, 18); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&S, 18, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&V, 18, 18); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&PU, 5, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&PS, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&PV, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&TU, curlines->used, curlines->used); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&TS, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&TV, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&R_algsol1, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&R_algsol2, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&R_sol1, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&R_sol2, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&S0_beta, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&S1_beta, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&S2_beta, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&M_poly, 5, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&buf33_1, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&buf53_1, 5, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&buf53_2, 5, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&R_est[0], rox_array2d_double_collection_get(poses, 0), 0, 0, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&T_est[0], rox_array2d_double_collection_get(poses, 0), 0, 3, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&R_est[1], rox_array2d_double_collection_get(poses, 1), 0, 0, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new_subarray2d(&T_est[1], rox_array2d_double_collection_get(poses, 1), 0, 3, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&A_trans, curlines->used, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&Ainv_trans, 3, curlines->used); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&b_trans, curlines->used, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dc, C );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(rox_array2d_double_collection_get(poses, 0));
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillunit(rox_array2d_double_collection_get(poses, 1));
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Build coefficients matrix
   for (idline = 0; idline < curlines->used; idline++)
   {
      x = curlines->data[idline].moment;
      y = reflines->data[idline].displacement;

      u[0] = x[0] * y[0];
      u[1] = x[0] * y[1];
      u[2] = x[0] * y[2];
      u[3] = x[1] * y[0];
      u[4] = x[1] * y[1];
      u[5] = x[1] * y[2];
      u[6] = x[2] * y[0];
      u[7] = x[2] * y[1];
      u[8] = x[2] * y[2];

      x = curlines->data[idline].displacement;
      y = reflines->data[idline].moment;

      v[0] = x[0] * y[0];
      v[1] = x[0] * y[1];
      v[2] = x[0] * y[2];
      v[3] = x[1] * y[0];
      v[4] = x[1] * y[1];
      v[5] = x[1] * y[2];
      v[6] = x[2] * y[0];
      v[7] = x[2] * y[1];
      v[8] = x[2] * y[2];

      x = curlines->data[idline].displacement;
      y = reflines->data[idline].displacement;

      w[0] = x[0] * y[0];
      w[1] = x[0] * y[1];
      w[2] = x[0] * y[2];
      w[3] = x[1] * y[0];
      w[4] = x[1] * y[1];
      w[5] = x[1] * y[2];
      w[6] = x[2] * y[0];
      w[7] = x[2] * y[1];
      w[8] = x[2] * y[2];

      for (idcol = 0; idcol < 9; idcol++)
      {
         dc[idline][idcol] = u[idcol] + v[idcol];
         dc[idline][9 + idcol] = w[idcol];
      }
   }
   
   //Compute SVD decomposition
   rox_array2d_double_mulmattransmat(C2, C, C);
   
   error = rox_array2d_double_qrp(Q,R,P,C2);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_svd_jacobi(U,S,V,Q,R,P); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Extract two ALGEBRAIC solutions to rotation
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dr1, R_algsol1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dr2, R_algsol2 );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dv, U );
   ROX_ERROR_CHECK_TERMINATE ( error );

   pos = 0;
   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++)
      {
         dr1[i][j] = dv[pos][16];
         dr2[i][j] = dv[pos][17];
         pos++;
      }
   }

   //Because R is an orthonormal matrix : R'*R = I
   //R'*R = alpha^2 * (R1 + beta*R2)'*(R1 + beta*R2)
   //R'*R = alpha^2 * (R1'*R1 + beta*(R2'*R1+R1'*R2) + beta^2 * R2'*R2)
   //R'*R = alpha^2 * (S0 + beta*S1 + beta^2 * S2)
   
   error = rox_array2d_double_mulmattransmat(S0_beta, R_algsol1, R_algsol2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmattransmat(S2_beta, R_algsol2, R_algsol1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_add(S1_beta, S0_beta, S2_beta);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmattransmat(S0_beta, R_algsol1, R_algsol1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmattransmat(S2_beta, R_algsol2, R_algsol2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Build polynomial set for rotation solve
   error = rox_array2d_double_get_data_pointer_to_pointer( &dp, M_poly);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &ds0,S0_beta);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &ds1,S1_beta);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &ds2,S2_beta);
   ROX_ERROR_CHECK_TERMINATE ( error );

   polys[0][0] = ds0[0][0] - ds0[1][1];
   polys[0][1] = ds1[0][0] - ds1[1][1];
   polys[0][2] = ds2[0][0] - ds2[1][1];
   polys[1][0] = ds0[2][2] - ds0[1][1];
   polys[1][1] = ds1[2][2] - ds1[1][1];
   polys[1][2] = ds2[2][2] - ds2[1][1];
   polys[2][0] = ds0[0][1];
   polys[2][1] = ds1[0][1];
   polys[2][2] = ds2[0][1];
   polys[3][0] = ds0[0][2];
   polys[3][1] = ds1[0][2];
   polys[3][2] = ds2[0][2];
   polys[4][0] = ds0[1][2];
   polys[4][1] = ds1[1][2];
   polys[4][2] = ds2[1][2];

   //Compute the norm of each polynomial
   normeq1 = sqrt(polys[0][0]*polys[0][0] + polys[0][1]*polys[0][1] + polys[0][2]*polys[0][2]);
   normeq2 = sqrt(polys[1][0]*polys[1][0] + polys[1][1]*polys[1][1] + polys[1][2]*polys[1][2]);
   normeq3 = sqrt(polys[2][0]*polys[2][0] + polys[2][1]*polys[2][1] + polys[2][2]*polys[2][2]);
   normeq4 = sqrt(polys[3][0]*polys[3][0] + polys[3][1]*polys[3][1] + polys[3][2]*polys[3][2]);
   normeq5 = sqrt(polys[4][0]*polys[4][0] + polys[4][1]*polys[4][1] + polys[4][2]*polys[4][2]);
   
   //Count the number of possible invalid polynomials
   countinvalids = 0;
   if (normeq1 < 1e-2) countinvalids++;
   if (normeq2 < 1e-2) countinvalids++;
   if (normeq3 < 1e-2) countinvalids++;
   if (normeq4 < 1e-2) countinvalids++;
   if (normeq5 < 1e-2) countinvalids++;
   
   //If all polynomials are possibly invalid, let create a solution with R=I
   if (countinvalids == 5)
   {
      error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, T_est[0] );
      error = rox_array2d_double_fillunit(R_est[0]);
      
      // Estimate linearly the translation from the observations and the estimated translation
      error = rox_array2d_double_get_data_pointer_to_pointer( &dr, R_est[0]);
      error = rox_array2d_double_get_data_pointer_to_pointer( &da, A_trans);
      
      for (idline = 0; idline < curlines->used; idline++)
      {
         Rox_Double l1, l2, l3;
         
         x = curlines->data[idline].displacement;
         y = reflines->data[idline].displacement;
         
         l1 = dr[0][0] * y[0] + dr[0][1] * y[1] + dr[0][2] * y[2];
         l2 = dr[1][0] * y[0] + dr[1][1] * y[1] + dr[1][2] * y[2];
         l3 = dr[2][0] * y[0] + dr[2][1] * y[1] + dr[2][2] * y[2];
         
         da[idline][0] =  -(x[1] * l3 - x[2] * l2);
         da[idline][1] =  -(- x[0] * l3 + x[2] * l1);
         da[idline][2] =  -(x[0] * l2 - x[1] * l1);
      }
      
      //Translation is the null space unique vector
      error = rox_array2d_double_svd(TU, TS, TV, A_trans); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_svd_sort_SV(TS, TV); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_get_data_pointer_to_pointer(&dv, TV);
      ROX_ERROR_CHECK_TERMINATE ( error );

      dt[0][0] = dv[0][2];
      dt[1][0] = dv[1][2];
      dt[2][0] = dv[2][2];
      
      (*validposes)++;
   }
   
   norm = 1e-12;
   isvalidset = 0;
   
   //Try to get a valid set of normalized polynomials
   while (norm < 1e-2 && !isvalidset)
   {
      countvalids = 0;
      rox_array2d_double_fillval(M_poly, 0);
      
      if (normeq1 > norm)
      {
         dp[countvalids][0] = polys[0][0] / normeq1;
         dp[countvalids][1] = polys[0][1] / normeq1;
         dp[countvalids][2] = polys[0][2] / normeq1;
         countvalids++;
      }
      
      if (normeq2 > norm)
      {
         dp[countvalids][0] = polys[1][0] / normeq2;
         dp[countvalids][1] = polys[1][1] / normeq2;
         dp[countvalids][2] = polys[1][2] / normeq2;
         countvalids++;
      }
      
      if (normeq3 > norm)
      {
         dp[countvalids][0] = polys[2][0] / normeq3;
         dp[countvalids][1] = polys[2][1] / normeq3;
         dp[countvalids][2] = polys[2][2] / normeq3;
         countvalids++;
      }
      
      if (normeq4 > norm)
      {
         dp[countvalids][0] = polys[3][0] / normeq4;
         dp[countvalids][1] = polys[3][1] / normeq4;
         dp[countvalids][2] = polys[3][2] / normeq4;
         countvalids++;
      }
      
      if (normeq5 > norm)
      {
         dp[countvalids][0] = polys[4][0] / normeq5;
         dp[countvalids][1] = polys[4][1] / normeq5;
         dp[countvalids][2] = polys[4][2] / normeq5;
         countvalids++;
      }
      if (countvalids == 0) break;
      
      //decompose polynomials coefficients matrix
      error = rox_array2d_double_svd(PU,PS,PV,M_poly); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_svd_sort(PU, PS, PV); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_get_data_pointer(&ds, PS);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //Equalize polynomials
      rox_array2d_double_fillval(buf53_1, 0);
      rox_array2d_double_set_value(buf53_1, 0, 0, ds[0]);
      rox_array2d_double_mulmatmattrans(buf53_2, buf53_1, V);
      rox_array2d_double_mulmatmat(M_poly, U, buf53_2);
      
      isvalidset = 1;
      for (j = 0; j < 3; j++)
      {
         Rox_Double ref = dp[0][j];
         
         for (i = 1; i < countvalids; i++)
         {
            Rox_Double dif = fabs(fabs(ref) - fabs(dp[i][j]));
            if (dif > 1e-4) isvalidset = 0;
         }
      }
     
      norm = norm * 10.0;
   }

   if (!isvalidset)
   {
      error = ROX_ERROR_NONE;
      goto on_terminate;
   }
          
   //Retrieve roots
   coefs[0] = dp[0][0];
   coefs[1] = dp[0][1];
   coefs[2] = dp[0][2];

   error = rox_quadratic_roots(roots, coefs); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Create solutions
   rox_array2d_double_scale(R_sol1, R_algsol2, 1.0 / roots[0].real);
   rox_array2d_double_add(R_sol1, R_sol1, R_algsol1);
   rox_array2d_double_detgl3(&det, R_sol1);
   scale = 1.0 / (ROX_SIGN(det) * pow(fabs(det), 1.0/3.0));
   rox_array2d_double_scale(R_sol1, R_sol1, scale);

   rox_array2d_double_scale(R_sol2, R_algsol2, 1.0 / roots[1].real);
   rox_array2d_double_add(R_sol2, R_sol2, R_algsol1);
   rox_array2d_double_detgl3(&det, R_sol2);
   scale = 1.0 / (ROX_SIGN(det) * pow(fabs(det), 1.0/3.0));
   rox_array2d_double_scale(R_sol2, R_sol2, scale);

   //Choose best solution among R_sol1 and R_sol2
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dr1, R_sol1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &dr2, R_sol2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   n1 = fabs(dr1[0][0] + dr1[1][1] + dr1[2][2] - 3);
   n2 = fabs(dr2[0][0] + dr2[1][1] + dr2[2][2] - 3);

   if (n1 < n2)
   {
      rox_array2d_double_copy(R_est[*validposes], R_sol2);
   }
   else
   {
      rox_array2d_double_copy(R_est[*validposes], R_sol1);
   }
   
   //Estimate linearly the translation from the observations and the estimated translation
   error = rox_array2d_double_get_data_pointer_to_pointer ( &ds, R_est[*validposes]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer ( &da, A_trans);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer ( &db, b_trans);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (idline = 0; idline < curlines->used; idline++)
   {
      Rox_Double l1, l2, l3;
      
      x = curlines->data[idline].moment;
      y = reflines->data[idline].displacement;
      
      l1 = dr[0][0] * y[0] + dr[0][1] * y[1] + dr[0][2] * y[2];
      l2 = dr[1][0] * y[0] + dr[1][1] * y[1] + dr[1][2] * y[2];
      l3 = dr[2][0] * y[0] + dr[2][1] * y[1] + dr[2][2] * y[2];
      db[idline] = - (x[0] * l1 + x[1] * l2 + x[2] * l3);
      
      x = curlines->data[idline].displacement;
      y = reflines->data[idline].moment;
      
      l1 = dr[0][0] * y[0] + dr[0][1] * y[1] + dr[0][2] * y[2];
      l2 = dr[1][0] * y[0] + dr[1][1] * y[1] + dr[1][2] * y[2];
      l3 = dr[2][0] * y[0] + dr[2][1] * y[1] + dr[2][2] * y[2];
      db[idline] -= x[0] * l1 + x[1] * l2 + x[2] * l3;
      
      x = curlines->data[idline].displacement;
      y = reflines->data[idline].displacement;
      
      l1 = dr[0][0] * y[0] + dr[0][1] * y[1] + dr[0][2] * y[2];
      l2 = dr[1][0] * y[0] + dr[1][1] * y[1] + dr[1][2] * y[2];
      l3 = dr[2][0] * y[0] + dr[2][1] * y[1] + dr[2][2] * y[2];
      da[idline][0] =  - (x[1] * l3 - x[2] * l2);
      da[idline][1] =  - (- x[0] * l3 + x[2] * l1);
      da[idline][2] =  - (x[0] * l2 - x[1] * l1);
   }
   
   error = rox_array2d_double_svdinverse(Ainv_trans, A_trans); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(T_est[*validposes], Ainv_trans, b_trans); ROX_ERROR_CHECK_TERMINATE ( error );
   
   (*validposes)++;


function_terminate:
   //Delete buffers
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&Q);
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&P);
   rox_array2d_double_del(&U);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&PU);
   rox_array2d_double_del(&PS);
   rox_array2d_double_del(&PV);
   rox_array2d_double_del(&TU);
   rox_array2d_double_del(&TS);
   rox_array2d_double_del(&TV);
   rox_array2d_double_del(&R_algsol1);
   rox_array2d_double_del(&R_algsol2);
   rox_array2d_double_del(&R_sol1);
   rox_array2d_double_del(&R_sol2);
   rox_array2d_double_del(&S0_beta);
   rox_array2d_double_del(&S1_beta);
   rox_array2d_double_del(&S2_beta);
   rox_array2d_double_del(&M_poly);
   rox_array2d_double_del(&buf53_1);
   rox_array2d_double_del(&buf53_2);
   rox_array2d_double_del(&buf33_1);
   rox_array2d_double_del(&R_est[0]);
   rox_array2d_double_del(&T_est[0]);
   rox_array2d_double_del(&R_est[1]);
   rox_array2d_double_del(&T_est[1]);
   rox_array2d_double_del(&A_trans);
   rox_array2d_double_del(&Ainv_trans);
   rox_array2d_double_del(&b_trans);

   return error;
}
