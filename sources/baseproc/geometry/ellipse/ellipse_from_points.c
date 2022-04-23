//==============================================================================
//
//    OPENROX   : File ellipse_from_points.c
//
//    Contents  : Implementation of ellipse_from_points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ellipse_from_points.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point2d_float_struct.h>

#include <generated/dynvec_uint.h>
#include <generated/dynvec_uint_struct.h>

#include <baseproc/geometry/point/dynvec_point2d_tools.h>

#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/decomposition/svdsort.h>
#include <baseproc/array/solve/svd_solve.h>
#include <baseproc/array/fill/fillval.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/ellipse/ellipse2d_struct.h>
#include <baseproc/geometry/ellipse/ellipse_transform.h>
#include <baseproc/geometry/measures/distance_point_to_line.h>
#include <baseproc/geometry/point/point2d_tools.h>
#include <baseproc/maths/random/combination.h>
#include <baseproc/maths/random/combination_struct.h>
#include <baseproc/geometry/line/line_from_points.h>

#include <inout/geometry/point/dynvec_point3d_print.h>
#include <inout/geometry/point/dynvec_point2d_print.h>

#include <inout/geometry/point/point2d_print.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_points_2d_normalization_matrix (
   Rox_Array2D_Double K_nor,
   const Rox_DynVec_Point2D_Double points2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint nbp = points2D->used;

   if (!K_nor || !points2D)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );
   }
   if (nbp == 0)
   {
      error = ROX_ERROR_INSUFFICIENT_DATA; ROX_ERROR_CHECK_TERMINATE ( error );
   }
   error = rox_array2d_double_check_size(K_nor, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //v = p(1, :). ^ 2 + p(2, :). ^ 2 + p(3, :). ^ 2;
   //exist_p = p(:, v > 0);
   //mean_p = mean(exist_p(1:2, : )')';
   Rox_Double total_x = 0;
   Rox_Double total_y = 0;
   for (Rox_Uint i = 0; i < nbp; ++i)
   {
      Rox_Double x = points2D->data[i].u;
      Rox_Double y = points2D->data[i].v;
      total_x += x;
      total_y += y;
   }
   Rox_Double mean_x = total_x / nbp;
   Rox_Double mean_y = total_y / nbp;

   //% Normaliszation using a diagonal approximation of the covariance matrix
   //dist_p = sqrt((exist_p(1, :) - mean_p(1)). ^ 2 + (exist_p(2, :) - mean_p(2)). ^ 2);
   //meandist_p = mean(dist_p(:));
   //scale = sqrt(2) / meandist_p;
   //invcov_p = scale*eye(2, 2);
   Rox_Double total_dist = 0;
   for (Rox_Uint i = 0; i < nbp; ++i)
   {
      Rox_Double x = points2D->data[i].u;
      Rox_Double y = points2D->data[i].v;
      Rox_Double d = (x - mean_x)*(x - mean_x) + (y - mean_y)*(y - mean_y);
      if (d < DBL_EPSILON)
      {
         error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
      }
      total_dist += sqrt(d);
   }
   Rox_Double mean_dist = total_dist / nbp;
   if (fabs(mean_dist) < DBL_EPSILON)
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }
   Rox_Double scale = sqrt(2) / mean_dist;

   //Kn = [invcov_p, -invcov_p*mean_p
   //   0     0           1];
   Rox_Double ** K_nor_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &K_nor_data, K_nor );
   ROX_ERROR_CHECK_TERMINATE ( error );

   K_nor_data[0][0] = scale;
   K_nor_data[0][1] = 0;
   K_nor_data[0][2] = -mean_x*scale;
   K_nor_data[1][0] = 0;
   K_nor_data[1][1] = scale;
   K_nor_data[1][2] = -mean_y*scale;
   K_nor_data[2][0] = 0;
   K_nor_data[2][1] = 0;
   K_nor_data[2][2] = 1;

   //pn = Kn * p;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ellipse2d_parametric_from_n_point2d(
   Rox_Ellipse2D_Parametric ellipse2d_parametric,
   const Rox_DynVec_Point2D_Double points2D)
{
   // Closed form solution of least squares problem
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double K_nor = NULL;
   Rox_Array2D_Double C = NULL;

   Rox_Array2D_Double U = NULL;
   Rox_Array2D_Double S = NULL;
   Rox_Array2D_Double V = NULL;

   Rox_Array2D_Double A = NULL;
   Rox_Array2D_Double b = NULL;
   Rox_Array2D_Double pc = NULL;
   Rox_Uint nbp = points2D->used;
   Rox_DynVec_Point2D_Double points2d_nor = NULL;
   Rox_Ellipse2D_Parametric_Struct ellipse2d_parametric_nor;

   if (!ellipse2d_parametric || !points2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (nbp < 5)
   { error = ROX_ERROR_INSUFFICIENT_DATA; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&K_nor, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&C, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&A, 2, 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&b, 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&pc, 2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(C, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** C_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &C_data, C );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the normalized points
   // such that: points2d_nor = K_nor * points2d;
   error = rox_points_2d_normalization_matrix(K_nor, points2D);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_new(&points2d_nor, nbp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_transform_homogeneous(points2d_nor, points2D, K_nor);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute linear system C * s = 0
   for (Rox_Uint i = 0; i < nbp; ++i)
   {
      Rox_Double x = points2d_nor->data[i].u;
      Rox_Double y = points2d_nor->data[i].v;

      // C_data[i][0] =        x * x ;
      // C_data[i][1] =        y * y ;
      // C_data[i][2] =  2.0 * x * y ;
      // C_data[i][3] = -2.0 * x ;
      // C_data[i][4] = -2.0 * y ;
      // C_data[i][5] =  1.0;

      Rox_Double x2 = x*x; Rox_Double x3 = x2*x; Rox_Double x4 = x3*x;
      Rox_Double y2 = y*y; Rox_Double y3 = y2*y; Rox_Double y4 = y3*y;

      C_data[0][0] += x4;           C_data[0][1] += x2*y2;     C_data[0][2] += 2.0*x3*y;   C_data[0][3] += -2.0*x3;     C_data[0][4] += -2.0*x2*y; C_data[0][5] += x2;
      C_data[1][0] += x2*y2;        C_data[1][1] += y4;        C_data[1][2] += 2.0*x*y3;   C_data[1][3] += -2.0*x*y2;   C_data[1][4] += -2.0*y3;   C_data[1][5] += y2;
      C_data[2][0] += 2.0*x3*y;     C_data[2][1] += 2.0*x*y3;  C_data[2][2] += 4.0*x2*y2;  C_data[2][3] += -4.0*x2*y;   C_data[2][4] += -4.0*x*y2; C_data[2][5] += 2.0*x*y;
      C_data[3][0] += -2.0*x3;      C_data[3][1] += -2.0*x*y2; C_data[3][2] += -4.0*x2*y;  C_data[3][3] += 4.0*x2;      C_data[3][4] += 4.0*x*y;   C_data[3][5] += -2.0*x;
      C_data[4][0] += -2.0*x2*y;    C_data[4][1] += -2.0*y3;   C_data[4][2] += -4.0*x*y2;  C_data[4][3] += 4.0*x*y;     C_data[4][4] += 4.0*y2;    C_data[4][5] += -2.0*y;
      C_data[5][0] += x2;           C_data[5][1] += y2;        C_data[5][2] += 2.0*x*y;    C_data[5][3] += -2.0*x;      C_data[5][4] += -2.0*y;    C_data[5][5] += 1.0;

// [      x^4,  x^2*y^2,   2*x^3*y,   -2*x^3, -2*x^2*y,   x^2]
// [  x^2*y^2,      y^4,   2*x*y^3, -2*x*y^2,   -2*y^3,   y^2]
// [  2*x^3*y,  2*x*y^3, 4*x^2*y^2, -4*x^2*y, -4*x*y^2, 2*x*y]
// [   -2*x^3, -2*x*y^2,  -4*x^2*y,    4*x^2,    4*x*y,  -2*x]
// [ -2*x^2*y,   -2*y^3,  -4*x*y^2,    4*x*y,    4*y^2,  -2*y]
// [      x^2,      y^2,     2*x*y,     -2*x,     -2*y,     1]

   }

   // solution is from [U,S,V] = svd(A)
   // k * s = V(:,6)
   error = rox_array2d_double_new(&U, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&S, 6, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&V, 6, 6);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** V_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &V_data, V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svd(U, S, V, C);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_svd_sort_SV(S, V);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** A_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &A_data, A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** b_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &b_data, b);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We can find xc and yc without knowing lambda
   A_data[0][0] = V_data[0][5];
   A_data[1][1] = V_data[1][5];
   A_data[1][0] = V_data[2][5];
   A_data[0][1] = V_data[2][5];

   b_data[0][0] = V_data[3][5];
   b_data[1][0] = V_data[4][5];

   // pc = inv(Q)*[sol(4:5,1)]
   Rox_Double ** pc_data = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &pc_data, pc );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_svd_solve(pc, A, b);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ellipse2d_parametric_nor.xc = pc_data[0][0];
   ellipse2d_parametric_nor.yc = pc_data[1][0];

   Rox_Double lambda = V_data[0][5]* ellipse2d_parametric_nor.xc*ellipse2d_parametric_nor.xc +
                     V_data[1][5]* ellipse2d_parametric_nor.yc*ellipse2d_parametric_nor.yc +
                   2*V_data[2][5]* ellipse2d_parametric_nor.xc*ellipse2d_parametric_nor.yc - V_data[5][5];

   ellipse2d_parametric_nor.nxx = V_data[0][5]/lambda;
   ellipse2d_parametric_nor.nyy = V_data[1][5]/lambda;
   ellipse2d_parametric_nor.nxy = V_data[2][5]/lambda;

   // Test if the matrix [nxx, nxy; nxy nyy] is definite positive with the Sylvester criteria
   if ((ellipse2d_parametric_nor.nxx < 0) || (ellipse2d_parametric_nor.nxx * ellipse2d_parametric_nor.nyy < ellipse2d_parametric_nor.nxy * ellipse2d_parametric_nor.nxy))
   {
      error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // points2d = inv(K_nor) * points2d_nor;
   // De-normalize the result
   error = rox_ellipse2d_transform_pixels_to_meters(ellipse2d_parametric, K_nor, &ellipse2d_parametric_nor);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:

   rox_array2d_double_del(&U);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&A);
   rox_array2d_double_del(&b);
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&pc);
   rox_array2d_double_del(&K_nor);
   rox_dynvec_point2d_double_del(&points2d_nor);

   return error;
}

Rox_ErrorCode rox_ellipse2d_separate_inliers_outliers (
   Rox_DynVec_Uint points2D_inliers_indexes,
   Rox_Float * mean_distance,
   const Rox_Ellipse2D_Parametric ellipse2d,
   const Rox_DynVec_Point2D_Float points2D,
   const Rox_Float distance_threshold)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Get the number of points in the dynamic vector
   Rox_Uint nbp = points2D->used;

   // Reset the dynamic vector
   rox_dynvec_uint_reset(points2D_inliers_indexes);

   // Init mean disatnce
   *mean_distance = 0.0f;
   for ( Rox_Uint k = 0; k < nbp; k++) // k must be of type Rox_Uint
   {
      Rox_Float distance = 0.0f;

      // error = rox_distance_point2d_to_ellipse2d(&distance, points2D->data[k], ellipse2d);
      // ROX_ERROR_CHECK_TERMINATE ( error );

      // If the distance is lower than distance_threshold we have an inlier
      if (distance < distance_threshold)
      {
         // Cumulate to compute mean distance
         *mean_distance += distance;

         // Add to indexes
         error = rox_dynvec_uint_append(points2D_inliers_indexes, &k);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

   if (points2D_inliers_indexes->used > 0)
   {
      *mean_distance = *mean_distance / points2D_inliers_indexes->used;
   }
   else
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
