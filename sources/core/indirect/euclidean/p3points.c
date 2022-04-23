//==============================================================================
//
//    OPENROX   : File p3points.c
//
//    Contents  : Implementation of p3points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "p3points.h"

#include <system/errors/errors.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/nonlin/polynomials.h>
#include <baseproc/array/decomposition/svd.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/determinant/detgl3.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_pose_from_two_3D_triangles (
   Rox_MatSE3 cMo,
   Rox_Point3D_Double pctriangle,
   Rox_Point3D_Double potriangle
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point3D_Double_Struct baryref, barycur;
   Rox_Array2D_Double C= NULL, U= NULL, S= NULL, V= NULL, W= NULL, T= NULL;
   Rox_Array2D_Double R= NULL;
   Rox_Double **dC, **dS, **dT, **dR;
   Rox_Double det;
   Rox_Double s1, s2, s3, mins;
   Rox_Uint posmin;

   Rox_Point3D_Double_Struct ctriangle[3];
   Rox_Point3D_Double_Struct otriangle[3];


   if (!cMo) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( cMo );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_fillunit(cMo);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new_subarray2d(&R, cMo, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_new_subarray2d(&T, cMo, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Compute barycenters for both sets
   barycur.X = (pctriangle[0].X + pctriangle[1].X + pctriangle[2].X) / 3.0;
   barycur.Y = (pctriangle[0].Y + pctriangle[1].Y + pctriangle[2].Y) / 3.0;
   barycur.Z = (pctriangle[0].Z + pctriangle[1].Z + pctriangle[2].Z) / 3.0;
   baryref.X = (potriangle[0].X + potriangle[1].X + potriangle[2].X) / 3.0;
   baryref.Y = (potriangle[0].Y + potriangle[1].Y + potriangle[2].Y) / 3.0;
   baryref.Z = (potriangle[0].Z + potriangle[1].Z + potriangle[2].Z) / 3.0;

   // Shift the sets on their barycenters
   for ( Rox_Sint id = 0; id < 3; id++)
   {
      ctriangle[id].X = pctriangle[id].X - barycur.X;
      ctriangle[id].Y = pctriangle[id].Y - barycur.Y;
      ctriangle[id].Z = pctriangle[id].Z - barycur.Z;
      otriangle[id].X = potriangle[id].X - baryref.X;
      otriangle[id].Y = potriangle[id].Y - baryref.Y;
      otriangle[id].Z = potriangle[id].Z - baryref.Z;
   }

   // Compute Orthogonal Procrustes problem solution
   error = rox_array2d_double_new(&C, 3, 3); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&U, 3, 3); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&S, 3, 1); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&V, 3, 3); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&W, 3, 3); ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dC, C); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dS, S); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dR, R); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer( &dT, T); ROX_ERROR_CHECK_TERMINATE ( error );

   dC[0][0] = ctriangle[0].X * otriangle[0].X + ctriangle[1].X * otriangle[1].X + ctriangle[2].X * otriangle[2].X;
   dC[0][1] = ctriangle[0].X * otriangle[0].Y + ctriangle[1].X * otriangle[1].Y + ctriangle[2].X * otriangle[2].Y;
   dC[0][2] = ctriangle[0].X * otriangle[0].Z + ctriangle[1].X * otriangle[1].Z + ctriangle[2].X * otriangle[2].Z;
   dC[1][0] = ctriangle[0].Y * otriangle[0].X + ctriangle[1].Y * otriangle[1].X + ctriangle[2].Y * otriangle[2].X;
   dC[1][1] = ctriangle[0].Y * otriangle[0].Y + ctriangle[1].Y * otriangle[1].Y + ctriangle[2].Y * otriangle[2].Y;
   dC[1][2] = ctriangle[0].Y * otriangle[0].Z + ctriangle[1].Y * otriangle[1].Z + ctriangle[2].Y * otriangle[2].Z;
   dC[2][0] = ctriangle[0].Z * otriangle[0].X + ctriangle[1].Z * otriangle[1].X + ctriangle[2].Z * otriangle[2].X;
   dC[2][1] = ctriangle[0].Z * otriangle[0].Y + ctriangle[1].Z * otriangle[1].Y + ctriangle[2].Z * otriangle[2].Y;
   dC[2][2] = ctriangle[0].Z * otriangle[0].Z + ctriangle[1].Z * otriangle[1].Z + ctriangle[2].Z * otriangle[2].Z;

   error = rox_array2d_double_svd(U, S, V, C);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Get determinant
   error = rox_array2d_double_mulmatmat(R, U, V);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_detgl3(&det, R);
   ROX_ERROR_CHECK_TERMINATE(error)

   // get minimal value
   s1 = dS[0][0];
   s2 = dS[1][0];
   s3 = dS[2][0];
   mins = s1;
   posmin = 0;
   if (s2 < mins)
   {
      mins = s2;
      posmin = 1;
   }
   if (s3 < mins)
   {
      posmin = 2;
   }

   // Compute Rotation
   rox_array2d_double_fillunit(W);
   rox_array2d_double_set_value(W, posmin, posmin, det);  // Make sure det(R) == 1
   rox_array2d_double_mulmatmattrans(C, W, V);
   rox_array2d_double_mulmatmat(R, U, C);

   // Compute translation
   dT[0][0] = barycur.X - (dR[0][0] * baryref.X + dR[0][1] * baryref.Y + dR[0][2] * baryref.Z);
   dT[1][0] = barycur.Y - (dR[1][0] * baryref.X + dR[1][1] * baryref.Y + dR[1][2] * baryref.Z);
   dT[2][0] = barycur.Z - (dR[2][0] * baryref.X + dR[2][1] * baryref.Y + dR[2][2] * baryref.Z);

function_terminate:
   rox_array2d_double_del(&R);
   rox_array2d_double_del(&T);
   rox_array2d_double_del(&C);
   rox_array2d_double_del(&U);
   rox_array2d_double_del(&S);
   rox_array2d_double_del(&V);
   rox_array2d_double_del(&W);

   return error;
}

Rox_ErrorCode rox_pose_from_3_points (
   Rox_Array2D_Double_Collection poses,
   Rox_Uint * validposes,
   Rox_Point3D_Double points3D,
   Rox_Point2D_Double points2D,
   double fx,
   double fy,
   double u0,
   double v0
) //, Rox_Array2D_Double calib);
{
   Rox_Uint id, countsol, idroot;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double curpose;
   Rox_Double ifx, ify, iu0, iv0;
   Rox_Point3D_Double_Struct mproj[3];
   Rox_Point3D_Double_Struct dir3D12, dir3D13, dir3D23;
   Rox_Double norme, inorme, x, y;
   Rox_Double cos12, cos13, cos23;
   Rox_Double ds12, ds13, ds23;
   Rox_Double ratio2313, ratio2312;
   Rox_Double quintic_coefficients[5];
   Rox_Complex_Struct quintic_roots[4];
   Rox_Point3D_Double_Struct possible_triangle[4][3];


   //  Check input 
   if (!poses)    
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (!points2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (!points3D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //  Check poses 
   if (rox_array2d_double_collection_get_count(poses) != 4) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   curpose = rox_array2d_double_collection_get(poses, 0);

   error = rox_array2d_double_check_size(curpose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE(error)

   //  First, we need to compute all cZ to get a 3D point cloud
   ifx = 1.0 / fx;
   ify = 1.0 / fy;
   iu0 = -u0 * ifx;
   iv0 = -v0 * ify;

   //  Get image points in meter space
   for (id = 0; id < 3; id++)
   {
      x = ifx * points2D[id].u + iu0;
      y = ify * points2D[id].v + iv0;

      norme = sqrt(x * x + y * y + 1.0);

      if (ROX_IS_ZERO_DOUBLE(norme)) 
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      inorme = 1.0 / norme;

      mproj[id].X = x * inorme;
      mproj[id].Y = y * inorme;
      mproj[id].Z = inorme;
   }

   // Global idea : a 3D transformation (SE(3)) keeps angles and distances between points.
   // We already know a 3D triangle in world frame (points3D).
   // Given our 2D Triangle in image space, search for a 3D Triangle in camera space which has the same property
   // than the given points3D triangle (angle and distances).
   // See Random Sample Consensus: A Paradigm for Model Fitting with Applicatlons to Image Analysis and Automated Cartography.

   // Get cos(angle) for 3 pairs (1;2 1;3 2;3) for 2D triangle
   cos12 = mproj[0].X * mproj[1].X + mproj[0].Y * mproj[1].Y + mproj[0].Z * mproj[1].Z;
   cos13 = mproj[0].X * mproj[2].X + mproj[0].Y * mproj[2].Y + mproj[0].Z * mproj[2].Z;
   cos23 = mproj[1].X * mproj[2].X + mproj[1].Y * mproj[2].Y + mproj[1].Z * mproj[2].Z;

   // Get 3D points distances squared for 3 pairs (1;2 1;3 2;3) for 3D triangle
   dir3D12.X = points3D[1].X - points3D[0].X;
   dir3D12.Y = points3D[1].Y - points3D[0].Y;
   dir3D12.Z = points3D[1].Z - points3D[0].Z;
   dir3D13.X = points3D[2].X - points3D[0].X;
   dir3D13.Y = points3D[2].Y - points3D[0].Y;
   dir3D13.Z = points3D[2].Z - points3D[0].Z;
   dir3D23.X = points3D[2].X - points3D[1].X;
   dir3D23.Y = points3D[2].Y - points3D[1].Y;
   dir3D23.Z = points3D[2].Z - points3D[1].Z;
   ds12 = dir3D12.X * dir3D12.X + dir3D12.Y * dir3D12.Y + dir3D12.Z * dir3D12.Z;
   ds13 = dir3D13.X * dir3D13.X + dir3D13.Y * dir3D13.Y + dir3D13.Z * dir3D13.Z;
   ds23 = dir3D23.X * dir3D23.X + dir3D23.Y * dir3D23.Y + dir3D23.Z * dir3D23.Z;

   // Solve quartic
   ratio2313 = ds23 / ds13;
   ratio2312 = ds23 / ds12;
   quintic_coefficients[0] = (ratio2313 * ratio2312 - ratio2313 - ratio2312) * (ratio2313 * ratio2312 - ratio2313 - ratio2312) - 4.0 * ratio2313 * ratio2312 * cos23 * cos23;
   quintic_coefficients[1] = 4.0 * (ratio2313 * ratio2312 - ratio2313 - ratio2312) * ratio2312 * (1.0 - ratio2313) * cos12 + 4.0 * ratio2313 * cos23 * ((ratio2313 * ratio2312 + ratio2312 - ratio2313) * cos13 + 2.0 * ratio2312 * cos12 * cos23);
   quintic_coefficients[2] = (2.0 * ratio2312 * (1.0 - ratio2313) * cos12) * (2.0 * ratio2312 * (1.0 - ratio2313) * cos12) + 2.0 * (ratio2313 * ratio2312 + ratio2313 - ratio2312) * (ratio2313 * ratio2312 - ratio2313 - ratio2312) + 4.0 * ratio2313 * ((ratio2313 - ratio2312) * cos23 * cos23 + (1.0 - ratio2312) * ratio2313 * cos13 * cos13 - 2.0 * ratio2312 * (1.0 + ratio2313) * cos12 * cos13 * cos23);
   quintic_coefficients[3] = 4.0 * (ratio2313 * ratio2312 + ratio2313 - ratio2312) * ratio2312 * (1.0 - ratio2313) * cos12 + 4.0 * ratio2313 * ((ratio2313 * ratio2312 - ratio2313 + ratio2312) * cos13 * cos23 + 2.0 * ratio2313 * ratio2312 * cos12 * cos13 * cos13);
   quintic_coefficients[4] = (ratio2313 * ratio2312 + ratio2313 - ratio2312) * (ratio2313 * ratio2312 + ratio2313 - ratio2312) - 4.0 * ratio2313 * ratio2313 * ratio2312 * cos13 * cos13;

   error = rox_quartic_roots(quintic_roots, quintic_coefficients);

   if (error != ROX_ERROR_NONE) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Given each root, try to get a corresponding ray length for each vertex of the 2D triangle
   countsol = 0;
   for (idroot = 0; idroot < 4; idroot++)
   {
      Rox_Double r, i;
      Rox_Double a, b, c, pc1, pc2;
      Rox_Double y, m0, m1, p0, p1, q0, q1;
      Rox_Double base;

      r = quintic_roots[idroot].real;
      i = quintic_roots[idroot].imag;

      // Check if the given root is valid or not
      if (i > 1e-8 || r < 0.0) continue;

      a = sqrt(ds12) / sqrt(r * r - 2.0 * r * cos12 + 1.0);
      b = a * r;
      c = -1;

      m0 = 1.0 - ratio2313;
      q0 = r * r - ratio2313;
      p0 = 2.0 * (ratio2313 * cos13 - r * cos23);
      m1 = 1.0;
      q1 = r * r * (1.0 - ratio2312) + 2 * r * ratio2312 * cos12 - ratio2312;
      p1 = -2 * r * cos23;

      if (fabs(m1 * q0 - m0 * q1) < DBL_EPSILON)
      {
         base = sqrt(cos13 * cos13 + (ds13 - a * a) / (a * a));
         pc1 = (cos13 + base) * a;
         pc2 = (cos13 - base) * a;

         if (fabs((b * b + pc1 * pc1 - 2.0 * b * pc1 * cos23) - (ds13)) < DBL_EPSILON)
         {
            c = pc1;
         }
         else if (fabs((b * b + pc2 * pc2 - 2.0 * b * pc2 * cos23) - (ds13)) < DBL_EPSILON)
         {
            c = pc2;
         }
      }
      else
      {
         y = (p1 * q0 - p0 * q1) / (m0 * q1 - m1 * q0);
         c = y * a;
      }

      if (c > 0.0)
      {
         possible_triangle[countsol][0].X = mproj[0].X * a;
         possible_triangle[countsol][0].Y = mproj[0].Y * a;
         possible_triangle[countsol][0].Z = mproj[0].Z * a;
         possible_triangle[countsol][1].X = mproj[1].X * b;
         possible_triangle[countsol][1].Y = mproj[1].Y * b;
         possible_triangle[countsol][1].Z = mproj[1].Z * b;
         possible_triangle[countsol][2].X = mproj[2].X * c;
         possible_triangle[countsol][2].Y = mproj[2].Y * c;
         possible_triangle[countsol][2].Z = mproj[2].Z * c;

         countsol++;
      }
   }

   // Get the 3D transformation which transforms 3D given triangle to estimated triangle
   for (idroot = 0; idroot < countsol; idroot++)
   {
      curpose = rox_array2d_double_collection_get(poses, idroot);
      error = rox_pose_from_two_3D_triangles(curpose, possible_triangle[idroot], points3D);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   *validposes = countsol;

function_terminate:
   return error;
}

Rox_ErrorCode rox_pose_from_p3p_pix (
   Rox_Array2D_Double_Collection possible_poses,
   Rox_Uint * validposes,
   Rox_Point3D_Double points3D_met,
   Rox_Point2D_Double points2D_pix,
   Rox_MatUT3 calib)//, double fu, double fv, double cu, double cv)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct points2D_nor[3];
   Rox_MatUT3 Ki = NULL;


   //  Check input 
   if (!possible_poses) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!validposes) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!points3D_met) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!points2D_pix) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&Ki, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_svdinverse(Ki, calib);
   ROX_ERROR_CHECK_TERMINATE(error)
   //error = rox_transformtools_build_calibration_matrix(Ki, 1.0/fu, 1.0/fv, -cu/fu, -cv/fv);

   // Compute points2D_nor = Ki * points2D_pix
   error = rox_point2d_double_homography(points2D_nor, points2D_pix, Ki, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_pose_from_p3p_nor(possible_poses, validposes, points3D_met, points2D_nor);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_del(&Ki);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   return error;
}

Rox_ErrorCode rox_pose_from_p3p_nor (
   Rox_Array2D_Double_Collection poses,
   Rox_Uint * validposes,
   Rox_Point3D_Double points3D,
   Rox_Point2D_Double points2D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint countsol;
   Rox_Array2D_Double curpose = NULL;
   Rox_Point3D_Double_Struct mproj[3];
   Rox_Point3D_Double_Struct dir3D12, dir3D13, dir3D23;
   Rox_Double norme, inorme, x, y;
   Rox_Double cos12, cos13, cos23;
   Rox_Double ds12, ds13, ds23;
   Rox_Double ratio2313, ratio2312;
   Rox_Double quintic_coefficients[5];
   Rox_Complex_Struct quintic_roots[4];
   Rox_Point3D_Double_Struct possible_triangle[4][3];

   //  Check input 
   if (!poses)    
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!points2D) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!points3D) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //  Check poses 
   if (rox_array2d_double_collection_get_count(poses) != 4) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   curpose = rox_array2d_double_collection_get(poses, 0);
   error = rox_array2d_double_check_size(curpose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE(error)

   //  First, we need to compute all cZ to get a 3D point cloud
   for ( Rox_Sint id = 0; id < 3; id++)
   {
      //  Get image points in meter space
      x = points2D[id].u;
      y = points2D[id].v;

      norme = sqrt(x * x + y * y + 1.0);
      if (ROX_IS_ZERO_DOUBLE(norme)) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error)}

      inorme = 1.0 / norme;

      mproj[id].X = x * inorme;
      mproj[id].Y = y * inorme;
      mproj[id].Z = inorme;
   }

   // Global idea : a 3D transformation (SE(3)) keeps angles and distances between points.
   // We already know a 3D triangle in world frame (points3D).
   // Given our 2D Triangle in image space, search for a 3D Triangle in camera space which has the same property
   // than the given points3D triangle (angle and distances).
   // See Random Sample Consensus: A Paradigm for Model Fitting with Applicatlons to Image Analysis and Automated Cartography.

   // Get cos(angle) for 3 pairs (1;2 1;3 2;3) for 2D triangle
   cos12 = mproj[0].X * mproj[1].X + mproj[0].Y * mproj[1].Y + mproj[0].Z * mproj[1].Z;
   cos13 = mproj[0].X * mproj[2].X + mproj[0].Y * mproj[2].Y + mproj[0].Z * mproj[2].Z;
   cos23 = mproj[1].X * mproj[2].X + mproj[1].Y * mproj[2].Y + mproj[1].Z * mproj[2].Z;

   // Get 3D points distances squared for 3 pairs (1;2 1;3 2;3) for 3D triangle
   dir3D12.X = points3D[1].X - points3D[0].X;
   dir3D12.Y = points3D[1].Y - points3D[0].Y;
   dir3D12.Z = points3D[1].Z - points3D[0].Z;
   dir3D13.X = points3D[2].X - points3D[0].X;
   dir3D13.Y = points3D[2].Y - points3D[0].Y;
   dir3D13.Z = points3D[2].Z - points3D[0].Z;
   dir3D23.X = points3D[2].X - points3D[1].X;
   dir3D23.Y = points3D[2].Y - points3D[1].Y;
   dir3D23.Z = points3D[2].Z - points3D[1].Z;
   ds12 = dir3D12.X * dir3D12.X + dir3D12.Y * dir3D12.Y + dir3D12.Z * dir3D12.Z;
   ds13 = dir3D13.X * dir3D13.X + dir3D13.Y * dir3D13.Y + dir3D13.Z * dir3D13.Z;
   ds23 = dir3D23.X * dir3D23.X + dir3D23.Y * dir3D23.Y + dir3D23.Z * dir3D23.Z;

   // Solve quartic
   ratio2313 = ds23 / ds13;
   ratio2312 = ds23 / ds12;
   quintic_coefficients[0] = (ratio2313 * ratio2312 - ratio2313 - ratio2312) * (ratio2313 * ratio2312 - ratio2313 - ratio2312) - 4.0 * ratio2313 * ratio2312 * cos23 * cos23;
   quintic_coefficients[1] = 4.0 * (ratio2313 * ratio2312 - ratio2313 - ratio2312) * ratio2312 * (1.0 - ratio2313) * cos12 + 4.0 * ratio2313 * cos23 * ((ratio2313 * ratio2312 + ratio2312 - ratio2313) * cos13 + 2.0 * ratio2312 * cos12 * cos23);
   quintic_coefficients[2] = (2.0 * ratio2312 * (1.0 - ratio2313) * cos12) * (2.0 * ratio2312 * (1.0 - ratio2313) * cos12) + 2.0 * (ratio2313 * ratio2312 + ratio2313 - ratio2312) * (ratio2313 * ratio2312 - ratio2313 - ratio2312) + 4.0 * ratio2313 * ((ratio2313 - ratio2312) * cos23 * cos23 + (1.0 - ratio2312) * ratio2313 * cos13 * cos13 - 2.0 * ratio2312 * (1.0 + ratio2313) * cos12 * cos13 * cos23);
   quintic_coefficients[3] = 4.0 * (ratio2313 * ratio2312 + ratio2313 - ratio2312) * ratio2312 * (1.0 - ratio2313) * cos12 + 4.0 * ratio2313 * ((ratio2313 * ratio2312 - ratio2313 + ratio2312) * cos13 * cos23 + 2.0 * ratio2313 * ratio2312 * cos12 * cos13 * cos13);
   quintic_coefficients[4] = (ratio2313 * ratio2312 + ratio2313 - ratio2312) * (ratio2313 * ratio2312 + ratio2313 - ratio2312) - 4.0 * ratio2313 * ratio2313 * ratio2312 * cos13 * cos13;

   error = rox_quartic_roots(quintic_roots, quintic_coefficients);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Given each root, try to get a corresponding ray length for each vertex of the 2D triangle
   countsol = 0;
   for ( Rox_Sint idroot = 0; idroot < 4; idroot++)
   {
      Rox_Double r, i;
      Rox_Double a, b, c, pc1, pc2;
      Rox_Double y, m0, m1, p0, p1, q0, q1;
      Rox_Double base;

      r = quintic_roots[idroot].real;
      i = quintic_roots[idroot].imag;

      // Check if the given root is valid or not
      if (i > 1e-8 || r < 0.0) continue;

      a = sqrt(ds12) / sqrt(r * r - 2.0 * r * cos12 + 1.0);
      b = a * r;
      c = -1;

      m0 = 1.0 - ratio2313;
      q0 = r * r - ratio2313;
      p0 = 2.0 * (ratio2313 * cos13 - r * cos23);
      m1 = 1.0;
      q1 = r * r * (1.0 - ratio2312) + 2 * r * ratio2312 * cos12 - ratio2312;
      p1 = -2 * r * cos23;

      if (fabs(m1 * q0 - m0 * q1) < DBL_EPSILON)
      {
         base = sqrt(cos13 * cos13 + (ds13 - a * a) / (a * a));
         pc1 = (cos13 + base) * a;
         pc2 = (cos13 - base) * a;

         if (fabs((b * b + pc1 * pc1 - 2.0 * b * pc1 * cos23) - (ds13)) < DBL_EPSILON)
         {
            c = pc1;
         }
         else if (fabs((b * b + pc2 * pc2 - 2.0 * b * pc2 * cos23) - (ds13)) < DBL_EPSILON)
         {
            c = pc2;
         }
      }
      else
      {
         y = (p1 * q0 - p0 * q1) / (m0 * q1 - m1 * q0);
         c = y * a;
      }

      if (c > 0.0)
      {
         possible_triangle[countsol][0].X = mproj[0].X * a;
         possible_triangle[countsol][0].Y = mproj[0].Y * a;
         possible_triangle[countsol][0].Z = mproj[0].Z * a;
         possible_triangle[countsol][1].X = mproj[1].X * b;
         possible_triangle[countsol][1].Y = mproj[1].Y * b;
         possible_triangle[countsol][1].Z = mproj[1].Z * b;
         possible_triangle[countsol][2].X = mproj[2].X * c;
         possible_triangle[countsol][2].Y = mproj[2].Y * c;
         possible_triangle[countsol][2].Z = mproj[2].Z * c;

         countsol++;
      }
   }

   // Get the 3D transformation which transforms 3D given triangle to estimated triangle
   for (Rox_Uint idroot = 0; idroot < countsol; idroot++)
   {
      curpose = rox_array2d_double_collection_get(poses, idroot);
      error = rox_pose_from_two_3D_triangles(curpose, possible_triangle[idroot], points3D);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   *validposes = countsol;

function_terminate:
   return error;
}
