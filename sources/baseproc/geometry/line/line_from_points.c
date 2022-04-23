//==============================================================================
//
//    OPENROX   : File line_from_points.c
//
//    Contents  : Implementation of line_from_points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "line_from_points.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point2d_float_struct.h>

#include <generated/dynvec_uint.h>
#include <generated/dynvec_uint_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/segment/segment3d_struct.h>
#include <baseproc/geometry/segment/segment3d.h>
#include <baseproc/geometry/plane/plane_3points.h>
#include <baseproc/geometry/line/line3d_struct.h>

#include <baseproc/geometry/line/line_from_planes.h>
#include <baseproc/geometry/measures/distance_point_to_line.h>
#include <baseproc/maths/random/combination.h>
#include <baseproc/maths/random/combination_struct.h>

#include <inout/geometry/point/dynvec_point3d_print.h>
#include <inout/geometry/point/dynvec_point2d_print.h>

#include <inout/geometry/point/point2d_print.h>
#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>


Rox_ErrorCode rox_line3d_parametric_from_segment3d (
   Rox_Line3D_Parametric line3d_parametric,
   const Rox_Segment3D segment3d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_line3d_parametric_from_2_point3d ( line3d_parametric, &segment3d->points[0], &segment3d->points[1] );
   ROX_ERROR_CHECK_TERMINATE ( error ); 

function_terminate:
   return error;
}


Rox_ErrorCode rox_line3d_parametric_from_2_point3d (
   Rox_Line3D_Parametric line3d_parametric,
   const Rox_Point3D_Double pt1,
   const Rox_Point3D_Double pt2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !line3d_parametric ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !pt1 || !pt2 ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   line3d_parametric->origin[0] = pt1->X;
   line3d_parametric->origin[1] = pt1->Y;
   line3d_parametric->origin[2] = pt1->Z;

   line3d_parametric->direction[0] = pt2->X - pt1->X;
   line3d_parametric->direction[1] = pt2->Y - pt1->Y;
   line3d_parametric->direction[2] = pt2->Z - pt1->Z;

   Rox_Double length = sqrt( line3d_parametric->direction[0] * line3d_parametric->direction[0] + line3d_parametric->direction[1] * line3d_parametric->direction[1] + line3d_parametric->direction[2] * line3d_parametric->direction[2] );
   if ( length < FLT_EPSILON ) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
 
   line3d_parametric->direction[0] /= length;
   line3d_parametric->direction[1] /= length;
   line3d_parametric->direction[2] /= length;

function_terminate:
   return error;
}


Rox_ErrorCode rox_line3d_planes_from_segment3d (
   Rox_Line3D_Planes line3d_planes,
   const Rox_Segment3D segment3d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_line3d_planes_from_2_point3d ( line3d_planes, &segment3d->points[0], &segment3d->points[1] );
   ROX_ERROR_CHECK_TERMINATE ( error ); 

function_terminate:
   return error;
}

// Since b = [0;-1;-1] we can optimize the function
Rox_ErrorCode rox_solve_linear_systen_optimized ( Rox_Double * x, Rox_Double * A )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Double A00 = A[0];
   Rox_Double A01 = A[1];
   Rox_Double A02 = A[2];
   Rox_Double A10 = A[3];
   Rox_Double A11 = A[4];
   Rox_Double A12 = A[5];
   Rox_Double A20 = A[6];
   Rox_Double A21 = A[7];
   Rox_Double A22 = A[8];

   Rox_Double inv_detA = 1.0/ ( A00*A11*A22 - A00*A12*A21 + A01*A12*A20 - A01*A10*A22  + A02*A10*A21 - A02*A11*A20 );

   x[0] = ( A02*(A11 - A21) + A01*(A22 - A12) ) * inv_detA;
   x[1] = ( A00*(A12 - A22) + A02*(A20 - A10) ) * inv_detA;
   x[2] = ( A01*(A10 - A20) + A00*(A21 - A11) ) * inv_detA;
   return error;
}

Rox_ErrorCode rox_compute_normal_vector ( Rox_Double * nx, Rox_Double * ny, Rox_Double * nz, Rox_Double * tx, Rox_Double * ty, Rox_Double * tz )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   Rox_Double tx2 = (*tx) * (*tx);
   Rox_Double ty2 = (*ty) * (*ty);
   Rox_Double tz2 = (*tz) * (*tz);

   Rox_Double s_tx2_ty2 = sqrt ( tx2 + ty2 );
   Rox_Double s_tx2_tz2 = sqrt ( tx2 + tz2 );
   Rox_Double s_ty2_tz2 = sqrt ( ty2 + tz2 );

   if ( s_ty2_tz2 > FLT_EPSILON )
   {
      *nx =  0.0;
      *ny =  *tz / s_ty2_tz2;
      *nz = -*ty / s_ty2_tz2;
   }
   else if ( s_tx2_tz2 > FLT_EPSILON )
   {
      *nx = -*tz / s_tx2_tz2;
      *ny = 0.0;
      *nz =  *tx / s_tx2_tz2;
   }
   else if ( s_tx2_ty2 > FLT_EPSILON )
   {
      *nx =  *ty / s_tx2_ty2;
      *ny = -*tx / s_tx2_ty2;
      *nz = 0.0;
   }
   return error;
}

Rox_ErrorCode rox_line3d_planes_from_2_point3d (
   Rox_Line3D_Planes line3d_planes,
   const Rox_Point3D_Double pt1,
   const Rox_Point3D_Double pt2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !line3d_planes ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   if ( !pt1 || !pt2 ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double X_1 = pt1->X;
   Rox_Double Y_1 = pt1->Y;
   Rox_Double Z_1 = pt1->Z;

   Rox_Double X_2 = pt2->X;
   Rox_Double Y_2 = pt2->Y;
   Rox_Double Z_2 = pt2->Z;

   // compute tangent vector t
   Rox_Double tx = X_1 - X_2;
   Rox_Double ty = Y_1 - Y_2;
   Rox_Double tz = Z_1 - Z_2;
   
   // verify that tangent vector t is not zero
   Rox_Double length = sqrt ( tx*tx + ty*ty + tz*tz );
   if (length < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double nx_1 = 0.0;
   Rox_Double ny_1 = 0.0;
   Rox_Double nz_1 = 0.0;

   // compute normal n
   rox_compute_normal_vector ( &nx_1, &ny_1, &nz_1, &tx, &ty, &tz );

   Rox_Double a1 = nx_1;
   Rox_Double b1 = ny_1;
   Rox_Double c1 = nz_1;
   Rox_Double d1 = -nx_1 * X_1 - ny_1 * Y_1 - nz_1 * Z_1;

   if (d1 > 0)
   {
     a1 = -a1;
     b1 = -b1;
     c1 = -c1;
     d1 = -d1;
   }

   // build the A matrix   
   Rox_Double A[9];

   A[0] = nx_1; A[1] = ny_1; A[2] = nz_1; 
   A[3] =  X_1; A[4] =  Y_1; A[5] =  Z_1; 
   A[6] =  X_2; A[7] =  Y_2; A[8] =  Z_2; 

   Rox_Double x[3];

   rox_solve_linear_systen_optimized ( x, A );
 
   Rox_Double d2 = - 1.0 / sqrt ( x[0]*x[0] + x[1]*x[1] + x[2]*x[2] ); // Force d2 to be negative
   Rox_Double c2 = x[2] * d2;
   Rox_Double b2 = x[1] * d2;
   Rox_Double a2 = x[0] * d2;

   // First 3D plane
   line3d_planes->planes[0].a = a1;
   line3d_planes->planes[0].b = b1;
   line3d_planes->planes[0].c = c1;
   line3d_planes->planes[0].d = d1;

   // Second 3D plane
   line3d_planes->planes[1].a = a2;
   line3d_planes->planes[1].b = b2;
   line3d_planes->planes[1].c = c2;
   line3d_planes->planes[1].d = d2; 

function_terminate:
   return error;
}

Rox_ErrorCode rox_line3d_planes_from_2_point3d_old (
   Rox_Line3D_Planes line3d_planes,
   const Rox_Point3D_Double pt1,
   const Rox_Point3D_Double pt2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Plane3D_Double_Struct pl1, pl2;
   Rox_Point3D_Double_Struct vec12, vec13, vec23;
   Rox_Point3D_Double_Struct pt3, pt4;

   if (!line3d_planes) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   vec12.X = pt2->X - pt1->X;
   vec12.Y = pt2->Y - pt1->Y;
   vec12.Z = pt2->Z - pt1->Z;
   
   Rox_Double length = sqrt ( vec12.X*vec12.X + vec12.Y*vec12.Y + vec12.Z*vec12.Z );
   if (length < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Get a random point
   pt3.X = 1000.0;
   pt3.Y = 1000.0;
   pt3.Z = 1000.0;
   
   vec13.X = pt3.X - pt1->X;
   vec13.Y = pt3.Y - pt1->Y;
   vec13.Z = pt3.Z - pt1->Z;
   length = sqrt ( vec13.X*vec13.X + vec13.Y*vec13.Y + vec13.Z*vec13.Z );
   
   if (length < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Check that 3rd point is not equal to pt2
   vec23.X = pt3.X - pt2->X;
   vec23.Y = pt3.Y - pt2->Y;
   vec23.Z = pt3.Z - pt2->Z;
   
   length = sqrt ( vec23.X*vec23.X + vec23.Y*vec23.Y + vec23.Z*vec23.Z );

   if (length < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // 4th point orthogonal to line and 3rd point
   pt4.X = vec12.Y * vec13.Z - vec12.Z * vec13.Y;
   pt4.Y = vec12.Z * vec13.X - vec12.X * vec13.Z;
   pt4.Z = vec12.X * vec13.Y - vec12.Y * vec13.X;

   // Build planes
   error = rox_plane3d_from_3_point3d ( &pl1, pt1, pt2, &pt3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_plane3d_from_3_point3d ( &pl2, pt1, pt2, &pt4 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Make representation unique
   error = rox_line3d_planes_from_2_planes ( line3d_planes, &pl1, &pl2 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_line3d_planes_from_2_point3d_float (
   Rox_Line3D_Planes line3d_planes, 
   const Rox_Point3D_Float pt1, 
   const Rox_Point3D_Float pt2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Plane3D_Double_Struct pl1, pl2;
   Rox_Point3D_Float_Struct vec12, vec13, vec23;
   Rox_Point3D_Float_Struct pt3, pt4;
   Rox_Double length;

   if (!line3d_planes) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   vec12.X = pt2->X - pt1->X;
   vec12.Y = pt2->Y - pt1->Y;
   vec12.Z = pt2->Z - pt1->Z;
   
   length = sqrt ( vec12.X*vec12.X + vec12.Y*vec12.Y + vec12.Z*vec12.Z );
   if (length < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Get a random point
   pt3.X = 1.0;
   pt3.Y = 1.0;
   pt3.Z = 1.0;

   vec13.X = pt3.X - pt1->X;
   vec13.Y = pt3.Y - pt1->Y;
   vec13.Z = pt3.Z - pt1->Z;
   length = sqrt ( vec13.X*vec13.X + vec13.Y*vec13.Y + vec13.Z*vec13.Z );
   
   if (length < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Check that 3rd point is not equal to pt2
   vec23.X = pt3.X - pt2->X;
   vec23.Y = pt3.Y - pt2->Y;
   vec23.Z = pt3.Z - pt2->Z;
   
   length = sqrt ( vec23.X*vec23.X + vec23.Y*vec23.Y + vec23.Z*vec23.Z );
   if (length < FLT_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // 4th point orthogonal to line and 3rd point
   pt4.X = vec12.Y * vec13.Z - vec12.Z * vec13.Y;
   pt4.Y = vec12.Z * vec13.X - vec12.X * vec13.Z;
   pt4.Z = vec12.X * vec13.Y - vec12.Y * vec13.X;

   // Build planes
   error = rox_plane3d_from_3_point3d_float ( &pl1, pt1, pt2, &pt3 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_plane3d_from_3_point3d_float ( &pl2, pt1, pt2, &pt4 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Make representation unique
   error = rox_line3d_planes_from_2_planes ( line3d_planes, &pl1, &pl2 ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_line3d_plucker_from_2_point3d (
   Rox_Line3D_Plucker line3d_plucker,
   const Rox_Point3D_Double pt1,
   const Rox_Point3D_Double pt2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!line3d_plucker) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   line3d_plucker->displacement[0] = pt2->X - pt1->X;
   line3d_plucker->displacement[1] = pt2->Y - pt1->Y;
   line3d_plucker->displacement[2] = pt2->Z - pt1->Z;
   line3d_plucker->moment[0] = pt1->Y * pt2->Z - pt1->Z * pt2->Y;
   line3d_plucker->moment[1] = pt1->Z * pt2->X - pt1->X * pt2->Z;
   line3d_plucker->moment[2] = pt1->X * pt2->Y - pt1->Y * pt2->X;

function_terminate:
   return error;
}

Rox_ErrorCode rox_line2d_homogeneous_from_n_point2d (
   Rox_Line2D_Homogeneous line2d_homogeneous,
   const Rox_DynVec_Point2D_Double points2D
)
{
   // Closed form solution of least squares problem
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double mu_10 = 0.0;
   Rox_Double mu_01 = 0.0;
   Rox_Double mu_11 = 0.0;
   Rox_Double mu_20 = 0.0;
   Rox_Double mu_02 = 0.0;

   // Get the number of points in the dynamic vector
   Rox_Uint nbp = points2D->used;
   
   if (!line2d_homogeneous || !points2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Compute means
   for (Rox_Uint i = 0; i < nbp; ++i)
   {
      Rox_Double u = points2D->data[i].u;
      Rox_Double v = points2D->data[i].v;

      mu_10 += u;
      mu_01 += v;

      mu_11 += u * v;
      mu_20 += u * u;
      mu_02 += v * v;
   }

   mu_10 = mu_10/nbp;
   mu_01 = mu_01/nbp;
   mu_11 = mu_11/nbp;
   mu_20 = mu_20/nbp;
   mu_02 = mu_02/nbp;

   Rox_Double a = mu_11 - mu_10 * mu_01;
   Rox_Double b = mu_20 - mu_02 + mu_01*mu_01 - mu_10*mu_10;

   Rox_Double theta = atan ( 2*a/b )/2;

   // Test if det(H) > 0
   Rox_Double sign_det_H = - b * cos(2*theta) ;

   if (sign_det_H <= 0)
   {
      theta = theta - ROX_PI/2;
   }

   Rox_Double rho = mu_10 * cos(theta) + mu_01 * sin(theta);

   error = rox_line2d_homogeneous_from_rho_theta(line2d_homogeneous, rho, theta);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}

Rox_ErrorCode rox_line2d_homogeneous_from_n_point2d_float (
   Rox_Line2D_Homogeneous line2d_homogeneous,
   const Rox_DynVec_Point2D_Float points2D
)
{
   // Closed form solution of least squares problem
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double mu_10 = 0.0;
   Rox_Double mu_01 = 0.0;
   Rox_Double mu_11 = 0.0;
   Rox_Double mu_20 = 0.0;
   Rox_Double mu_02 = 0.0;
   // Get the number of points in the dynamic vector
   Rox_Uint nbp = points2D->used;
   
   if (!line2d_homogeneous || !points2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Compute means
   for (Rox_Uint i = 0; i < nbp; ++i)
   {
      Rox_Double u = (Rox_Double) points2D->data[i].u;
      Rox_Double v = (Rox_Double) points2D->data[i].v;

      mu_10 += u;
      mu_01 += v;

      mu_11 += u * v;
      mu_20 += u * u;
      mu_02 += v * v;
   }

   mu_10 = mu_10/nbp;
   mu_01 = mu_01/nbp;
   mu_11 = mu_11/nbp;
   mu_20 = mu_20/nbp;
   mu_02 = mu_02/nbp;

   Rox_Double a = mu_11 - mu_10 * mu_01;
   Rox_Double b = mu_20 - mu_02 + mu_01*mu_01 - mu_10*mu_10;

   Rox_Double theta = atan(2*a/b)/2;

   // Test if det(H) > 0
   Rox_Double sign_det_H = - b * cos(2*theta) ;

   if (sign_det_H <= 0)
   {
      theta = theta - ROX_PI/2;
   }

   Rox_Double rho = mu_10 * cos(theta) + mu_01 * sin(theta);

   error = rox_line2d_homogeneous_from_rho_theta(line2d_homogeneous, rho, theta);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}

Rox_ErrorCode rox_line2d_homogeneous_from_2_point2d (
   Rox_Line2D_Homogeneous line2d_homogeneous,
   const Rox_Point2D_Double pt1,
   const Rox_Point2D_Double pt2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!line2d_homogeneous) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   line2d_homogeneous->a = pt1->v-pt2->v;
   line2d_homogeneous->b = pt2->u-pt1->u;
   line2d_homogeneous->c = pt2->v*pt1->u - pt2->u*pt1->v;

   Rox_Double norm = sqrt((pt2->u-pt1->u)*(pt2->u-pt1->u)+(pt1->v-pt2->v)*(pt1->v-pt2->v));

   if (norm > 0.0)
   {
      line2d_homogeneous->a = line2d_homogeneous->a/norm;
      line2d_homogeneous->b = line2d_homogeneous->b/norm;
      line2d_homogeneous->c = line2d_homogeneous->c/norm;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_line2d_homogeneous_from_2_point2d_float (
   Rox_Line2D_Homogeneous line2d_homogeneous,
   const Rox_Point2D_Float pt1,
   const Rox_Point2D_Float pt2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!line2d_homogeneous) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   line2d_homogeneous->a = pt1->v-pt2->v;
   line2d_homogeneous->b = pt2->u-pt1->u;
   line2d_homogeneous->c = pt2->v*pt1->u - pt2->u*pt1->v;

   Rox_Double norm = sqrt((pt2->u-pt1->u)*(pt2->u-pt1->u)+(pt1->v-pt2->v)*(pt1->v-pt2->v));

   if (norm > 0.0)
   {
      line2d_homogeneous->a = line2d_homogeneous->a/norm;
      line2d_homogeneous->b = line2d_homogeneous->b/norm;
      line2d_homogeneous->c = line2d_homogeneous->c/norm;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_line2d_separate_inliers_outliers (
   Rox_DynVec_Uint points2D_inliers_indexes,
   Rox_Float * mean_distance,
   const Rox_Line2D_Homogeneous line2d,
   const Rox_DynVec_Point2D_Float points2D,
   const Rox_Float distance_threshold
)
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

      error = rox_distance_point2d_to_line2d(&distance, &points2D->data[k], line2d);
      ROX_ERROR_CHECK_TERMINATE ( error );

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

Rox_ErrorCode rox_dynvec_point2d_float_select_from_indexes (
   Rox_DynVec_Point2D_Float points2D_inliers,
   Rox_DynVec_Uint points2D_inliers_indexes,
   Rox_DynVec_Point2D_Float points2D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Get the number of inlier points in the dynamic vector
   Rox_Uint nbi = points2D_inliers_indexes->used;

   // Reset the dynamic vector
   rox_dynvec_point2d_float_reset(points2D_inliers);

   for (Rox_Uint k = 0; k < nbi; k++)
   {
      // Get the point from the input vector
      Rox_Point2D_Float point = &points2D->data[points2D_inliers_indexes->data[k]];

      // Append the point to the inliers vector
      error = rox_dynvec_point2d_float_append(points2D_inliers, point);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_line2d_parmetric_print (
   const Rox_Line2D_Homogeneous line2D
)
{
    rox_log("line2D = (%f, %f, %f)\n", line2D->a, line2D->b, line2D->c);
    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_line2d_homogeneous_from_n_point2d_ransac(
   Rox_Line2D_Homogeneous line2d_homogeneous,
   const Rox_DynVec_Point2D_Float points2D,
   const Rox_Float distance_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Uint points2D_inliers_indexes = NULL;
   Rox_DynVec_Uint points2D_inliers_indexes_best = NULL;

   //! The random combination for the 2D points
   Rox_Combination combination_points2D = NULL;

   if (!line2d_homogeneous || !points2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the number of points in the dynamic vector
   Rox_Uint nbp = points2D->used;

   // Test if we have at least 3 points otherwise we do not need ransac to estimate a line through2 points
   if (nbp < 3)
   { error = ROX_ERROR_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Define the indexes for the inliers
   error = rox_dynvec_uint_new(&points2D_inliers_indexes, nbp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Define the indexes for the inliers
   error = rox_dynvec_uint_new(&points2D_inliers_indexes_best, nbp);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create a new combination for selecting 2 from the set of 2D points
   error = rox_combination_new(&combination_points2D, nbp, 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute the number of trials
   Rox_Uint nbt = 100;

   // Init
   Rox_Uint nbp_inliers_best = nbp/2; // Accept only if at least half of the points are on the line
   Rox_Float mean_distance_best = FLT_MAX;

   for (Rox_Uint k = 0; k < nbt; k++)
   {
      Rox_Point2D_Float point_0 = NULL;
      Rox_Point2D_Float point_1 = NULL;
      Rox_Line2D_Homogeneous_Struct line2d_candidate;
      Rox_Float mean_distance = 0.0;
      Rox_DynVec_Uint draw_point2D = NULL;

      // Draw a new combination for 2D points
      error = rox_combination_draw(combination_points2D);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // TODO use the getter
      draw_point2D = combination_points2D->draw;

      // Get the couple of points
      point_0 = &points2D->data[draw_point2D->data[0]];
      point_1 = &points2D->data[draw_point2D->data[1]];

      // Compute the line2d passing through the points
      error = rox_line2d_homogeneous_from_2_point2d_float(&line2d_candidate, point_0, point_1);
      ROX_ERROR_CHECK_CONTINUE(error);

      // Get the inliers and mean_distance
      error = rox_line2d_separate_inliers_outliers(points2D_inliers_indexes, &mean_distance, &line2d_candidate, points2D, distance_threshold);
      ROX_ERROR_CHECK_CONTINUE(error);

      // Get the number of inlier points in the dynamic vector
      Rox_Uint nbp_inliers = points2D_inliers_indexes->used;

      // Test if current trial has more inliers than the best one and half the points are inliers
      if ((nbp_inliers >= nbp / 2) && (nbp_inliers >= nbp_inliers_best))
      {
         // Test if current trial has smaller mean distance
         if (mean_distance < mean_distance_best)
         {
            // Store the best inliers indexes
            error = rox_dynvec_uint_clone(points2D_inliers_indexes_best, points2D_inliers_indexes);
            ROX_ERROR_CHECK_CONTINUE(error);

            nbp_inliers_best = nbp_inliers;
            mean_distance_best = mean_distance;
         }
      }
   }

   // Accept only if at least half of the points are on the line
   if (nbp_inliers_best > nbp/2)
   {
      Rox_DynVec_Point2D_Float points2D_inliers = NULL;

      error = rox_dynvec_point2d_float_new(&points2D_inliers, nbp_inliers_best);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Get the inliers
      error = rox_dynvec_point2d_float_select_from_indexes(points2D_inliers, points2D_inliers_indexes_best, points2D);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute the line corresponding to the best inliers
      error = rox_line2d_homogeneous_from_n_point2d_float(line2d_homogeneous, points2D_inliers);
	   ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point2d_float_del(&points2D_inliers);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      error = ROX_ERROR_ALGORITHM_FAILURE; goto function_terminate;
   }

function_terminate:

   rox_dynvec_uint_del(&points2D_inliers_indexes);
   rox_dynvec_uint_del(&points2D_inliers_indexes_best);
   rox_combination_del(&combination_points2D);

   return error;
}


Rox_ErrorCode rox_line2d_homogeneous_from_rho_theta (
   Rox_Line2D_Homogeneous line2d_homogeneous,
   const Rox_Double rho,
   const Rox_Double theta
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !line2d_homogeneous ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // If theta = 0 we get an vertical line
   // if theta = pi/2 we get a horizontal line
   line2d_homogeneous->a = +cos ( theta );
   line2d_homogeneous->b = +sin ( theta );

   line2d_homogeneous->c = -rho;

function_terminate:
   return error;
}

Rox_ErrorCode rox_line2d_normal_from_2_point2d (
   Rox_Line2D_Normal line2d_normal,
   const Rox_Point2D_Double pt1,
   const Rox_Point2D_Double pt2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!line2d_normal) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Compute tangent
   Rox_Double dx = pt2->u-pt1->u;
   Rox_Double dy = pt2->v-pt1->v;
   Rox_Double norm = sqrt(dx*dx+dy*dy);
   
   if (fabs(norm) < DBL_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   line2d_normal->theta = atan2(dx, -dy);
   line2d_normal->rho = cos(line2d_normal->theta) * pt1->u + sin(line2d_normal->theta) * pt1->v;

function_terminate:
   return error;
}
