//==============================================================================
//
//    OPENROX   : File jacobian_row_bundle_matse3_point3d.c
//
//    Contents  : Implementation of jacobian_row_bundle_matse3_point3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "jacobian_row_bundle_matse3_point3d.h"

#include <float.h>

#include <system/errors/errors.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_se3bundle_row_from_points_float (
   Rox_Double * rowuJpose, 
   Rox_Double * rowvJpose, 
   Rox_Double * rowuJpoint, 
   Rox_Double * rowvJpoint, 
   Rox_Double ** dt, 
   Rox_Point3D_Double transformed_point3d, 
   Rox_Point3D_Double point3d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double Jp[3][3], Jt[3][6];
   
   Rox_Double X = point3d->X;
   Rox_Double Y = point3d->Y;
   Rox_Double Z = point3d->Z;
   
   if (fabs(transformed_point3d->Z) < DBL_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double dx = 1.0 / transformed_point3d->Z;
   Rox_Double dy = 1.0 / transformed_point3d->Z;
   Rox_Double cx = - transformed_point3d->X / (transformed_point3d->Z * transformed_point3d->Z);
   Rox_Double cy = - transformed_point3d->Y / (transformed_point3d->Z * transformed_point3d->Z);

   Jp[0][0] = dt[0][0];
   Jp[0][1] = dt[0][1];
   Jp[0][2] = dt[0][2];
   Jp[1][0] = dt[1][0];
   Jp[1][1] = dt[1][1];
   Jp[1][2] = dt[1][2];
   Jp[2][0] = dt[2][0];
   Jp[2][1] = dt[2][1];
   Jp[2][2] = dt[2][2];

   Jt[0][0] = dt[0][0];
   Jt[0][1] = dt[0][1];
   Jt[0][2] = dt[0][2];
   Jt[0][3] = Y * dt[0][2] - Z * dt[0][1];
   Jt[0][4] = -X * dt[0][2] + Z * dt[0][0];
   Jt[0][5] = X * dt[0][1] - Y * dt[0][0];
   Jt[1][0] = dt[1][0];
   Jt[1][1] = dt[1][1];
   Jt[1][2] = dt[1][2];
   Jt[1][3] = Y * dt[1][2] - Z * dt[1][1];
   Jt[1][4] = -X * dt[1][2] + Z * dt[1][0];
   Jt[1][5] = X * dt[1][1] - Y * dt[1][0];
   Jt[2][0] = dt[2][0];
   Jt[2][1] = dt[2][1];
   Jt[2][2] = dt[2][2];
   Jt[2][3] = Y * dt[2][2] - Z * dt[2][1];
   Jt[2][4] = -X * dt[2][2] + Z * dt[2][0];
   Jt[2][5] = X * dt[2][1] - Y * dt[2][0];

   for ( Rox_Sint i = 0; i < 3; i++)
   {
      rowuJpoint[i] = dx * Jp[0][i] + cx * Jp[2][i];
      rowvJpoint[i] = dy * Jp[1][i] + cy * Jp[2][i];
   }

   for ( Rox_Sint i = 0; i < 6; i++)
   {
      rowuJpose[i] = dx * Jt[0][i] + cx * Jt[2][i];
      rowvJpose[i] = dy * Jt[1][i] + cy * Jt[2][i];
   }

function_terminate:
   return error;
}
