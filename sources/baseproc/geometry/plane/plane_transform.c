//==============================================================================
//
//    OPENROX   : File plane_transform.c
//
//    Contents  : Implementation of plane_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "plane_transform.h"

#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_plane3d_transform (
   Rox_Plane3D_Double plane3d_out, 
   const Rox_MatSE3 pose, 
   const Rox_Plane3D_Double plane3d_inp
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!plane3d_inp || !pose ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!plane3d_out ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double * transformed_a = &(plane3d_out->a); 
   Rox_Double * transformed_b = &(plane3d_out->b);
   Rox_Double * transformed_c = &(plane3d_out->c);
   Rox_Double * transformed_d = &(plane3d_out->d);
   Rox_Double a = plane3d_inp->a;
   Rox_Double b = plane3d_inp->b;
   Rox_Double c = plane3d_inp->c;
   Rox_Double d = plane3d_inp->d;
   
   error = rox_plane_transform ( transformed_a, transformed_b, transformed_c, transformed_d, pose, a, b, c, d);

function_terminate:
   return error;
}

Rox_ErrorCode rox_plane_transform (
   Rox_Double * transformed_a, 
   Rox_Double * transformed_b, 
   Rox_Double * transformed_c, 
   Rox_Double * transformed_d, 
   const Rox_MatSE3 pose, 
   const Rox_Double a, 
   const Rox_Double b, 
   const Rox_Double c, 
   const Rox_Double d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!transformed_a || !transformed_b || !transformed_c || !transformed_c) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Let n be the old normal, p a point on the plane, and d the distance
   // Let pp be the new point, R and T is the transformation applied to the space
   //
   // pp = R.p + T
   // n(t).p=-d (eq 1)
   // nn(t).pp= -nd
   // nn(t).(R.p + T) = -nd
   // nn(t).R.p + nn(t).T = -nd
   //
   // nn(t) = n(t).R^-1
   // n(t).R^-1.R.p + n(t).R^-1.T = -nd
   // n(t).p + n(t).R^-1.T = nd --> using eq(1) :
   //  -nd = -d + n(t).R^-1.T
   //
   // nn = (n(t).R^-1)(t) = R^-1(t)*n(t)(t) = R*n (^-1 = ^t for rotations)
   //
   // n(t).R^-1.T is a scalar so :
   // -nd = -d + ((n(t).R^-1).T)(t)
   // -nd = -d + (T(t).(n(t).R^-1)(t))
   // -nd = -d + T(t).R.n = -d + T(t).nn
   //  nd = d - T(t).nn

   // New normal is simply the rotation of the old normal
   Rox_Double na = dt[0][0] * a + dt[0][1] * b + dt[0][2] * c;
   Rox_Double nb = dt[1][0] * a + dt[1][1] * b + dt[1][2] * c;
   Rox_Double nc = dt[2][0] * a + dt[2][1] * b + dt[2][2] * c;

   // new distance is computed by solving
   Rox_Double nd = d - (na*dt[0][3] + nb*dt[1][3] + nc*dt[2][3]);

   *transformed_a = na;
   *transformed_b = nb;
   *transformed_c = nc;
   *transformed_d = nd;
   
function_terminate:
   return error;
}

