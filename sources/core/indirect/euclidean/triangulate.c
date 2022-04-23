//==============================================================================
//
//    OPENROX   : File triangulate.c
//
//    Contents  : Implementation of triangulate module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "triangulate.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_pose_triangulate_simple (
   Rox_Point3D_Double result, 
   const Rox_Point2D_Double ref2D, 
   const Rox_Point2D_Double cur2D, 
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double E[3][3];

   if (!result || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size(pose); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dt, pose );

   Rox_Double qx = ref2D->u;
   Rox_Double qy = ref2D->v;
   Rox_Double qpx = cur2D->u;
   Rox_Double qpy = cur2D->v;

   E[0][0] = -dt[2][3] * dt[1][0] + dt[1][3] * dt[2][0];
   E[0][1] = -dt[2][3] * dt[1][1] + dt[1][3] * dt[2][1];
   E[0][2] = -dt[2][3] * dt[1][2] + dt[1][3] * dt[2][2];
   E[1][0] =  dt[2][3] * dt[0][0] - dt[0][3] * dt[2][0];
   E[1][1] =  dt[2][3] * dt[0][1] - dt[0][3] * dt[2][1];
   E[1][2] =  dt[2][3] * dt[0][2] - dt[0][3] * dt[2][2];
   E[2][0] = -dt[1][3] * dt[0][0] + dt[0][3] * dt[1][0];
   E[2][1] = -dt[1][3] * dt[0][1] + dt[0][3] * dt[1][1];
   E[2][2] = -dt[1][3] * dt[0][2] + dt[0][3] * dt[1][2];

   Rox_Double X = -E[1][0] * qx - E[1][1] * qy - E[1][2];
   Rox_Double Y = E[0][0] * qx + E[0][1] * qy + E[0][2];
   Rox_Double Z = qpx * E[1][0] * qx + qpx * E[1][1] * qy + qpx * E[1][2] - qpy * E[0][0] * qx - qpy * E[0][1] * qy - qpy * E[0][2];

   Rox_Double C1 = dt[0][0] * X + dt[1][0] * Y + dt[2][0] * Z;
   Rox_Double C2 = dt[0][1] * X + dt[1][1] * Y + dt[2][1] * Z;
   Rox_Double C3 = dt[0][2] * X + dt[1][2] * Y + dt[2][2] * Z;
   Rox_Double C4 = dt[0][3] * X + dt[1][3] * Y + dt[2][3] * Z;

   Rox_Double denom = - 1.0 / (qx * C1 + qy * C2 + C3);

   result->X = (C4 * qx) * denom;
   result->Y = (C4 * qy) * denom;
   result->Z = (C4     ) * denom;

function_terminate:
   return error;
}

Rox_ErrorCode rox_pose_triangulate_oneway (
   Rox_Point3D_Double result, 
   const Rox_Point2D_Double ref2D, 
   const Rox_Point2D_Double cur2D, 
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double AtA[2][2], Att[2], sol[2], denom;

   if (!result || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, pose);

   // zc*qc = R*qr*zr + t
   // zc*qc - R*qr*zr = t
   // [qc -R*qr][zc zr]' = t
   
   Rox_Double a11 = cur2D->u;
   Rox_Double a21 = cur2D->v;
   Rox_Double a31 = 1.0;
   Rox_Double a12 = - (dt[0][0] * ref2D->u + dt[0][1] * ref2D->v + dt[0][2]);
   Rox_Double a22 = - (dt[1][0] * ref2D->u + dt[1][1] * ref2D->v + dt[1][2]);
   Rox_Double a32 = - (dt[2][0] * ref2D->u + dt[2][1] * ref2D->v + dt[2][2]);

   AtA[0][0] = a11 * a11 + a21 * a21 + a31 * a31;
   AtA[0][1] = a11 * a12 + a21 * a22 + a31 * a32;
   AtA[1][0] = a11 * a12 + a21 * a22 + a31 * a32;
   AtA[1][1] = a12 * a12 + a22 * a22 + a32 * a32;

   Att[0] = a11 * dt[0][3] + a21 * dt[1][3] + a31 * dt[2][3];
   Att[1] = a12 * dt[0][3] + dt[1][3] * a22 + dt[2][3] * a32;

   denom =  (AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0]);
   if (fabs(denom) < DBL_EPSILON) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error)}

   denom = 1.0 / denom;

   sol[0] = ( AtA[1][1] * Att[0] - AtA[0][1] * Att[1]) * denom;
   sol[1] = (-AtA[1][0] * Att[0] + AtA[0][0] * Att[1]) * denom;

   result->X = ref2D->u * sol[1];
   result->Y = ref2D->v * sol[1];
   result->Z = sol[1];

function_terminate:
   return error;
}

Rox_ErrorCode rox_pose_triangulate_correction_gold ( 
   Rox_Point2D_Double corref, 
   Rox_Point2D_Double corcur, 
   const Rox_Point2D_Double ref2D, 
   const Rox_Point2D_Double cur2D, 
   const Rox_MatSE3 pose
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double E[3][3];
   Rox_Double n[2], np[2],dx[2],dxp[2];
   Rox_Double cx,cy,rx,ry;
   Rox_Double a,b,c,d,l;

   if (!corcur || !corref || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Double ** dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   E[0][0] = -dt[2][3] * dt[1][0] + dt[1][3] * dt[2][0];
   E[0][1] = -dt[2][3] * dt[1][1] + dt[1][3] * dt[2][1];
   E[0][2] = -dt[2][3] * dt[1][2] + dt[1][3] * dt[2][2];
   E[1][0] =  dt[2][3] * dt[0][0] - dt[0][3] * dt[2][0];
   E[1][1] =  dt[2][3] * dt[0][1] - dt[0][3] * dt[2][1];
   E[1][2] =  dt[2][3] * dt[0][2] - dt[0][3] * dt[2][2];
   E[2][0] = -dt[1][3] * dt[0][0] + dt[0][3] * dt[1][0];
   E[2][1] = -dt[1][3] * dt[0][1] + dt[0][3] * dt[1][1];
   E[2][2] = -dt[1][3] * dt[0][2] + dt[0][3] * dt[1][2];

   rx = ref2D->u; ry = ref2D->v;
   cx = cur2D->u; cy = cur2D->v;

   n[0] = E[0][0] * ref2D->u + E[0][1] * ref2D->v + E[0][2];
   n[1] = E[1][0] * ref2D->u + E[1][1] * ref2D->v + E[1][2];

   np[0] = E[0][0] * cur2D->u + E[1][0] * cur2D->v + E[2][0];
   np[1] = E[0][1] * cur2D->u + E[1][1] * cur2D->v + E[2][1];

   a = n[0] * (E[0][0] * np[0] + E[0][1] * np[1]) + n[1] * (E[1][0] * np[0] + E[1][1] * np[1]);
   b = 0.5 * (n[0] * n[0] + n[1] * n[1] + np[0] * np[0] + np[1] * np[1]);
   c = cx * (E[0][0] * rx + E[0][1] * ry + E[0][2]) + cy * (E[1][0] * rx + E[1][1] * ry + E[1][2]) + (E[2][0] * rx + E[2][1] * ry + E[2][2]);
   d = sqrt(b*b - a*c);

   l = c / (b + d);

   dx[0] = l * n[0];
   dx[1] = l * n[1];

   dxp[0] = l * np[0];
   dxp[1] = l * np[1];

   n[0] = n[0] - (E[0][0] * dxp[0] + E[0][1] * dxp[1]);
   n[1] = n[1] - (E[1][0] * dxp[0] + E[1][1] * dxp[1]);

   np[0] = np[0] - (E[0][0] * dx[0] + E[1][0] * dx[1]);
   np[1] = np[1] - (E[0][1] * dx[0] + E[1][1] * dx[1]);

   l = l * (2.0 * d) / (n[0] * n[0] + n[1] * n[1] + np[0] * np[0] + np[1] * np[1]);

   dx[0] = l * n[0];
   dx[1] = l * n[1];

   dxp[0] = l * np[0];
   dxp[1] = l * np[1];

   corcur->u = cur2D->u - dx[0];
   corcur->v = cur2D->v - dx[1];

   corref->u = ref2D->u - dxp[0];
   corref->v = ref2D->v - dxp[1];

function_terminate:
   return error;
}
