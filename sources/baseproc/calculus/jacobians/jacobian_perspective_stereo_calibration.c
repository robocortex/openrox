//==============================================================================
//
//    OPENROX   : File jacobian_perspective_stereo_calibration.c
//
//    Contents  : Implementation of jacobian_perspective_stereo_calibration module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "jacobian_perspective_stereo_calibration.h"

#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_jacobian_perspective_stereo_calibration (
   Rox_Array2D_Double Jk, 
   const Rox_Point2D_Double pc, 
   const Rox_Uint nbpts
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!Jk || !pc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check size 
   error = rox_array2d_double_check_size(Jk, 2*nbpts, 5); 
   ROX_ERROR_CHECK_TERMINATE ( error );


   Rox_Double ** dJ = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ, Jk);

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      Rox_Uint row = i * 2;

      // First row
      dJ[row][0] = pc[i].u;
      dJ[row][1] = pc[i].v;
      dJ[row][2] = 1.0;
      dJ[row][3] = 0.0;
      dJ[row][4] = 0.0;

      // Second row
      dJ[row+1][0] = 0.0;
      dJ[row+1][1] = 0.0;
      dJ[row+1][2] = 0.0;
      dJ[row+1][3] = pc[i].v;
      dJ[row+1][4] = 1.0;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_jacobian_perspective_stereo_calibration_f_cu_cv (
   Rox_Array2D_Double Jk, 
   Rox_Point2D_Double pc, 
   Rox_Uint nbpts
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!Jk || !pc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check size 
   error = rox_array2d_double_check_size(Jk, 2*nbpts, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dJ = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ, Jk);

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
       Rox_Uint row = i * 2;

       // First row
       dJ[row][0] = pc[i].u;
       dJ[row][1] = 1.0;
       dJ[row][2] = 0.0;

       // Second row
       dJ[row+1][0] = pc[i].v;
       dJ[row+1][1] = 0.0;
       dJ[row+1][2] = 1.0;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_jacobian_perspective_stereo_calibration_pose (
   Rox_Array2D_Double JT, 
   Rox_Array2D_Double K, 
   Rox_Array2D_Double pose, 
   Rox_Point3D_Double pts, 
   Rox_Point2D_Double pc, 
   Rox_Double * zc, 
   Rox_Uint nbpts
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!JT || !K || !pose || !pts || !pc || !zc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check size 
   error = rox_array2d_double_check_size(JT, 2*nbpts, 6); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dJ = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ, JT);

   Rox_Double ** dK = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dK, K);

   Rox_Double ** dT = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dT, pose);

   Rox_Double k11 = dK[0][0]; Rox_Double k12 = dK[0][1]; Rox_Double k13 = dK[0][2]; Rox_Double k22 = dK[1][1]; Rox_Double k23 = dK[1][2];

   Rox_Double T11 = dT[0][0]; Rox_Double T12 = dT[0][1]; Rox_Double T13 = dT[0][2];
   Rox_Double T21 = dT[1][0]; Rox_Double T22 = dT[1][1]; Rox_Double T23 = dT[1][2];
   Rox_Double T31 = dT[2][0]; Rox_Double T32 = dT[2][1]; Rox_Double T33 = dT[2][2];

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      Rox_Uint row = 2*i;

      Rox_Double X = pts[i].X;
      Rox_Double Y = pts[i].Y;
      Rox_Double Z = pts[i].Z;

      // TODO: test if zc[i] is close to zero
      Rox_Double zi = 1.0 / zc[i];

      Rox_Double J11 = k11 * T11 + k12 * T21 + (k13 - pc[i].u) * T31;
      Rox_Double J12 = k11 * T12 + k12 * T22 + (k13 - pc[i].u) * T32;
      Rox_Double J13 = k11 * T13 + k12 * T23 + (k13 - pc[i].u) * T33;

      Rox_Double J21 = k22 * T21 + (k23 - pc[i].v) * T31;
      Rox_Double J22 = k22 * T22 + (k23 - pc[i].v) * T32;
      Rox_Double J23 = k22 * T23 + (k23 - pc[i].v) * T33;

      // First row
      dJ[row][0] = zi * J11;
      dJ[row][1] = zi * J12;
      dJ[row][2] = zi * J13;
      dJ[row][3] = zi * (-Z * J12 + Y * J13);
      dJ[row][4] = zi * ( Z * J11 - X * J13);
      dJ[row][5] = zi * (-Y * J11 + X * J12);

      // Second row
      dJ[row+1][0] = zi * J21;
      dJ[row+1][1] = zi * J22;
      dJ[row+1][2] = zi * J23;
      dJ[row+1][3] = zi * (-Z * J22 + Y * J23);
      dJ[row+1][4] = zi * ( Z * J21 - X * J23);
      dJ[row+1][5] = zi * (-Y * J21 + X * J22);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_jacobian_perspective_stereo_calibration_pose_intercamera (
   Rox_Matrix JT, 
   Rox_MatUT3 K, 
   Rox_MatSE3 rTl, 
   Rox_MatSE3 lTo, 
   Rox_MatSE3 rTo, 
   Rox_Point3D_Double pts, 
   Rox_Uint nbpts
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Point3D_Double  pts_l = NULL;
   Rox_Point2D_Double  pc = NULL;

   Rox_Double * zc = NULL;

   if (!JT || !K || !rTl || !lTo || !rTo || !pts) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check size 
   error = rox_array2d_double_check_size(JT, 2*nbpts, 6); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Allocate memory 
   pc = (Rox_Point2D_Double) rox_memory_allocate(sizeof(*pc), nbpts);
   if (!pc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   zc = (Rox_Double*) rox_memory_allocate(sizeof(*zc), nbpts);
   if (!zc) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   pts_l = (Rox_Point3D_Double ) rox_memory_allocate(sizeof(*pts_l), nbpts);
   if (!pts_l) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_point2d_double_transform_project(pc, zc, K, rTo, pts, nbpts);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_point3d_double_transform(pts_l, lTo, pts, nbpts);
   ROX_ERROR_CHECK_TERMINATE(error)

   Rox_Double ** dJ = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dJ, JT);

   Rox_Double ** dK = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dK, K);

   Rox_Double ** dT = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dT, rTl);

   Rox_Double k11 = dK[0][0]; Rox_Double k12 = dK[0][1]; Rox_Double k13 = dK[0][2]; 
                              Rox_Double k22 = dK[1][1]; Rox_Double k23 = dK[1][2];

   Rox_Double T11 = dT[0][0]; Rox_Double T12 = dT[0][1]; Rox_Double T13 = dT[0][2];
   Rox_Double T21 = dT[1][0]; Rox_Double T22 = dT[1][1]; Rox_Double T23 = dT[1][2];
   Rox_Double T31 = dT[2][0]; Rox_Double T32 = dT[2][1]; Rox_Double T33 = dT[2][2];

   for (Rox_Uint i = 0; i < nbpts; i++)
   {
      Rox_Uint row = 2*i;

      Rox_Double X = pts_l[i].X;
      Rox_Double Y = pts_l[i].Y;
      Rox_Double Z = pts_l[i].Z;

      Rox_Double zi = 1.0 / zc[i];

      Rox_Double J11 = k11 * T11 + k12 * T21 + (k13 - pc[i].u) * T31;
      Rox_Double J12 = k11 * T12 + k12 * T22 + (k13 - pc[i].u) * T32;
      Rox_Double J13 = k11 * T13 + k12 * T23 + (k13 - pc[i].u) * T33;

      Rox_Double J21 = k22 * T21 + (k23 - pc[i].v) * T31;
      Rox_Double J22 = k22 * T22 + (k23 - pc[i].v) * T32;
      Rox_Double J23 = k22 * T23 + (k23 - pc[i].v) * T33;

      // First row
      dJ[row][0] = zi * J11;
      dJ[row][1] = zi * J12;
      dJ[row][2] = zi * J13;
      dJ[row][3] = zi * (-Z * J12 + Y * J13);
      dJ[row][4] = zi * ( Z * J11 - X * J13);
      dJ[row][5] = zi * (-Y * J11 + X * J12);

      // Second row
      dJ[row+1][0] = zi * J21;
      dJ[row+1][1] = zi * J22;
      dJ[row+1][2] = zi * J23;
      dJ[row+1][3] = zi * (-Z * J22 + Y * J23);
      dJ[row+1][4] = zi * ( Z * J21 - X * J23);
      dJ[row+1][5] = zi * (-Y * J21 + X * J22);
   }

function_terminate:
   rox_memory_delete(pc);
   rox_memory_delete(zc);
   rox_memory_delete(pts_l);

   return error;
}
