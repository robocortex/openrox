//==============================================================================
//
//    OPENROX   : File point2d_projection_from_point3d.c
//
//    Contents  : Implementation of pointsproject module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point2d_projection_from_point3d.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_point2d_double_project (
   Rox_Point2D_Double output2d, 
   Rox_Point3D_Double input, 
   Rox_Array2D_Double calib, 
   const Rox_Sint nbpts
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output2d || !calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double px = K[0][0];
   Rox_Double py = K[1][1];
   Rox_Double u0 = K[0][2];
   Rox_Double v0 = K[1][2];

   for (Rox_Sint i = 0; i < nbpts; i++)
   {
      if (ROX_IS_ZERO_DOUBLE(input[i].Z))
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      Rox_Double x = input[i].X / input[i].Z;
      Rox_Double y = input[i].Y / input[i].Z;

      output2d[i].u = px * x + u0;
      output2d[i].v = py * y + v0;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_double_transform_project (
   Rox_Point2D_Double res, 
   Rox_Double * depth,
   const Rox_Array2D_Double calib,
   const Rox_MatSE3 pose,
   const Rox_Point3D_Double input,
   const Rox_Sint nbpts
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!depth)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double px = K[0][0];
   Rox_Double py = K[1][1];
   Rox_Double u0 = K[0][2];
   Rox_Double v0 = K[1][2];
   Rox_Double sk = K[0][1];

   Rox_Double **dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < nbpts; i++)
   {
      Rox_Double X = dt[0][0] * input[i].X + dt[0][1] * input[i].Y + dt[0][2] * input[i].Z + dt[0][3];
      Rox_Double Y = dt[1][0] * input[i].X + dt[1][1] * input[i].Y + dt[1][2] * input[i].Z + dt[1][3];
      Rox_Double Z = dt[2][0] * input[i].X + dt[2][1] * input[i].Y + dt[2][2] * input[i].Z + dt[2][3];

      if (fabs(Z) < DBL_EPSILON) Z = DBL_EPSILON;

      Rox_Double x = X / Z;
      Rox_Double y = Y / Z;

      res[i].u = px * x + sk * y + u0;
      res[i].v = py * y + v0;

      depth[i] = Z;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_float_project(Rox_Point2D_Float output2d, Rox_Point3D_Float input, Rox_Array2D_Double calib, const Rox_Sint nbpts)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output2d || !calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K, calib);

   Rox_Double px = K[0][0];
   Rox_Double py = K[1][1];
   Rox_Double u0 = K[0][2];
   Rox_Double v0 = K[1][2];

   for (Rox_Sint i = 0; i < nbpts; i++)
   {
      if (ROX_IS_ZERO_DOUBLE(input[i].Z))
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      Rox_Double x = input[i].X / input[i].Z;
      Rox_Double y = input[i].Y / input[i].Z;

      output2d[i].u = (Rox_Float) (px * x + u0);
      output2d[i].v = (Rox_Float) (py * y + v0);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_float_transform_project (
   Rox_Point2D_Float res, Rox_Float * depth,
   const Rox_MatUT3 calib,
   const Rox_MatSE3 pose,
   const Rox_Point3D_Float input,
   const Rox_Sint nbpts
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!depth)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &K, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float px = (Rox_Float) K[0][0];
   Rox_Float py = (Rox_Float) K[1][1];
   Rox_Float u0 = (Rox_Float) K[0][2];
   Rox_Float v0 = (Rox_Float) K[1][2];
   Rox_Float sk = (Rox_Float) K[0][1];

   Rox_Double **dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < nbpts; i++)
   {
      Rox_Float X = (Rox_Float) (dt[0][0] * input[i].X + dt[0][1] * input[i].Y + dt[0][2] * input[i].Z + dt[0][3]);
      Rox_Float Y = (Rox_Float) (dt[1][0] * input[i].X + dt[1][1] * input[i].Y + dt[1][2] * input[i].Z + dt[1][3]);
      Rox_Float Z = (Rox_Float) (dt[2][0] * input[i].X + dt[2][1] * input[i].Y + dt[2][2] * input[i].Z + dt[2][3]);

      if (fabsf(Z) < DBL_EPSILON) Z = DBL_EPSILON;

      Rox_Float x = X / Z;
      Rox_Float y = Y / Z;

      res[i].u = px * x + sk * y + u0;
      res[i].v = py * y + v0;

      depth[i] = Z;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_float_project_meters ( Rox_Point2D_Float q, Rox_Point3D_Float input, const Rox_Sint nbpts )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!q)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Sint i = 0; i < nbpts; i++)
   {
      if (ROX_IS_ZERO_DOUBLE(input[i].Z))
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      Rox_Double iZ = 1.0 / input[i].Z;
      Rox_Double x = input[i].X * iZ;
      Rox_Double y = input[i].Y * iZ;

      q[i].u = (Rox_Float) x;
      q[i].v = (Rox_Float) y;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_float_transform_project_meters (
   Rox_Point2D_Float q, Rox_Float * depth,
   const Rox_MatSE3 pose,
   const Rox_Point3D_Float input,
   const Rox_Sint nbpts
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!depth)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!q)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **dt = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer(&dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < nbpts; i++)
   {
      Rox_Float X = (Rox_Float) (dt[0][0] * input[i].X + dt[0][1] * input[i].Y + dt[0][2] * input[i].Z + dt[0][3]);
      Rox_Float Y = (Rox_Float) (dt[1][0] * input[i].X + dt[1][1] * input[i].Y + dt[1][2] * input[i].Z + dt[1][3]);
      Rox_Float Z = (Rox_Float) (dt[2][0] * input[i].X + dt[2][1] * input[i].Y + dt[2][2] * input[i].Z + dt[2][3]);

      if (fabsf(Z) < FLT_EPSILON) Z = FLT_EPSILON;

      q[i].u = X / Z;
      q[i].v = Y / Z;

      depth[i] = Z;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_double_intermodel_projection(Rox_Point2D_Double res, const Rox_Array2D_Double homography, const Rox_Point3D_Double model, const Rox_Sint nbpts)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !model || !homography)
   { error =  ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)

   Rox_Double ** H = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &H, homography);

   for (Rox_Sint i = 0; i < nbpts; i++)
   {
      Rox_Double uw = H[0][0] * model[i].X + H[0][1] * model[i].Y + H[0][2];
      Rox_Double vw = H[1][0] * model[i].X + H[1][1] * model[i].Y + H[1][2];
      Rox_Double w  = H[2][0] * model[i].X + H[2][1] * model[i].Y + H[2][2];

      if (fabs(w) < DBL_EPSILON) w = DBL_EPSILON;

      res[i].u = uw / w;
      res[i].v = vw / w;
  }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_float_intermodel_projection(Rox_Point2D_Float res, const Rox_Array2D_Double homography, const Rox_Point3D_Double model, const Rox_Sint nbpts)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !model || !homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(homography, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** H = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &H, homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint i = 0; i < nbpts; i++)
   {
      Rox_Double uw = H[0][0] * model[i].X + H[0][1] * model[i].Y + H[0][2];
      Rox_Double vw = H[1][0] * model[i].X + H[1][1] * model[i].Y + H[1][2];
      Rox_Double w  = H[2][0] * model[i].X + H[2][1] * model[i].Y + H[2][2];

      if (fabs(w) < DBL_EPSILON) w = DBL_EPSILON;

      res[i].u = (Rox_Float) (uw / w);
      res[i].v = (Rox_Float) (vw / w);
   }

function_terminate:
   return error;
}
