//==============================================================================
//
//    OPENROX   : File pointstransformproject.c
//
//    Contents  : Implementation of pointstransformproject module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point2d_projection_from_point3d_transform.h"

#include <float.h>
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_point3d_float_transform_project (
   Rox_Point2D_Float res,
   Rox_MatSE3 pose,
   Rox_MatUT3 calib,
   Rox_Point3D_Float input,
   Rox_Uint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K_data = NULL;
   error = rox_matut3_get_data_pointer_to_pointer ( &K_data, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** T_data = NULL;
   error = rox_matse3_get_data_pointer_to_pointer ( &T_data, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double fu = K_data[0][0];
   Rox_Double fv = K_data[1][1];
   Rox_Double cu = K_data[0][2];
   Rox_Double cv = K_data[1][2];

   for (Rox_Uint i = 0; i < count; i++)
   {
      Rox_Float X = (Rox_Float) ( T_data[0][0] * input[i].X + T_data[0][1] * input[i].Y + T_data[0][2] * input[i].Z + T_data[0][3] );
      Rox_Float Y = (Rox_Float) ( T_data[1][0] * input[i].X + T_data[1][1] * input[i].Y + T_data[1][2] * input[i].Z + T_data[1][3] );
      Rox_Float Z = (Rox_Float) ( T_data[2][0] * input[i].X + T_data[2][1] * input[i].Y + T_data[2][2] * input[i].Z + T_data[2][3] );

      if (fabsf(Z) < DBL_EPSILON) Z = FLT_EPSILON;
      Rox_Float x = X / Z;
      Rox_Float y = Y / Z;

      res[i].u = (Rox_Float) ( fu * x + cu );
      res[i].v = (Rox_Float) ( fv * y + cv );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_double_transform_project (
   Rox_Point2D_Double res,
   const Rox_MatSE3 pose,
   const Rox_MatUT3 calib,
   const Rox_Point3D_Double input,
   const Rox_Uint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!calib)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_matut3_check_size ( calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** K_data = NULL;
   error = rox_matut3_get_data_pointer_to_pointer ( &K_data, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** T_data = NULL;
   error = rox_matse3_get_data_pointer_to_pointer ( &T_data, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double fu = K_data[0][0];
   Rox_Double fv = K_data[1][1];
   Rox_Double cu = K_data[0][2];
   Rox_Double cv = K_data[1][2];

   for ( Rox_Sint k = 0; k < (Rox_Sint) count; k++)
   {
      Rox_Double X = ( T_data[0][0] * input[k].X + T_data[0][1] * input[k].Y + T_data[0][2] * input[k].Z + T_data[0][3] );
      Rox_Double Y = ( T_data[1][0] * input[k].X + T_data[1][1] * input[k].Y + T_data[1][2] * input[k].Z + T_data[1][3] );
      Rox_Double Z = ( T_data[2][0] * input[k].X + T_data[2][1] * input[k].Y + T_data[2][2] * input[k].Z + T_data[2][3] );

      if (fabs(Z) < DBL_EPSILON) Z = DBL_EPSILON;
      Rox_Double x = X / Z;
      Rox_Double y = Y / Z;

      res[k].u = fu * x + cu;
      res[k].v = fv * y + cv;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_float_transform_project_meters (
   Rox_Point2D_Float res,
   Rox_MatSE3 pose,
   Rox_Point3D_Float input,
   Rox_Uint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** T_data = NULL;
   error = rox_matse3_get_data_pointer_to_pointer ( &T_data, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < count; i++)
   {
      Rox_Float X = (Rox_Float)( T_data[0][0] * input[i].X + T_data[0][1] * input[i].Y + T_data[0][2] * input[i].Z + T_data[0][3] );
      Rox_Float Y = (Rox_Float)( T_data[1][0] * input[i].X + T_data[1][1] * input[i].Y + T_data[1][2] * input[i].Z + T_data[1][3] );
      Rox_Float Z = (Rox_Float)( T_data[2][0] * input[i].X + T_data[2][1] * input[i].Y + T_data[2][2] * input[i].Z + T_data[2][3] );

      if (fabsf(Z) < DBL_EPSILON) Z = DBL_EPSILON;
      Rox_Float iZ = (Rox_Float)1.0 / Z;
      Rox_Float x = X * iZ;
      Rox_Float y = Y * iZ;

      res[i].u = x;
      res[i].v = y;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_double_transform_project_meters (
   Rox_Point2D_Double res,
   Rox_MatSE3 pose,
   Rox_Point3D_Double input,
   Rox_Uint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** T_data = NULL;
   error = rox_matse3_get_data_pointer_to_pointer( &T_data, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < count; i++)
   {
      Rox_Double X = ( T_data[0][0] * input[i].X + T_data[0][1] * input[i].Y + T_data[0][2] * input[i].Z + T_data[0][3] );
      Rox_Double Y = ( T_data[1][0] * input[i].X + T_data[1][1] * input[i].Y + T_data[1][2] * input[i].Z + T_data[1][3] );
      Rox_Double Z = ( T_data[2][0] * input[i].X + T_data[2][1] * input[i].Y + T_data[2][2] * input[i].Z + T_data[2][3] );

      if (fabs(Z) < DBL_EPSILON) Z = DBL_EPSILON;
      Rox_Double iZ = 1.0 / Z;
      Rox_Double x = X * iZ;
      Rox_Double y = Y * iZ;

      res[i].u = x;
      res[i].v = y;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_float_transform_project_homography (
   Rox_Point2D_Float res,
   Rox_MatSL3 homography,
   Rox_Point3D_Float input,
   Rox_Uint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_check_size ( homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** H = NULL;
   error = rox_matsl3_get_data_pointer_to_pointer( &H, homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < count; i++)
   {
      Rox_Float X = (Rox_Float) ( H[0][0] * input[i].X + H[0][1] * input[i].Y + H[0][2] );
      Rox_Float Y = (Rox_Float) ( H[1][0] * input[i].X + H[1][1] * input[i].Y + H[1][2] );
      Rox_Float Z = (Rox_Float) ( H[2][0] * input[i].X + H[2][1] * input[i].Y + H[2][2] );

      if (fabsf(Z) < FLT_EPSILON) Z = FLT_EPSILON;
      Rox_Float iZ = (Rox_Float) (1.0 / Z);
      Rox_Float x = X * iZ;
      Rox_Float y = Y * iZ;

      res[i].u = x;
      res[i].v = y;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_double_transform_project_homography (
   Rox_Point2D_Double res,
   Rox_MatSL3 homography,
   Rox_Point3D_Double input,
   Rox_Uint count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!res)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_check_size ( homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** H_data = NULL;
   error = rox_matsl3_get_data_pointer_to_pointer( &H_data, homography );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < count; i++)
   {
      Rox_Double X = ( H_data[0][0] * input[i].X + H_data[0][1] * input[i].Y + H_data[0][2] );
      Rox_Double Y = ( H_data[1][0] * input[i].X + H_data[1][1] * input[i].Y + H_data[1][2] );
      Rox_Double Z = ( H_data[2][0] * input[i].X + H_data[2][1] * input[i].Y + H_data[2][2] );

      if (fabs(Z) < DBL_EPSILON) Z = DBL_EPSILON;
      Rox_Double iZ = 1.0 / Z;
      Rox_Double x = X * iZ;
      Rox_Double y = Y * iZ;

      res[i].u = x;
      res[i].v = y;
   }

function_terminate:
   return error;
}
