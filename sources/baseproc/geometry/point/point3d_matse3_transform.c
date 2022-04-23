//==============================================================================
//
//    OPENROX   : File point3d_matse3_transform.c
//
//    Contents  : API of point3d matse3 transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "point3d_matse3_transform.h"

#include <float.h>
#include <baseproc/geometry/point/point3d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_point3d_double_transform (
   Rox_Point3D_Double   res,
   Rox_MatSE3           pose,
   Rox_Point3D_Double   input,
   Rox_Uint             count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( NULL == pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( NULL == res || NULL == input )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_matse3_get_data_pointer_to_pointer(&dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < count; i++)
   {
      Rox_Double X = input[i].X;
      Rox_Double Y = input[i].Y;
      Rox_Double Z = input[i].Z;
      res[i].X = dt[0][0]*X + dt[0][1]*Y + dt[0][2]*Z + dt[0][3];
      res[i].Y = dt[1][0]*X + dt[1][1]*Y + dt[1][2]*Z + dt[1][3];
      res[i].Z = dt[2][0]*X + dt[2][1]*Y + dt[2][2]*Z + dt[2][3];
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_point3d_float_transform (
   Rox_Point3D_Float    res,
   Rox_MatSE3           pose,
   Rox_Point3D_Float    input,
   Rox_Uint             count
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( NULL == pose)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( NULL == res || NULL == input )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dt = NULL;
   error = rox_matse3_get_data_pointer_to_pointer ( &dt, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i = 0; i < count; i++)
   {
      Rox_Float X = input[i].X;
      Rox_Float Y = input[i].Y;
      Rox_Float Z = input[i].Z;
      res[i].X = (Rox_Float) (dt[0][0]*X + dt[0][1]*Y + dt[0][2]*Z + dt[0][3]);
      res[i].Y = (Rox_Float) (dt[1][0]*X + dt[1][1]*Y + dt[1][2]*Z + dt[1][3]);
      res[i].Z = (Rox_Float) (dt[2][0]*X + dt[2][1]*Y + dt[2][2]*Z + dt[2][3]);
   }

function_terminate:
   return error;
}
