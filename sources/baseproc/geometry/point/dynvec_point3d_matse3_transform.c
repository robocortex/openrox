//==============================================================================
//
//    OPENROX   : File dynvec_point3d_matse3_transform.c
//
//    Contents  : Implementation of dynvec point3d matse3 transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_point3d_matse3_transform.h"

#include <float.h>

#include <generated/dynvec_point3d_double_struct.h>
#include <generated/dynvec_point3d_float_struct.h>

#include <baseproc/geometry/point/point3d_matse3_transform.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_dynvec_point3d_double_transform ( 
   Rox_DynVec_Point3D_Double  res,
   Rox_MatSE3                 pose,
   Rox_DynVec_Point3D_Double  input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose || !res || !input )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_point3d_double_transform ( res->data, pose, input->data, input->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_float_transform ( 
   Rox_DynVec_Point3D_Float   res,
   Rox_MatSE3                 pose,
   Rox_DynVec_Point3D_Float   input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !pose || !res || !input )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_point3d_float_transform ( res->data, pose, input->data, input->used );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
