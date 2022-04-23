//==============================================================================
//
//    OPENROX   : File objset_dynvec_point3d_print.c
//
//    Contents  : Implementation of objset dynvec point3d display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "objset_dynvec_point3d_print.h"

#include <generated/objset_dynvec_point3d_float_struct.h>
#include <generated/objset_dynvec_point3d_double_struct.h>

#include "dynvec_point3d_print.h"

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#include <system/errors/errors.h>

Rox_ErrorCode rox_objset_dynvec_point3d_float_print(Rox_ObjSet_DynVec_Point3D_Float points3D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint k=0; k<points3D->used; k++)
   {
      rox_log("Display set %d of dynvec point 3d float\n", k);
      rox_dynvec_point3d_float_print(points3D->data[k]);
   }

function_terminate:
   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_objset_dynvec_point3d_double_print(Rox_ObjSet_DynVec_Point3D_Double points3D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !points3D )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint k=0; k<points3D->used; k++)
   {
      rox_log("Display set %d of dynvec point 3d double\n", k);
      rox_dynvec_point3d_double_print(points3D->data[k]);
   }

function_terminate:
   return ROX_ERROR_NONE;
}