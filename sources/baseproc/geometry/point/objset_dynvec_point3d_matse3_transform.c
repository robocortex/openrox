//==============================================================================
//
//    OPENROX   : File objset_dynvec_point3d_matse3_transform.c
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

#include "objset_dynvec_point3d_matse3_transform.h"
#include <float.h>

#include <generated/objset_dynvec_point3d_double_struct.h>
#include <generated/objset_dynvec_point3d_float_struct.h>
#include <generated/objset_matse3_struct.h>

#include <baseproc/geometry/point/dynvec_point3d_matse3_transform.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_objset_dynvec_point3d_double_transform (
   Rox_ObjSet_DynVec_Point3D_Double mo,
   Rox_ObjSet_MatSE3                oTb,
   Rox_ObjSet_DynVec_Point3D_Double mb
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !mo || !mb || !oTb )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint k=0; k<mb->used; k++)
   {
      error = rox_dynvec_point3d_double_transform ( mo->data[k], oTb->data[k], mb->data[k]);
      ROX_ERROR_CHECK_TERMINATE ( error ); 
   }

function_terminate:
   return error;
}