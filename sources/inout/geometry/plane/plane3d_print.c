//==============================================================================
//
//    OPENROX   : File plane3d_print.c
//
//    Contents  : Implementation of plane3d print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <stdio.h>
#include "plane3d_print.h"
#include <baseproc/geometry/plane/plane_struct.h>

#include <inout/system/print.h>
#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_plane3d_print (
   const Rox_Plane3D_Double plane3d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !plane3d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("plane3D = [%16.16f; %16.16f; %16.16f; %16.16f]\n", plane3d->a, plane3d->b, plane3d->c, plane3d->d);

function_terminate:
   return error;
}
