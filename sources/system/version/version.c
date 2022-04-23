//==============================================================================
//
//    OPENROX   : File version.c
//
//    Contents  : Implementation of version module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "version.h"

#include <stdlib.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_get_version(Rox_Uint * major, Rox_Uint * minor, Rox_Uint * patch)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!major || !minor || !patch)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *major = atoi(OPENROX_MAJOR_VERSION);
   *minor = atoi(OPENROX_MINOR_VERSION);
   *patch = atoi(OPENROX_PATCH_VERSION);

function_terminate:
   return error;
}
