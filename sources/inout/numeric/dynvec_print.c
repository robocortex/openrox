//==============================================================================
//
//    OPENROX   : File dynvec_print.c
//
//    Contents  : Implementation of sdynvec display module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_print.h"

#include <generated/dynvec_float_struct.h>
#include <generated/dynvec_double_struct.h>
#include <generated/dynvec_uint_struct.h>

#include <system/errors/errors.h>

#include <inout/system/errors_print.h>
#include "inout/system/print.h"

Rox_ErrorCode rox_dynvec_float_print(Rox_DynVec_Float dynvec)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!dynvec) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("Dynvec (%lu):\r\n", (long unsigned int) dynvec->used);
   for (Rox_Uint k = 0; k < dynvec->used; k++)
   {
      rox_log("%.16f ", dynvec->data[k]);
   }
   rox_log("\n");

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_double_print(Rox_DynVec_Double dynvec)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!dynvec) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("Dynvec (%lu):\r\n", (long unsigned int) dynvec->used);
   for (Rox_Uint k = 0; k < dynvec->used; k++)
   {
      rox_log("%.16f ", dynvec->data[k]);
   }
   rox_log("\n");

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_uint_print(Rox_DynVec_Uint dynvec)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!dynvec) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("Dynvec (%lu):\r\n", (long unsigned int) dynvec->used);
   for (Rox_Uint k = 0; k < dynvec->used; k++)
   {
      rox_log("%d ", dynvec->data[k]);
   }
   rox_log("\n");

function_terminate:
   return error;
}
