//==============================================================================
//
//    OPENROX   : File dynvec_point2d_save.c
//
//    Contents  : Implementation of dynvec point2d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_point2d_save.h"
#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point2d_double_struct.h>

#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_dynvec_point2d_float_save(const Rox_Char * filename, const Rox_DynVec_Point2D_Float source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!source || !filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   out = fopen(filename, "w");
   if (!out) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_float_save_stream(out, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if(out) fclose(out);
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_float_save_stream(FILE * output, Rox_DynVec_Point2D_Float source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!source || !output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint id = 0; id < source->used; id++)
   {
      fprintf(output, "%f %f\n", source->data[id].u, source->data[id].v);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_double_save(const Rox_Char * filename, const Rox_DynVec_Point2D_Double source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!source || !filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   out = fopen(filename, "w");
   if (!out) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_double_save_stream(out, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  if(out) fclose(out);
  return error;
}

Rox_ErrorCode rox_dynvec_point2d_double_save_stream(FILE * output, Rox_DynVec_Point2D_Double source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!source || !output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint id = 0; id < source->used; id++)
   {
      fprintf(output, "%32.32f %32.32f\n", source->data[id].u, source->data[id].v);
   }

function_terminate:
   return error;
}
