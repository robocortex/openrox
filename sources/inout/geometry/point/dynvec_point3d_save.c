//==============================================================================
//
//    OPENROX   : File dynvec_point3d_save.c
//
//    Contents  : Implementation of dynvec point3d save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_point3d_save.h"
#include <generated/dynvec_point3d_float_struct.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <inout/numeric/dynvec_serialize.h>

#include <system/errors/errors.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_dynvec_point3d_double_save_binary(const Rox_Char *filename, Rox_DynVec_Point3D_Double source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!source || !filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   out = fopen(filename, "wb");
   if (!out) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point3d_double_serialize(out, source);

function_terminate:
   if(out) fclose(out);
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_load_binary ( Rox_DynVec_Point3D_Double source, Rox_Char *filename )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * inp = NULL;

   if (!source || !filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   inp = fopen(filename, "rb");
   if (!inp) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point3d_double_deserialize(source, inp);

function_terminate:
   if(inp) fclose(inp);
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_save(const Rox_Char * filename, const Rox_DynVec_Point3D_Double dynvec_point3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!dynvec_point3d || !filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   out = fopen(filename, "w");
   if (!out) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_dynvec_point3d_double_save_stream(out, dynvec_point3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if(out) fclose(out);
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_save_stream(FILE * output, Rox_DynVec_Point3D_Double source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!source || !output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint id = 0; id < source->used; id++)
   {
      fprintf(output, "%32.32f %32.32f %32.32f\n", source->data[id].X, source->data[id].Y, source->data[id].Z);
   }

function_terminate:
   return error;
}
