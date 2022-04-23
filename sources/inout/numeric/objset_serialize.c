//============================================================================
//
//    OPENROX   : File objset_serialize.c
//
//    Contents  : Implementation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "objset_serialize.h"
#include <generated/objset_sdwm_object_struct.h>

#include <inout/features/sdwm_object_serialize.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_dynvec_sdwm_object_serialize(FILE* out, Rox_ObjSet_Sdwm_Object input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Handle file opening
   if (!out) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   
   if (!input) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   // Write numeric members
   fwrite(&(input->used), sizeof(Rox_Uint), 1, out);

   // Write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      if (!input->data[id]) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

      fwrite(&(input->data[id]->width), sizeof(Rox_Uint), 1, out);
      fwrite(&(input->data[id]->height), sizeof(Rox_Uint), 1, out);

      error =rox_sdwm_object_serialize(out, input->data[id]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }  


function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_sdwm_object_deserialize(Rox_ObjSet_Sdwm_Object output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint oused, owidth, oheight;
   size_t read_res = 0;
   Rox_Sdwm_Object data;

   // Handle file opening
   if (!in) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   if (!output) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   // Read numeric members
   read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if (read_res != 1) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error); }

   // Read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      read_res = fread(&owidth, sizeof(Rox_Uint), 1, in);
      if ( read_res != 1 )
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error); }

      read_res = fread(&oheight, sizeof(Rox_Uint), 1, in);
      if ( read_res != 1 )
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error); }

      error = rox_sdwm_object_new(&data, owidth, oheight);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_sdwn_object_deserialize(data, in);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // rox_objset_sdwm_object_append handles the pointer
      error = rox_objset_sdwm_object_append(output, data);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
