//============================================================================
//
//    OPENROX   : File sdwm_object_serialize.c
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

#include "sdwm_object_serialize.h"

#include <inout/numeric/objset_array2d_serialize.h>
#include <inout/numeric/objset_dynvec_serialize.h>
#include <inout/numeric/dynvec_serialize.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_sdwm_object_serialize(FILE* out, Rox_Sdwm_Object input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint write_res = 0;

   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->width), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   write_res = (Rox_Sint) fwrite(&(input->height), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_objset_array2d_sshort_serialize(out, input->anglemaps);
   error = rox_objset_dynvec_point2d_sshort_serialize(out, input->pointsset);

function_terminate:
   return error;
}

ROX_API Rox_ErrorCode rox_sdwn_object_deserialize(Rox_Sdwm_Object output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint read_res = 0;
   Rox_Uint owidth, oheight;

   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&owidth, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   read_res = (Rox_Sint) fread(&oheight, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //test if reading the same type of object
   if (owidth != output->width || oheight != output->height)
   {
      error = ROX_ERROR_BAD_SIZE; 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_objset_array2d_sshort_deserialize(output->anglemaps, owidth, oheight, in);
   error = rox_objset_dynvec_point2d_sshort_deserialize(output->pointsset, in);

function_terminate:
   return error;
}
