//============================================================================
//
//    OPENROX   : File sdwm_serialize.c
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

#include "sdwm_serialize.h"

#include <inout/features/pyramid_npot_serialize.h>
#include <inout/features/fpsm_serialize.h>
#include <inout/numeric/objset_serialize.h>
#include <inout/numeric/objset_dynvec_serialize.h>
#include <inout/numeric/dynvec_serialize.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_sdwm_serialize(FILE* out, Rox_Sdwm input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint write_res = 0;

   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->nb_levels), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->scale_per_level), sizeof(Rox_Double), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->width_model), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->height_model), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->width_current), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->height_current), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_pyramid_npot_uchar_serialize(out, input->pyramid);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_sdwm_object_serialize(out, input->objects);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_dynvec_rect_sint_serialize(out, input->results);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_sint_serialize(out, input->results_indices);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_sint_serialize(out, input->results_scores);
   ROX_ERROR_CHECK_TERMINATE ( error );


   write_res = (Rox_Sint) fwrite(&(input->sum_edges), sizeof(Rox_Ulint), 1, out);
   if(!write_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->count_templates), sizeof(Rox_Uint), 1, out);
   if(!write_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_fpsm_serialize(out, input->fpsm_model);
   for ( Rox_Uint idlevel = 0; idlevel < input->nb_levels; idlevel++)
   {
      error = rox_fpsm_serialize(out, input->fpsm_currents[idlevel]);

    ROX_ERROR_CHECK_TERMINATE ( error );
  }
   error = rox_fpsm_index_serialize(out, input->index);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_sdwn_deserialize(Rox_Sdwm output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint read_res = 0;
   Rox_Uint onb_levels, owidth_m, oheight_m, owidth_c, oheight_c;
   Rox_Double oscale_per_level;

   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&onb_levels, sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&oscale_per_level, sizeof(Rox_Double), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&owidth_m, sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&oheight_m, sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&owidth_c, sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&oheight_c, sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //test if reading the same type of object
   //TODO float comparison
   if (  onb_levels != output->nb_levels     || oscale_per_level != output->scale_per_level
      || owidth_m != output->width_model      || oheight_m != output->height_model
      || owidth_c != output->width_current   || oheight_c != output->height_current          )
   {
      error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_pyramid_npot_uchar_deserialize(output->pyramid, in);

   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_sdwm_object_deserialize(output->objects, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_dynvec_rect_sint_deserialize(output->results, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_sint_deserialize(output->results_indices, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_sint_deserialize(output->results_scores, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   read_res = (Rox_Sint) fread(&(output->sum_edges), sizeof(Rox_Ulint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&(output->count_templates), sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_fpsm_deserialize(output->fpsm_model, in);
   for (Rox_Uint idlevel = 0; idlevel < output->nb_levels; idlevel++)
   {
      error = rox_fpsm_deserialize(output->fpsm_currents[idlevel], in);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   //this object has a special treatment as it is initialized in rox_sdwm_new(), reset it before deserialize it
   rox_objset_dynvec_fpsm_template_reset(output->index->index);
   error = rox_fpsm_index_deserialize(output->index, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
