//============================================================================
//
//    OPENROX   : File fpsm_serialize.c
//
//    Contents  : Implementation of fpsm serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "fpsm_serialize.h"

#include <core/features/descriptors/fpsm/fpsm_struct.h>

#include <inout/numeric/objset_array2d_serialize.h>
#include <inout/numeric/array2d_serialize.h>
#include <inout/numeric/objset_dynvec_serialize.h>
#include <inout/numeric/dynvec_serialize.h>
#include <inout/features/edge_serialize.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_fpsm_serialize(FILE* out, Rox_Fpsm input)
{
   Rox_Sint write_res = 0;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   //tests io
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //write numeric data
   write_res = (Rox_Sint) fwrite(&(input->width), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->height), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->nbr_ref_points), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->count_edgels), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
      
   //write struct data   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_edgepreproc_gray_serialize(out, input->preproc);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
  error = rox_edgedraw_serialize(out, input->drawing);
    ROX_ERROR_CHECK_TERMINATE ( error );

  error = rox_edgepostproc_ac_serialize(out, input->postproc);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_serialize(out, input->edges);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
  error = rox_array2d_sshort_serialize(out, input->angles);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_sshort_serialize(out, input->anglemap);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_sshort_serialize(out, input->distancemap);
    ROX_ERROR_CHECK_TERMINATE ( error );

  error = rox_array2d_point2d_sshort_serialize(out, input->distancemap_points);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_fpsm_deserialize(Rox_Fpsm output, FILE* in)
{
   Rox_Sint read_res = 0;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint owidth, oheight, onbr_ref_points;
   //tests io
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //read numeric data
   read_res = (Rox_Sint) fread(&owidth, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&oheight, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&onbr_ref_points, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //test if reading the same type of object
   if (owidth != output->width || oheight != output->height || onbr_ref_points != output->nbr_ref_points)
   {
      {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   read_res = (Rox_Sint) fread(&output->count_edgels, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //read struct data
   error = rox_edgepreproc_gray_deserialize(output->preproc, in);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
  error = rox_edgedraw_deserialize(output->drawing, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_edgepostproc_ac_deserialize(output->postproc, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_deserialize(output->edges, in);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
  error = rox_array2d_sshort_deserialize(output->angles, in);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
  error = rox_array2d_sshort_deserialize(output->anglemap, in);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
  error = rox_array2d_sshort_deserialize(output->distancemap, in);
    ROX_ERROR_CHECK_TERMINATE ( error );

  error = rox_array2d_point2d_sshort_deserialize(output->distancemap_points, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_fpsm_index_serialize(FILE* out, Rox_Fpsm_Index input)
{
   Rox_Sint write_res = 0;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   //tests io
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //write numeric data
   write_res = (Rox_Sint) fwrite(&(input->nd), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->ntheta), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->m), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->width), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->height), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->min_votes), sizeof(Rox_Uint), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   write_res = (Rox_Sint) fwrite(&(input->maxdist), sizeof(Rox_Double), 1, out);
   if(!write_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //write struct data
   error = rox_objset_dynvec_fpsm_template_serialize(out, input->index);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_objset_dynvec_sint_serialize(out, input->counters);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_fpsm_template_serialize(out, input->results);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_fpsm_index_deserialize(Rox_Fpsm_Index output, FILE* in)
{
   Rox_Sint read_res = 0;
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint owidth, oheight, ond, ontheta, om, omin_votes;
   //tests io
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //read numeric data
   read_res = (Rox_Sint) fread(&ond, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&ontheta, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&om, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&owidth, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&oheight, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   read_res = (Rox_Sint) fread(&omin_votes, sizeof(Rox_Uint), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //test if reading the same type of object
   if(   owidth != output->width
      || oheight != output->height
      || ond != output->nd
      || ontheta != output->ntheta
      || om != output->m
      || omin_votes != output->min_votes
      )
   {
      {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   read_res = (Rox_Sint) fread(&output->maxdist, sizeof(Rox_Double), 1, in);
   if(!read_res) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //read struct data
   error = rox_objset_dynvec_fpsm_template_deserialize(output->index, in);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_objset_dynvec_sint_deserialize(output->counters, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_fpsm_template_deserialize(output->results, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

