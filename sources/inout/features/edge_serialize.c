//============================================================================
//
//    OPENROX   : File edge_serialize.c
//
//    Contents  : Implementation of edge serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "edge_serialize.h"

#include <inout/numeric/array2d_serialize.h>
#include <inout/numeric/dynvec_serialize.h>
#include <inout/numeric/objset_dynvec_serialize.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edgepreproc_gray_serialize(FILE * out, Rox_EdgePreproc_Gray input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint write_res = 0;

   // Tests IO
   if (!out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Write numeric data
   write_res = (Rox_Sint) fwrite(&(input->width), sizeof(Rox_Uint), 1, out);
   if (!write_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   write_res = (Rox_Sint) fwrite(&(input->height), sizeof(Rox_Uint), 1, out);
   if (!write_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Write struct data
   error = rox_array2d_float_serialize(out, input->hkernel);
   error = rox_array2d_float_serialize(out, input->vkernel);
   // only created and used when processing, don't save/load
   // error = rox_array2d_uchar_serialize(out, input->filtered);
   error = rox_array2d_point2d_sshort_serialize(out, input->gradients);

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgepreproc_gray_deserialize(Rox_EdgePreproc_Gray output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint read_res = 0;
   Rox_Uint owidth, oheight;

   // Tests IO
   if (!in)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //Write numeric data
   read_res = (Rox_Sint) fread(&owidth, sizeof(Rox_Uint), 1, in);
   if (!read_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   read_res = (Rox_Sint) fread(&oheight, sizeof(Rox_Uint), 1, in);
   if (!read_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //test if reading the same type of object
   if (owidth != output->width || oheight != output->height)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //Write struct data
   error = rox_array2d_float_deserialize(output->hkernel, in);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_float_deserialize(output->vkernel, in);
   ROX_ERROR_CHECK_TERMINATE ( error );
   //only created and used when processing, don't save/load
   //error = rox_array2d_uchar_serialize(out, input->filtered);
   error = rox_array2d_point2d_sshort_deserialize(output->gradients, in);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgedraw_serialize(FILE* out, Rox_EdgeDraw input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint write_res = 0;

   // Tests IO
   if (!out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Write numeric data
   write_res = (Rox_Sint) fwrite(&(input->width), sizeof(Rox_Uint), 1, out);
   if (!write_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   write_res = (Rox_Sint) fwrite(&(input->height), sizeof(Rox_Uint), 1, out);
   if (!write_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   write_res = (Rox_Sint) fwrite(&(input->anchorthresh), sizeof(Rox_Sshort), 1, out);
   if (!write_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   write_res = (Rox_Sint) fwrite(&(input->min_gradient), sizeof(Rox_Sshort), 1, out);
   if (!write_res)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Write struct data
   error = rox_array2d_sshort_serialize(out, input->gnorm);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_uchar_serialize(out, input->gori);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_edgel_serialize(out, input->anchors);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_edgeturn_serialize(out, input->stack);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_uchar_serialize(out, input->resultmask);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_objset_dynvec_edgel_serialize(out, input->resultsegments);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}

Rox_ErrorCode rox_edgedraw_deserialize(Rox_EdgeDraw output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint read_res = 0;
   Rox_Uint owidth, oheight;
   Rox_Sshort oanchorthresh, omin_gradient;
   //Tests IO
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //Write numeric data
   read_res = (Rox_Sint) fread(&owidth, sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   read_res = (Rox_Sint) fread(&oheight, sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   read_res = (Rox_Sint) fread(&oanchorthresh, sizeof(Rox_Sshort), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   read_res = (Rox_Sint) fread(&omin_gradient, sizeof(Rox_Sshort), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //test if reading the same type of object
   if(   owidth != output->width
      || oheight != output->height
      || oanchorthresh != output->anchorthresh
      || omin_gradient != output->min_gradient
      )
   {
      {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   //Write struct data
   error = rox_array2d_sshort_deserialize(output->gnorm, in);
   error = rox_array2d_uchar_deserialize(output->gori, in);
   error = rox_dynvec_edgel_deserialize(output->anchors, in);
   error = rox_dynvec_edgeturn_deserialize(output->stack, in);
   error = rox_array2d_uchar_deserialize(output->resultmask, in);
   error = rox_objset_dynvec_edgel_deserialize(output->resultsegments, in);

 function_terminate:
  return error;
}

Rox_ErrorCode rox_edgepostproc_ac_serialize(FILE* out, Rox_EdgePostproc_Ac input)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
  Rox_Sint write_res = 0;
   //Tests IO
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //Write numeric data
   write_res = (Rox_Sint) fwrite(&(input->width), sizeof(Rox_Uint), 1, out);
   if(!write_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   write_res = (Rox_Sint) fwrite(&(input->height), sizeof(Rox_Uint), 1, out);
   if(!write_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //Write struct data
   error = rox_dynvec_segment_part_serialize(out, input->validation_stack);
   error = rox_objset_dynvec_edgel_serialize(out, input->resultsegments);

 function_terminate:
  return error;
}
Rox_ErrorCode rox_edgepostproc_ac_deserialize(Rox_EdgePostproc_Ac output, FILE* in)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
  Rox_Sint read_res = 0;
   Rox_Uint owidth, oheight;
   //Tests IO
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //Write numeric data
   read_res = (Rox_Sint) fread(&owidth, sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}
   read_res = (Rox_Sint) fread(&oheight, sizeof(Rox_Uint), 1, in);
   if(!read_res){error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //test if reading the same type of object
   if(   owidth != output->width
      || oheight != output->height
      )
   {
      {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   //Write struct data
   error = rox_dynvec_segment_part_deserialize(output->validation_stack, in);
   error = rox_objset_dynvec_edgel_deserialize(output->resultsegments, in);

 function_terminate:
  return error;
}
