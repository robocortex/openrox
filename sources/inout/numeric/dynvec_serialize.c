//==============================================================================
//
//    OPENROX   : File dynvec_serialize.c
//
//    Contents  : Implementation
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_serialize.h"

#include <generated/dynvec_sint_struct.h>
#include <generated/dynvec_edgel_struct.h>
#include <generated/dynvec_edgeturn_struct.h>
#include <generated/dynvec_segment_part_struct.h>
#include <generated/dynvec_point2d_sshort_struct.h>
#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>
#include <generated/dynvec_rect_sint_struct.h>
#include <generated/dynvec_fpsm_feature_struct.h>
#include <generated/dynvec_fpsm_template_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_dynvec_sint_serialize(FILE * out, Rox_DynVec_Sint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   size_t write_res = 0;

   // Handle file opening
   if (!out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Write numeric members
   write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Write struct members
   write_res = fwrite(input->data, sizeof(Rox_Sint), input->used, out);
   if(write_res != input->used)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_sint_deserialize(Rox_DynVec_Sint output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //read numeric members
   Rox_Uint oused;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Sint data;
      read_res = fread(&data, sizeof(Rox_Sint), 1, in);
      if(read_res != 1)
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      CHECK_ERROR_RETURN(rox_dynvec_sint_append(output, &data))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_edgel_serialize(FILE* out, Rox_DynVec_Edgel input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // handle file opening
   if (!out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      write_res = fwrite(&(input->data[id].u), sizeof(Rox_Uint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      write_res = fwrite(&(input->data[id].v), sizeof(Rox_Uint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      write_res = fwrite(&(input->data[id].score), sizeof(Rox_Uint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_edgel_deserialize(Rox_DynVec_Edgel output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //read numeric members
   Rox_Uint oused;
   Rox_Sint read_res = (Rox_Sint) fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //read struct members
   for ( Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Edgel_Struct data;
      read_res = (Rox_Sint) fread(&(data.u), sizeof(Rox_Uint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      read_res = (Rox_Sint) fread(&(data.v), sizeof(Rox_Uint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      read_res = (Rox_Sint) fread(&(data.score), sizeof(Rox_Uint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      //rox_dynvec_edgel_append will copy data members
      CHECK_ERROR_RETURN(rox_dynvec_edgel_append(output, &data))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_edgeturn_serialize(FILE* out, Rox_DynVec_EdgeTurn input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // handle file opening
   if (!out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      write_res = fwrite(&(input->data[id].u), sizeof(Rox_Uint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      write_res = fwrite(&(input->data[id].v), sizeof(Rox_Uint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      write_res = fwrite(&(input->data[id].direction), sizeof(Rox_Uint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_edgeturn_deserialize(Rox_DynVec_EdgeTurn output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!output)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //read numeric members
   Rox_Uint oused;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_EdgeTurn_Struct data;
      read_res = fread(&(data.u), sizeof(Rox_Uint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      read_res = fread(&(data.v), sizeof(Rox_Uint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      read_res = fread(&(data.direction), sizeof(Rox_Uint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      //rox_dynvec_edgel_append will copy data members
      CHECK_ERROR_RETURN(rox_dynvec_edgeturn_append(output, &data))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_segment_part_serialize(FILE* out, Rox_DynVec_Segment_Part input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      write_res = fwrite(&(input->data[id].length), sizeof(Rox_Uint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      write_res = fwrite(input->data[id].data, sizeof(Rox_Edgel_Struct), input->data[id].length, out);
      if(write_res != input->data[id].length) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_segment_part_deserialize(Rox_DynVec_Segment_Part output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //read numeric members
   Rox_Uint oused;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Edgel_Struct* data;
      Rox_Uint length = 0;
      Rox_Segment_Part_Struct segment;

      read_res = fread(&length, sizeof(Rox_Uint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      data = (Rox_Edgel_Struct *)rox_memory_allocate(sizeof(Rox_Edgel_Struct), length);
      if (data == NULL) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

      read_res = fread(data, sizeof(Rox_Edgel_Struct), length, in);
      if (read_res != length) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      //rox_dynvec_segment_part_append will copy members and deallocate data when the time comes
      segment.data = data;
      segment.length = length;
      CHECK_ERROR_RETURN(rox_dynvec_segment_part_append(output, &segment))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_sshort_serialize(FILE* out, Rox_DynVec_Point2D_Sshort input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      write_res = fwrite(&(input->data[id].u), sizeof(Rox_Sshort), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      write_res = fwrite(&(input->data[id].v), sizeof(Rox_Sshort), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_sshort_deserialize(Rox_DynVec_Point2D_Sshort output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //read numeric members
   Rox_Uint oused = 0;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Point2D_Sshort_Struct data;
      read_res = fread(&(data.u), sizeof(Rox_Sshort), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      read_res = fread(&(data.v), sizeof(Rox_Sshort), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      //rox_dynvec_point2d_sshort_append will copy data members
      CHECK_ERROR_RETURN(rox_dynvec_point2d_sshort_append(output, &data))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_rect_sint_serialize(FILE* out, Rox_DynVec_Rect_Sint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   //write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   //write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      write_res = fwrite(&(input->data[id].x), sizeof(Rox_Sint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
      write_res = fwrite(&(input->data[id].y), sizeof(Rox_Sint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
      write_res = fwrite(&(input->data[id].width), sizeof(Rox_Sint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
      write_res = fwrite(&(input->data[id].height), sizeof(Rox_Sint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_rect_sint_deserialize(Rox_DynVec_Rect_Sint output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //read numeric members
   Rox_Uint oused = 0;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Rect_Sint_Struct data;

      read_res = fread(&(data.x), sizeof(Rox_Sint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      read_res = fread(&(data.y), sizeof(Rox_Sint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      read_res = fread(&(data.width), sizeof(Rox_Sint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      read_res = fread(&(data.height), sizeof(Rox_Sint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      //rox_dynvec_rect_int_append copies the struct members
      CHECK_ERROR_RETURN(rox_dynvec_rect_sint_append(output, &data))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_fpsm_feature_serialize(FILE* out, Rox_DynVec_Fpsm_Feature input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      write_res = fwrite(&(input->data[id].top), sizeof(Rox_Sint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      write_res = fwrite(&(input->data[id].left), sizeof(Rox_Sint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      write_res = fwrite(&(input->data[id].distances), sizeof(Rox_Double), 128, out);
      if(write_res != 128) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      write_res = fwrite(&(input->data[id].angles), sizeof(Rox_Double), 128, out);
      if(write_res != 128) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_fpsm_feature_deserialize(Rox_DynVec_Fpsm_Feature output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //read numeric members
   Rox_Uint oused;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Fpsm_Feature_Struct data;
      read_res = fread(&(data.top), sizeof(Rox_Sint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      read_res = fread(&(data.left), sizeof(Rox_Sint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      read_res = fread(&(data.distances), sizeof(Rox_Double), 128, in);
      if(read_res != 128) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      read_res = fread(&(data.angles), sizeof(Rox_Double), 128, in);
      if(read_res != 128) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      //rox_dynvec_fpsm_feature_append copies the data
      CHECK_ERROR_RETURN(rox_dynvec_fpsm_feature_append(output, &data))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_fpsm_template_serialize(FILE* out, Rox_DynVec_Fpsm_Template input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      write_res = fwrite(&(input->data[id].object_id), sizeof(Rox_Sint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      write_res = fwrite(&(input->data[id].view_id), sizeof(Rox_Sint), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      write_res = fwrite(&(input->data[id].angle), sizeof(Rox_Double), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      write_res = fwrite(&(input->data[id].dist), sizeof(Rox_Double), 1, out);
      if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_fpsm_template_deserialize(Rox_DynVec_Fpsm_Template output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //read numeric members
   Rox_Uint oused = 0;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Fpsm_Template_Struct data;

      read_res = fread(&(data.object_id), sizeof(Rox_Sint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      read_res = fread(&(data.view_id), sizeof(Rox_Sint), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      read_res = fread(&(data.angle), sizeof(Rox_Double), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}
      read_res = fread(&(data.dist), sizeof(Rox_Double), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

      //rox_dynvec_fpsm_template_append copies the data
      CHECK_ERROR_RETURN(rox_dynvec_fpsm_template_append(output, &data))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_serialize(FILE * out, Rox_DynVec_Point3D_Double input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!out)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if (!input)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      write_res = fwrite(&(input->data[id].X), sizeof(Rox_Double), 1, out);
      if(write_res != 1)
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      write_res = fwrite(&(input->data[id].Y), sizeof(Rox_Double), 1, out);
      if(write_res != 1)
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      write_res = fwrite(&(input->data[id].Z), sizeof(Rox_Double), 1, out);
      if(write_res != 1)
      { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point3d_double_deserialize(Rox_DynVec_Point3D_Double output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in)
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (!output)
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //read numeric members
   Rox_Uint oused = 0;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1)
      {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Point3D_Double_Struct data;
      read_res = fread(&(data.X), sizeof(Rox_Double), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

      read_res = fread(&(data.Y), sizeof(Rox_Double), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

      read_res = fread(&(data.Z), sizeof(Rox_Double), 1, in);
      if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

      //rox_dynvec_point3d_double_append will copy data members
      CHECK_ERROR_RETURN(rox_dynvec_point3d_double_append(output, &data))
   }

function_terminate:
   return error;
}
