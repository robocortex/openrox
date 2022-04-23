//==============================================================================
//
//    OPENROX   : File objset_dynvec_serialize.c
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

#include "objset_dynvec_serialize.h"

#include <inout/numeric/dynvec_serialize.h>

#include <generated/objset_dynvec_point2d_sshort_struct.h>
#include <generated/objset_dynvec_fpsm_template_struct.h>
#include <generated/objset_dynvec_sint.h>
#include <generated/objset_dynvec_sint_struct.h>
#include <generated/objset_dynvec_rect_sint.h>
#include <generated/objset_dynvec_rect_sint_struct.h>

#include <generated/objset_dynvec_edgel_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_objset_dynvec_point2d_sshort_serialize(FILE* out, Rox_ObjSet_DynVec_Point2D_Sshort input)
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
      if (!input->data[id]) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
      error = rox_dynvec_point2d_sshort_serialize(out, input->data[id]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_dynvec_point2d_sshort_deserialize(Rox_ObjSet_DynVec_Point2D_Sshort output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint oused;
   Rox_Uint id;
   Rox_DynVec_Point2D_Sshort data;
   size_t read_res = 0;

   //handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}   

   //read numeric members
   read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   //read struct members
   for (id = 0; id < oused; id++)
   {
      error = rox_dynvec_point2d_sshort_new(&data, 10);
      error = rox_dynvec_point2d_sshort_deserialize(data, in);
      //rox_objset_dynvec_point2d_sshort_append will handle the pointer
      error = rox_objset_dynvec_point2d_sshort_append(output, data);
   }

 function_terminate:
  return error;
}

Rox_ErrorCode rox_objset_dynvec_sint_serialize(FILE* out, Rox_ObjSet_DynVec_Sint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint id;
   size_t write_res = 0;

   //handle file opening
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   //write numeric members
   write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   //write struct members
   for (id = 0; id < input->used; id++)
   {
      if (!input->data[id]) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
      error = rox_dynvec_sint_serialize(out, input->data[id]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_dynvec_sint_deserialize(Rox_ObjSet_DynVec_Sint output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint oused;
   Rox_Uint id;
   Rox_DynVec_Sint data;
   size_t read_res = 0;

   //handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}   

   //read numeric members
   read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   //read struct members
   for (id = 0; id < oused; id++)
   {
      error = rox_dynvec_sint_new(&data, 10);
      error = rox_dynvec_sint_deserialize(data, in);
      //rox_objset_dynvec_sint_append will handle the pointer
      error = rox_objset_dynvec_sint_append(output, data);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_dynvec_rect_sint_serialize(FILE* out, Rox_ObjSet_DynVec_Rect_Sint input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   size_t write_res = 0;

   //handle file opening
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   //write numeric members
   write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   //write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      if (!input->data[id]) 
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

      error = rox_dynvec_rect_sint_serialize(out, input->data[id]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_dynvec_rect_sint_deserialize(Rox_ObjSet_DynVec_Rect_Sint output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint oused;
   size_t read_res = 0;

   // Handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Read numeric members
   read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_DynVec_Rect_Sint data;

      error = rox_dynvec_rect_sint_new(&data, 10);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_dynvec_rect_sint_deserialize(data, in);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //rox_objset_dynvec_rect_int_append will handle the pointer
      error = rox_objset_dynvec_rect_sint_append(output, data);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_dynvec_edgel_serialize(FILE* out, Rox_ObjSet_DynVec_Edgel input)
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
      if (!input->data[id]) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
      error = rox_dynvec_edgel_serialize(out, input->data[id]);
   }

function_terminate:
   return error;
}
Rox_ErrorCode rox_objset_dynvec_edgel_deserialize(Rox_ObjSet_DynVec_Edgel output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}   

   //read numeric members
   Rox_Uint oused = 0;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_DynVec_Edgel data;

      error = rox_dynvec_edgel_new(&data, 10);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_dynvec_edgel_deserialize(data, in);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //rox_objset_dynvec_edgel_append will handle the pointer
      error = rox_objset_dynvec_edgel_append(output, data);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_dynvec_fpsm_template_serialize(FILE* out, Rox_ObjSet_DynVec_Fpsm_Template input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Handle file opening
   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      if (!input->data[id]) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
      error = rox_dynvec_fpsm_template_serialize(out, input->data[id]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_dynvec_fpsm_template_deserialize(Rox_ObjSet_DynVec_Fpsm_Template output, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Handle file opening
   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}   

   // Read numeric members
   Rox_Uint oused;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_DynVec_Fpsm_Template data;
      error = rox_dynvec_fpsm_template_new(&data, 10);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_dynvec_fpsm_template_deserialize(data, in);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //rox_objset_dynvec_edgel_append will handle the pointer
      error = rox_objset_dynvec_fpsm_template_append(output, data);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}


