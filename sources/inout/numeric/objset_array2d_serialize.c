//============================================================================
//
//    OPENROX   : File objset_array2d_serialize.c
//
//    Contents  : Implementation of array2d_serialize module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "objset_array2d_serialize.h"

#include <generated/objset_array2d_sshort_struct.h>
#include <generated/objset_array2d_uchar_struct.h>

#include <inout/numeric/array2d_serialize.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_objset_array2d_sshort_serialize(FILE* out, Rox_ObjSet_Array2D_Sshort input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      CHECK_ERROR_RETURN(rox_array2d_sshort_serialize(out, input->data[id]))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_array2d_sshort_serialize_binary(const char * filename, Rox_ObjSet_Array2D_Sshort input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   //handle file opening
   if (!filename) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   out = fopen(filename, "wb");
   if (!out) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //write numeric members
   CHECK_ERROR_TERMINATE(rox_objset_array2d_sshort_serialize(out, input))

function_terminate:
   if (out) fclose(out);
   return error;
}

Rox_ErrorCode rox_objset_array2d_sshort_deserialize(Rox_ObjSet_Array2D_Sshort output, Rox_Sint width, Rox_Sint height, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Read numeric members
   Rox_Uint oused;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   // Read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Array2D_Sshort data;
      CHECK_ERROR_RETURN(rox_array2d_sshort_new(&data, height, width))
      CHECK_ERROR_RETURN(rox_array2d_sshort_deserialize(data, in))
      CHECK_ERROR_RETURN(rox_objset_array2d_sshort_append(output,data))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_array2d_sshort_deserialize_binary(Rox_ObjSet_Array2D_Sshort output, Rox_Sint width, Rox_Sint height, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * in = NULL;

   // Handle file opening
   if (!filename) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   in = fopen(filename, "rb");
   if (!in) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   CHECK_ERROR_TERMINATE(rox_objset_array2d_sshort_deserialize(output, width, height, in))

function_terminate:
   if(in) fclose(in);

   return error;
}

Rox_ErrorCode rox_objset_array2d_uchar_serialize(FILE* out, Rox_ObjSet_Array2D_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!out) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Write numeric members
   size_t write_res = fwrite(&(input->used), sizeof(Rox_Uint), 1, out);
   if(write_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   // Write struct members
   for (Rox_Uint id = 0; id < input->used; id++)
   {
      CHECK_ERROR_RETURN(rox_array2d_uchar_serialize(out, input->data[id]))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_array2d_uchar_serialize_binary(const char * filename, Rox_ObjSet_Array2D_Uchar input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   //handle file opening
   if (!filename) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!input) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   out = fopen(filename, "wb");
   if (!out) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   //write numeric members
   CHECK_ERROR_TERMINATE(rox_objset_array2d_uchar_serialize(out, input))

function_terminate:
   if(out) fclose(out);

   return error;
}

Rox_ErrorCode rox_objset_array2d_uchar_deserialize(Rox_ObjSet_Array2D_Uchar output, Rox_Sint width, Rox_Sint height, FILE* in)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!in) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //read numeric members
   Rox_Uint oused = 0;
   size_t read_res = fread(&oused, sizeof(Rox_Uint), 1, in);
   if(read_res != 1) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   //read struct members
   for (Rox_Uint id = 0; id < oused; id++)
   {
      Rox_Array2D_Uchar data = NULL;
      CHECK_ERROR_RETURN(rox_array2d_uchar_new(&data, height, width))
      CHECK_ERROR_RETURN(rox_array2d_uchar_deserialize(data, in))
      CHECK_ERROR_RETURN(rox_objset_array2d_uchar_append(output,data))
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_array2d_uchar_deserialize_binary(Rox_ObjSet_Array2D_Uchar output, Rox_Sint width, Rox_Sint height, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * in = NULL;

   //handle file opening
   if (!filename) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!output) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   in = fopen(filename, "rb");
   if (!in) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   CHECK_ERROR_TERMINATE(rox_objset_array2d_uchar_deserialize(output, width, height, in))

function_terminate:
   if(in) fclose(in);

   return error;
}



