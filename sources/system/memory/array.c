//============================================================================
//
//    OPENROX   : File array.c
//
//    Contents  : Implementation of array module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "array.h"
#include "array_struct.h"

#include <string.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array_new_internal (
   Rox_Array * array, 
   const Rox_Datatype_Description datatype, 
   const Rox_Uint length, Rox_Void * buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array ret_array = NULL;

   if (!array) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (length == 0) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( (Rox_Sint) datatype == 0) 
   { error = ROX_ERROR_BAD_TYPE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *array = NULL;

   // Allocate structure
   ret_array = (Rox_Array) rox_memory_allocate(sizeof(struct Rox_Array_Struct), 1);
   if (!ret_array) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!buffer)
   {
      // Allocate data
      ret_array->system_ptr = rox_memory_allocate_aligned(&ret_array->base_ptr, ROX_DATATYPE_BYTECOUNT(datatype), length, ROX_DEFAULT_ALIGNMENT);
      if (!ret_array->system_ptr)
      {
         rox_memory_delete(ret_array);
         {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
      }
      ret_array->owndata = 1;
   }
   else
   {
      // Don't allocate data, just tell where our buffer is
      ret_array->system_ptr = buffer;
      ret_array->base_ptr = buffer;
      ret_array->owndata = 0;
   }

   // Set information
   ret_array->datatype = datatype;
   ret_array->length = length;
   ret_array->reference_count = 1;

   *array = ret_array;

function_terminate:
   return error;

}

Rox_ErrorCode rox_array_new (
   Rox_Array * array, 
   const Rox_Datatype_Description datatype, 
   const Rox_Uint length
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!array) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (length == 0) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( (Rox_Sint) datatype == 0) 
   { error = ROX_ERROR_BAD_TYPE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Call generic structure initialization
   error = rox_array_new_internal ( array, datatype, length, NULL );

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_new_frombuffer (
   Rox_Array * array, 
   const Rox_Datatype_Description datatype, 
   const Rox_Uint length, 
   Rox_Void * buffer
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!buffer) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!array) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (length == 0) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( (Rox_Sint) datatype == 0) 
   { error = ROX_ERROR_BAD_TYPE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //call generic structure initialization
   error = rox_array_new_internal(array, datatype, length, buffer);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_del (
   Rox_Array *ptr
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array todel = NULL;

   if (!ptr) 
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   todel = *ptr;
   *ptr = NULL;
   
   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   if (todel->system_ptr && todel->owndata)
   {
      rox_memory_delete(todel->system_ptr);
   }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_reference (
   Rox_Array* ptr
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!ptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!*ptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) ;}

   (*ptr)->reference_count++;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_dereference (
   Rox_Array* ptr
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   if (!*ptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

   //Delete array if not used anymore
   if ((*ptr)->reference_count <= 1)
   {
      rox_array_del(ptr);
      goto function_terminate;
   }

   (*ptr)->reference_count--;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array_copy ( Rox_Array dest, const Rox_Array src )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Check parameters
   if (!dest || !src) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (dest->datatype != src->datatype) 
   { error = ROX_ERROR_BAD_TYPE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (dest->length != src->length) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Perform copy
   Rox_Size size = dest->length * ROX_DATATYPE_BYTECOUNT(dest->datatype);
   memcpy(dest->base_ptr, src->base_ptr, size);

function_terminate:
   return error;
}
