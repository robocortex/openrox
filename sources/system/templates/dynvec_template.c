//==============================================================================
//
//    OPENROX   : File dynvec_@LDYNVECTYPE@.c
//
//    Contents  : Implementation of dynvec_@LDYNVECTYPE@ module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "generated/dynvec_@LDYNVECTYPE@.h"
#include "generated/dynvec_@LDYNVECTYPE@_struct.h"
#include <string.h>

#include @DYNVECTYPE_INCLUDE_STRUCT@

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_new(Rox_DynVec_@DYNVECTYPE@ * obj, Rox_Uint allocblocks)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_@DYNVECTYPE@ ret_dynvec = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   // Allocate structure
   ret_dynvec = (Rox_DynVec_@DYNVECTYPE@) rox_memory_allocate(sizeof(struct Rox_DynVec_@DYNVECTYPE@_Struct), 1);
   if (!ret_dynvec)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret_dynvec->data = NULL;

   // Allocate data
   ret_dynvec->data = (Rox_@SDYNVECTYPE@ *) rox_memory_allocate(sizeof(Rox_@SDYNVECTYPE@), allocblocks);
   if (!ret_dynvec->data)
   {
      rox_memory_delete(ret_dynvec);
      { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

   // Set information
   ret_dynvec->used        = 0;
   ret_dynvec->allocated   = allocblocks;
   ret_dynvec->allocblocks = allocblocks;

   *obj = ret_dynvec;

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_del(Rox_DynVec_@DYNVECTYPE@ * ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_DynVec_@DYNVECTYPE@ todel = NULL;

   if (!ptr)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   todel = *ptr;
   *ptr = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   if (todel->data)
   {
      rox_memory_delete(todel->data);
   }

   rox_memory_delete(todel);

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_get_data_pointer ( Rox_@SDYNVECTYPE@ ** data_pointer, Rox_DynVec_@DYNVECTYPE@ ptr )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if (!ptr)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   *data_pointer = ptr->data;

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_get_used(Rox_Uint * nb_used, Rox_DynVec_@DYNVECTYPE@ ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!nb_used)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!ptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *nb_used = ptr->used;

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_reset(Rox_DynVec_@DYNVECTYPE@ ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ptr->used = 0;

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_usecells(Rox_DynVec_@DYNVECTYPE@ ptr, Rox_Uint nbcells)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ptr->used += nbcells;

   if (ptr->used >= ptr->allocated)
   {
      // Be sure we allocate a multiple of allocblocks
      Rox_Uint blockused = ptr->used / ptr->allocblocks;
      ptr->allocated = (blockused + 1) * ptr->allocblocks; //XXX wrong behavior if blockused is a multiple of allocblocks ?

      // Update memory allocation
      ptr->data = (Rox_@SDYNVECTYPE@ *) rox_memory_reallocate(ptr->data, sizeof(Rox_@SDYNVECTYPE@), ptr->allocated);
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_reserve(Rox_DynVec_@DYNVECTYPE@ ptr, Rox_Uint nbcells)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (nbcells > ptr->allocated)
   {
      // Be sure we allocate a multiple of allocblocks
      Rox_Uint blocks_needed = nbcells / ptr->allocblocks;
      blocks_needed += ( nbcells % ptr->allocblocks ) ? 1 : 0;

      ptr->allocated = blocks_needed * ptr->allocblocks;

      // Update memory allocation
      ptr->data = (Rox_@SDYNVECTYPE@ *) rox_memory_reallocate(ptr->data, sizeof(Rox_@SDYNVECTYPE@), ptr->allocated);
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_stack(Rox_DynVec_@DYNVECTYPE@ ptr, Rox_DynVec_@DYNVECTYPE@ other)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr || !other)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (other->used == 0)
   { error = ROX_ERROR_NONE; goto function_terminate; }

   Rox_Uint lastcount = ptr->used;
   ptr->used = ptr->used + other->used;

   if (ptr->used >= ptr->allocated)
   {
      // Be sure we allocate a multiple of allocblocks
      Rox_Uint blockused = ptr->used / ptr->allocblocks;
      ptr->allocated = (blockused + 1) * ptr->allocblocks;

      // Update memory allocation
      ptr->data = (Rox_@SDYNVECTYPE@ *) rox_memory_reallocate(ptr->data, sizeof(Rox_@SDYNVECTYPE@), ptr->allocated);
   }

   memcpy(&ptr->data[lastcount], other->data, sizeof(Rox_@SDYNVECTYPE@) *other->used);

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_clone(Rox_DynVec_@DYNVECTYPE@ ptr, Rox_DynVec_@DYNVECTYPE@ source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ptr->used = source->used;

   if (ptr->allocated != source->allocated)
   {
      // Be sure we allocate a multiple of allocblocks
      ptr->allocated = source->allocated;

      // Update memory allocation
      ptr->data = (Rox_@SDYNVECTYPE@ *) rox_memory_reallocate(ptr->data, sizeof(Rox_@SDYNVECTYPE@), ptr->allocated);
   }

   memcpy(ptr->data, source->data, sizeof(Rox_@SDYNVECTYPE@) *source->used);

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_@LDYNVECTYPE@_append(Rox_DynVec_@DYNVECTYPE@ ptr, Rox_@SDYNVECTYPE@ * elem)
{
   Rox_ErrorCode error         = ROX_ERROR_NONE;
   Rox_Uint      current_index = 0;

   if (!ptr)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!elem)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //increment used slots
   current_index = ptr->used;
   ptr->used++;

   //resize if needed
   if (current_index >= ptr->allocated)
   {
      // Be sure we allocate a multiple of allocblocks
      Rox_Uint blockused = ptr->used / ptr->allocblocks;
      ptr->allocated = (blockused + 1) * ptr->allocblocks;//XXX wrong behavior if blockused is a multiple of allocblocks ?

      // Update memory allocation
      ptr->data = (Rox_@SDYNVECTYPE@ *) rox_memory_reallocate(ptr->data, sizeof(Rox_@SDYNVECTYPE@), ptr->allocated);
   }

   //assign value to store
   ptr->data[current_index] = *elem;

function_terminate:
   return error;
}
