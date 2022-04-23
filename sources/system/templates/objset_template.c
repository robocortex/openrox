//==============================================================================
//
//    OPENROX   : File objset_@LOBJSETTYPE@.c
//
//    Contents  : Implementation of objset_@LOBJSETTYPE@ module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "generated/objset_@LOBJSETTYPE@.h"
#include "generated/objset_@LOBJSETTYPE@_struct.h"
#include <string.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_objset_@LOBJSETTYPE@_new(Rox_ObjSet_@OBJSETTYPE@ * obj, Rox_Uint allocblocks)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_ObjSet_@OBJSETTYPE@ ret_objset;
   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   // Allocate structure
   ret_objset = (Rox_ObjSet_@OBJSETTYPE@) rox_memory_allocate(sizeof(struct Rox_ObjSet_@OBJSETTYPE@_Struct), 1);
   if (!ret_objset) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Allocate data
   ret_objset->data = (Rox_@OBJSETTYPE@*) rox_memory_allocate(sizeof(Rox_@OBJSETTYPE@), allocblocks);
   if (!ret_objset->data)
   {
      rox_memory_delete(ret_objset);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   // Set information
   ret_objset->used = 0;
   ret_objset->allocated = allocblocks;
   ret_objset->allocblocks = allocblocks;

   *obj = ret_objset;

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_@LOBJSETTYPE@_del(Rox_ObjSet_@OBJSETTYPE@ * ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_ObjSet_@OBJSETTYPE@ todel = NULL;

   if ( !ptr ) 
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   todel = *ptr;
   *ptr = NULL;

   if ( !todel ) 
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   if (todel->data)
   {
      for ( Rox_Uint id = 0; id < todel->used; id++)
      {
         // rox_@LOBJSETTYPE@_del ( &todel->data[id] );
         @OBJSETDELETER@ ( &todel->data[id] );
      }
      rox_memory_delete(todel->data);
   }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_@LOBJSETTYPE@_get_data_pointer ( Rox_@OBJSETTYPE@ ** ptr_data, Rox_ObjSet_@OBJSETTYPE@ ptr )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !ptr || !ptr_data )
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   *ptr_data = ptr->data;

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_@LOBJSETTYPE@_get_used ( Rox_Uint * used, Rox_ObjSet_@OBJSETTYPE@ ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr) { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   *used = ptr->used;


function_terminate:
   return error;
}


Rox_ErrorCode rox_objset_@LOBJSETTYPE@_reset(Rox_ObjSet_@OBJSETTYPE@ ptr)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr){ error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   for ( Rox_Uint id = 0; id < ptr->used; id++)
   {
      // error = rox_@LOBJSETTYPE@_del ( &ptr->data[id] );
      error = @OBJSETDELETER@(&ptr->data[id]);
      if(error)goto function_terminate;
   }

   ptr->used = 0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_@LOBJSETTYPE@_stack(Rox_ObjSet_@OBJSETTYPE@ ptr, Rox_ObjSet_@OBJSETTYPE@ other)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr || !other) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (other->used == 0) {error = ROX_ERROR_NONE; goto function_terminate;}

   Rox_Uint lastcount = ptr->used;
   ptr->used = ptr->used + other->used;

   if (ptr->used >= ptr->allocated)
   {
      // Be sure we allocate a multiple of allocblocks
      Rox_Uint blockused = ptr->used / ptr->allocblocks;
      ptr->allocated = (blockused + 1) * ptr->allocblocks;

      // Update memory allocation
      ptr->data = (Rox_@OBJSETTYPE@*)rox_memory_reallocate(ptr->data, sizeof(Rox_@OBJSETTYPE@), ptr->allocated);
   }

   memcpy(&ptr->data[lastcount], other->data, sizeof(Rox_@OBJSETTYPE@) * other->used);

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_@LOBJSETTYPE@_clone(Rox_ObjSet_@OBJSETTYPE@ ptr, Rox_ObjSet_@OBJSETTYPE@ source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr || !source) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ptr->used = source->used;

   if (ptr->allocated != source->allocated)
   {
      // Be sure we allocate a multiple of allocblocks
      ptr->allocated = source->allocated;

      // Update memory allocation
      ptr->data = (Rox_@OBJSETTYPE@*)rox_memory_reallocate(ptr->data, sizeof(Rox_@OBJSETTYPE@), ptr->allocated);
   }

   memcpy(ptr->data, source->data, sizeof(Rox_@OBJSETTYPE@) * source->used);

function_terminate:
   return error;
}

Rox_ErrorCode rox_objset_@LOBJSETTYPE@_append(Rox_ObjSet_@OBJSETTYPE@ ptr, Rox_@OBJSETTYPE@ data)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint current_index = 0;

   if (!ptr) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!data) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   //increment used slots
   current_index = ptr->used;
   ptr->used++;

   //resize if needed
   if (current_index >= ptr->allocated)
   {
      // Be sure we allocate a multiple of allocblocks
      Rox_Uint blockused = ptr->used / ptr->allocblocks;
      ptr->allocated = (blockused + 1) * ptr->allocblocks;

      // Update memory allocation
      ptr->data = (Rox_@OBJSETTYPE@*)rox_memory_reallocate(ptr->data, sizeof(Rox_@OBJSETTYPE@), ptr->allocated);
   }

   //assign value to store
   ptr->data[current_index] = data;

function_terminate:
   return error;
}
