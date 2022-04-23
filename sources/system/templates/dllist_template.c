//==============================================================================
//
//    OPENROX   : File dllist_template.h
//
//    Contents  : Implementation of dllist_template module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dllist_@LDLLISTTYPE@.h"
#include "dllist_@LDLLISTTYPE@_struct.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_new(Rox_Dllist_@DLLISTTYPE@ * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Dllist_@DLLISTTYPE@ ret;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   *obj = 0;

   ret = (Rox_Dllist_@DLLISTTYPE@) rox_memory_allocate(sizeof(struct Rox_Dllist_@DLLISTTYPE@_Struct), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // List is empty by default
   ret->used = 0;
   ret->allocated = 0;
   ret->first = 0;
   ret->last = 0;
   ret->last_used = 0;

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_del(Rox_Dllist_@DLLISTTYPE@ * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Dllist_@DLLISTTYPE@ todel = NULL;
   Rox_Dllist_@DLLISTTYPE@_Node node = NULL, next = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   todel = *obj;
   *obj = NULL;
   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   // Recursively delete list
   node = todel->first;
   while (node)
   {
      next = node->next;
      rox_memory_delete(node);
      node = next;
   }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_append(Rox_Dllist_@DLLISTTYPE@ obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Dllist_@DLLISTTYPE@_Node node = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   node = (Rox_Dllist_@DLLISTTYPE@_Node) rox_memory_allocate(sizeof(struct Rox_Dllist_@DLLISTTYPE@_Node_Struct), 1);
   if (!node) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   node->previous = 0;
   node->next = 0;
   obj->allocated++;

   // If first node
   if (!obj->first) obj->first = node;

   // If a node exists
   if (obj->last)
   {
      obj->last->next = node;
      node->previous = obj->last;
   }

   // Set node as last one
   obj->last = node;

function_terminate:
   return error;
}

Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_removelast(Rox_Dllist_@DLLISTTYPE@ obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Dllist_@DLLISTTYPE@_Node prev = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!obj->last) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Store previous element
   prev = 0;
   if (obj->last->previous)
   {
      prev = obj->last->previous;
   }

   if (obj->last == obj->first) obj->first = 0;
   if (obj->last == obj->last_used) obj->last_used = prev;

   // Update last node
   rox_memory_delete(obj->last);
   obj->last = prev;

   obj->allocated--;

function_terminate:
   return error;
}

Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_removefirst(Rox_Dllist_@DLLISTTYPE@ obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Dllist_@DLLISTTYPE@_Node next = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if (!obj->first) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Store previous element
   if (obj->first->next)
   {
      next = obj->first->next;
   }

   if (obj->last == obj->first) obj->first = 0;

   // Update last node
   rox_memory_delete(obj->first);
   obj->first = next;

   obj->allocated--;
   obj->used--;

function_terminate:
   return error;
}

Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_reset(Rox_Dllist_@DLLISTTYPE@ obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   obj->last_used = NULL;
   obj->used = 0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_dllist_@LDLLISTTYPE@_add(Rox_Dllist_@DLLISTTYPE@ obj, Rox_@DLLISTTYPE@_Struct * val)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (obj->allocated == 0 || obj->last_used == obj->last)
   {
      error = rox_dllist_@LDLLISTTYPE@_append(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      obj->last_used = obj->last;
   }
   else if (obj->last_used == NULL)
   {
      obj->last_used = obj->first;
   }
   else
   {
      obj->last_used = obj->last_used->next;
   }

   obj->last_used->data = *val;
   obj->used++;

function_terminate:
   return error;
}
