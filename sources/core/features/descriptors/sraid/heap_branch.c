//==============================================================================
//
//    OPENROX   : File heap_branch.c
//
//    Contents  : Implementation of heap_branch module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "heap_branch.h"
#include <inout/system/errors_print.h>

// inline int Parent(int node)
int Parent(int node) // to compile with rox_dev
{
   return (node - 1) / 2;
}

// inline int LeftChild(int node)
int LeftChild(int node) // to compile with rox_dev
{
   return (node * 2) + 1;
}

Rox_ErrorCode rox_heap_branch_new(Rox_Heap_Branch * obj, Rox_Uint size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Heap_Branch ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   if (size < 1) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret = (Rox_Heap_Branch) rox_memory_allocate(sizeof(struct Rox_Heap_Branch_Struct), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->_count_elems = 0;
   ret->_max_elems = size;

   ret->_data = (Rox_Branch_Struct *)rox_memory_allocate(sizeof(Rox_Branch_Struct), size);
   if (!ret->_data)
   {
      rox_heap_branch_del(&ret);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_heap_branch_del(Rox_Heap_Branch * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Heap_Branch todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_memory_delete(todel->_data);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_heap_branch_reset(Rox_Heap_Branch obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   obj->_count_elems = 0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_heap_branch_shiftup(Rox_Heap_Branch obj, int node)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint parent, current;
   Rox_Branch_Struct item;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   current = node;
   parent = Parent(node);
   item = obj->_data[current];

   while (current > 0)
   {
      if (obj->_data[parent].score > item.score)
      {
         obj->_data[current] = obj->_data[parent];
         current = parent;
         parent = Parent(current);
      }
      else
      {
         break;
      }
   }

   obj->_data[current] = item;

function_terminate:
   return error;
}

Rox_ErrorCode rox_heap_branch_shiftdown(Rox_Heap_Branch obj, int node)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint child, current;
   Rox_Branch_Struct item;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   current = node;
   child = LeftChild(node);
   item = obj->_data[current];

   while (child < (Rox_Sint) obj->_count_elems)
   {
      if (child < (Rox_Sint) (obj->_count_elems - 1))
      {
         if (obj->_data[child].score > obj->_data[child + 1].score)
         {
            child++;
         }
      }

      if (item.score > obj->_data[child].score)
      {
         obj->_data[current] = obj->_data[child];
         current = child;
         child = LeftChild(current);
      }
      else
      {
         break;
      }
   }

   obj->_data[current] = item;

function_terminate:
   return error;
}

Rox_ErrorCode rox_heap_branch_push(Rox_Heap_Branch obj, Rox_Kdtree_Sraid_Node node, Rox_Uint score)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (obj->_count_elems >= obj->_max_elems) {error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

   obj->_data[obj->_count_elems].node = node;
   obj->_data[obj->_count_elems].score = score;

   error = rox_heap_branch_shiftup(obj, obj->_count_elems);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->_count_elems++;

function_terminate:
   return error;
}

Rox_ErrorCode rox_heap_branch_pop(Rox_Heap_Branch obj, Rox_Kdtree_Sraid_Node * node, Rox_Uint * score)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Branch_Struct branch;

   if (!obj || !node || !score) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (obj->_count_elems == 0) {error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE(error)}

   branch = obj->_data[0];
   obj->_data[0] = obj->_data[obj->_count_elems - 1];

   obj->_count_elems--;

   error = rox_heap_branch_shiftdown(obj, 0);
   // ROX_ERROR_CHECK_TERMINATE ( error );

   *node = branch.node;
   *score = branch.score;

function_terminate:
   return error;
}
