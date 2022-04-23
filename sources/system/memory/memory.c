//==============================================================================
//
//    OPENROX   : File memory.c
//
//    Contents  : Implementation of memory module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "memory.h"
#include <string.h>
#include <limits.h>
#include <system/errors/errors.h>
#include <inout/system/memory_print.h>

void * rox_memory_allocate(const Rox_Size element_size, const Rox_Size element_count)
{
   void *ret_ptr = NULL;

   //check parameters validity
   if (element_count * element_size == 0)goto function_terminate;
   if ((Rox_Double)element_count * (Rox_Double)element_size >= (Rox_Double)SIZE_MAX)goto function_terminate;

   // We simply use malloc to allocate memory
   ret_ptr = malloc(element_count * element_size);

#ifdef OPENROX_LOGMEMORY
   rox_memory_log_alloc(ret_ptr, element_count * element_size);
#endif

function_terminate:
   return ret_ptr;
}

void * rox_memory_reallocate(void *pointer, const Rox_Size element_size, const Rox_Size element_count)
{
   void *ret_ptr = NULL;

   //check parameters validity
   if (!pointer) goto function_terminate;
   if (element_count * element_size == 0) goto function_terminate;
   if ((Rox_Double)element_count * (Rox_Double)element_size >= (Rox_Double)SIZE_MAX)goto function_terminate;

   // We simply use realloc to reallocate memory
   ret_ptr = realloc(pointer, element_size * element_count);

#ifdef OPENROX_LOGMEMORY
   rox_memory_log_realloc(pointer, ret_ptr, element_count * element_size);
#endif

function_terminate:
   return ret_ptr;
}

void rox_memory_delete(void * pointer)
{
   // Check if pointer is valid
   if (pointer)
   {
#ifdef OPENROX_LOGMEMORY
      rox_memory_log_delete(pointer);
#endif

      // Free it
      free(pointer);
   }
}

void * rox_memory_allocate_aligned(void **aligned_adress, const Rox_Size element_size, const Rox_Size element_count, const Rox_Uchar alignment_bytes)
{
   //check parameters validity
   Rox_Size total_alloc = 0;
   Rox_Size alignment = 0;
   void *base_ptr = NULL;
   void *aligned_ptr = NULL;

   if (aligned_adress)*aligned_adress = NULL;
   else goto function_terminate;
   if (alignment_bytes == 0)goto function_terminate;
   if (element_count * element_size == 0)goto function_terminate;
   if ((Rox_Double)element_count * (Rox_Double)element_size >= (Rox_Double)SIZE_MAX)goto function_terminate;

   alignment = alignment_bytes - 1;
   // Array allocated is bigger than needed to allow shifting of start pointers for alignment
   total_alloc = element_size * element_count + alignment;
   base_ptr = rox_memory_allocate(total_alloc, 1);

   // Compute aligned pointer
   // If base_ptr is null, aligning it will always produce a 0 pointer also, so we don't need another if
   aligned_ptr = (void *)((((Rox_Size) base_ptr) + alignment) & ~alignment);
   if (aligned_adress) *aligned_adress = aligned_ptr;

function_terminate:
   return base_ptr;
}

void * rox_memory_reallocate_aligned ( void **aligned_adress, void *oldpointer, void *oldaligned, const Rox_Size oldsize, const Rox_Size element_size, const Rox_Size element_count, const Rox_Uchar alignment_bytes)
{
   Rox_Size total_alloc = 0;
   Rox_Size alignment = 0;
   Rox_Size copysize = 0;
   void * base_ptr = NULL;
   void * aligned_ptr = NULL;

   //check parameters validity
   if (aligned_adress)*aligned_adress = NULL;
   else goto function_terminate;
   if (element_count * element_size == 0)goto function_terminate;
   if (oldaligned == NULL)goto function_terminate;
   if ((Rox_Double)element_count * (Rox_Double)element_size >= (Rox_Double)SIZE_MAX)goto function_terminate;

   alignment = alignment_bytes - 1;
   // Array allocated is bigger than needed to allow shifting of start pointers for alignment
   total_alloc = element_size * element_count + alignment;
   base_ptr = rox_memory_allocate(total_alloc, 1);

   // Exit if error in allocate
   if (!base_ptr) goto function_terminate;

   // Compute aligned pointer
   // If base_ptr is null, aligning it will always produce a 0 pointer also, so we don't need another if
   aligned_ptr = (void *)((((Rox_Size) base_ptr) + alignment) & ~alignment);
   if (aligned_adress) *aligned_adress = aligned_ptr;

   // Copy old array content to new array
   copysize = element_size * element_count;
   if (copysize > oldsize) copysize = oldsize;
   memcpy(aligned_ptr, oldaligned, copysize);

   // dereference old pointer
   if (oldpointer)rox_memory_delete(oldpointer);
   else if (oldaligned)rox_memory_delete(oldaligned);

function_terminate:
   return base_ptr;
}


