//==============================================================================
//
//    OPENROX   : File memory_pool.c
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
#include <system/errors/errors.h>
#include <inout/system/memory_print.h>

//TODO generic bloc sizes and values ?
#define NBR_BLOCKS_128        1000        //mbo 456   mbi 161
#define NBR_BLOCKS_1024       200         //mbo 127   mbi 24
#define NBR_BLOCKS_8192       50          //mbo 33    mbi 6
#define NBR_BLOCKS_65536      50          //mbo 34    mbi 14
#define NBR_BLOCKS_524288     15          //mbo 1     mbi 11
#define NBR_BLOCKS_4194304    15          //mbo 0     mbi 10

//128*1000     =  0,128,000
//1024*200     =  0,204,800
//8192*50      =  0,409,600
//65536*50     =  3,276,800
//524288*15    =  7,864,320
//4194304*15   =  62,914,560
//             =  74,798,080  =  74798080

//pointer to memory
static Rox_Void* blocks_128[NBR_BLOCKS_128];          
static Rox_Void* blocks_1024[NBR_BLOCKS_1024];        
static Rox_Void* blocks_8192[NBR_BLOCKS_8192];        
static Rox_Void* blocks_65536[NBR_BLOCKS_65536];
static Rox_Void* blocks_524288[NBR_BLOCKS_524288];
static Rox_Void* blocks_4194304[NBR_BLOCKS_4194304];

//occupation
static Rox_Bool blocks_occupation_128[NBR_BLOCKS_128];          
static Rox_Bool blocks_occupation_1024[NBR_BLOCKS_1024];        
static Rox_Bool blocks_occupation_8192[NBR_BLOCKS_8192];        
static Rox_Bool blocks_occupation_65536[NBR_BLOCKS_65536];
static Rox_Bool blocks_occupation_524288[NBR_BLOCKS_524288];
static Rox_Bool blocks_occupation_4194304[NBR_BLOCKS_4194304];

//next free
static Rox_Ushort blocks_free_128 = 0;
static Rox_Ushort blocks_free_1024 = 0;
static Rox_Ushort blocks_free_8192 = 0;
static Rox_Ushort blocks_free_65536 = 0;
static Rox_Ushort blocks_free_524288 = 0;
static Rox_Ushort blocks_free_4194304 = 0;

//last freed
static Rox_Void* blocks_freed_128 = NULL;
static Rox_Void* blocks_freed_1024 = NULL;
static Rox_Void* blocks_freed_8192 = NULL;
static Rox_Void* blocks_freed_65536 = NULL;
static Rox_Void* blocks_freed_524288 = NULL;
static Rox_Void* blocks_freed_4194304 = NULL;

//general allocation
static Rox_Char blocks[74798080];
static Rox_Bool blocks_ready = 0;

void rox_memory_pool_init()
{
   //TODO if (USHRT_MAX < NBR_BLOCKS_XXX)
   for (Rox_Ushort i = 0; i < NBR_BLOCKS_128; ++i)
   {
      blocks_128[i] = (&blocks[0]) + i * 128;
      blocks_occupation_128[i] = 0;
   }

   for (Rox_Ushort i = 0; i < NBR_BLOCKS_1024; ++i)
   {
      blocks_1024[i] = (&blocks[0]) + i * 1024 + NBR_BLOCKS_128 * 128;
      blocks_occupation_1024[i] = 0;
   }

   for (Rox_Ushort i = 0; i < NBR_BLOCKS_8192; ++i)
   {
      blocks_8192[i] = (&blocks[0]) + i * 8192 + NBR_BLOCKS_1024 * 1024 + NBR_BLOCKS_128 * 128;
      blocks_occupation_8192[i] = 0;
   }

   for (Rox_Ushort i = 0; i < NBR_BLOCKS_65536; ++i)
   {
      blocks_65536[i] = (&blocks[0]) + i * 65536 + NBR_BLOCKS_8192 * 8192 + NBR_BLOCKS_1024 * 1024 + NBR_BLOCKS_128 * 128;
      blocks_occupation_65536[i] = 0;
   }

   for (Rox_Ushort i = 0; i < NBR_BLOCKS_524288; ++i)
   {
      blocks_524288[i] = (&blocks[0]) + i * 524288 + NBR_BLOCKS_65536 * 65536 + +NBR_BLOCKS_8192 * 8192 + NBR_BLOCKS_1024 * 1024 + NBR_BLOCKS_128 * 128;
      blocks_occupation_524288[i] = 0;
   }

   for (Rox_Ushort i = 0; i < NBR_BLOCKS_4194304; ++i)
   {
      blocks_4194304[i] = (&blocks[0]) + i * 4194304 + NBR_BLOCKS_524288 * 524288 + NBR_BLOCKS_65536 * 65536 + NBR_BLOCKS_8192 * 8192 + NBR_BLOCKS_1024 * 1024 + NBR_BLOCKS_128 * 128;
      blocks_occupation_4194304[i] = 0;
   }

   blocks_free_128 = 0;
   blocks_free_1024 = 0;
   blocks_free_8192 = 0;
   blocks_free_65536 = 0;
   blocks_free_524288 = 0;
   blocks_free_4194304 = 0;

   blocks_freed_128 = NULL;
   blocks_freed_1024 = NULL;
   blocks_freed_8192 = NULL;
   blocks_freed_65536 = NULL;
   blocks_freed_524288 = NULL;
   blocks_freed_4194304 = NULL;
   
}

void* next_free(void** blocks_addr, Rox_Bool* blocks_occupation)
{
   Rox_Bool* p_o = blocks_occupation;
   Rox_Ushort i = 0;
   while (*p_o != 0)
   {
      ++p_o;
      ++i;
   }

   *p_o = 1;
   return blocks_addr[i];
}

void * rox_memory_allocate(const Rox_Size element_size, const Rox_Size element_count)
{
   void *ret_ptr = NULL;

   if (element_count * element_size == 0)goto function_terminate;
   if ((Rox_Double) element_count * (Rox_Double) element_size >= (Rox_Double) SIZE_MAX) goto function_terminate;

   if (!blocks_ready)
   {
      rox_memory_pool_init();
      blocks_ready = 1;
   }

   const Rox_Size global_size = element_count * element_size;
   //choose the right container and search a free slot
   if (global_size < 128)
   {
      if (blocks_freed_128)
      {
         ret_ptr = blocks_freed_128;
         blocks_freed_128 = NULL;
      }
      else if (blocks_free_128 != 0)
      {
         //assigned saved free block
         ret_ptr = blocks_128[blocks_free_128];
         blocks_occupation_128[blocks_free_128] = 1;
         blocks_free_128 = 0;
      }
      else
      {
         ret_ptr = next_free(blocks_128, blocks_occupation_128);
      }      
   }
   else if (global_size < 1024)
   {
      if (blocks_freed_1024)
      {
         ret_ptr = blocks_freed_1024;
         blocks_freed_1024 = NULL;
      }
      else if (blocks_free_1024 != 0)
      {
         //assigned saved free block
         ret_ptr = blocks_1024[blocks_free_1024];
         blocks_occupation_1024[blocks_free_1024] = 1;
         blocks_free_1024 = 0;
      }
      else
      { 
         ret_ptr = next_free(blocks_1024, blocks_occupation_1024);
      }
   }
   else if (global_size < 8192)
   {
      if (blocks_freed_8192)
      {
         ret_ptr = blocks_freed_8192;
         blocks_freed_8192 = NULL;
      }
      else if (blocks_free_8192 != 0)
      {
         //assigned saved free block
         ret_ptr = blocks_8192[blocks_free_8192];
         blocks_occupation_8192[blocks_free_8192] = 1;
         blocks_free_8192 = 0;
      }
      else
      {
         ret_ptr = next_free(blocks_8192, blocks_occupation_8192);
      }
   }
   else if (global_size < 65536)
   {
      if (blocks_freed_65536)
      {
         ret_ptr = blocks_freed_65536;
         blocks_freed_65536 = NULL;
      }
      else if (blocks_free_65536 != 0)
      {
         //assigned saved free block
         ret_ptr = blocks_65536[blocks_free_65536];
         blocks_occupation_65536[blocks_free_65536] = 1;
         blocks_free_65536 = 0;
      }
      else
      {
         ret_ptr = next_free(blocks_65536, blocks_occupation_65536);
      }
   }
   else if (global_size < 524288)
   {
      if (blocks_freed_524288)
      {
         ret_ptr = blocks_freed_524288;
         blocks_freed_524288 = NULL;
      }
      else if (blocks_free_524288 != 0)
      {
         //assigned saved free block
         ret_ptr = blocks_524288[blocks_free_524288];
         blocks_occupation_524288[blocks_free_524288] = 1;
         blocks_free_524288 = 0;
      }
      else
      {
         ret_ptr = next_free(blocks_524288, blocks_occupation_524288);
      }
   }
   else if (global_size < 4194304)
   {
      if (blocks_freed_4194304)
      {
         ret_ptr = blocks_freed_4194304;
         blocks_freed_4194304 = NULL;
      }
      else if (blocks_free_4194304 != 0)
      {
         //assigned saved free block
         ret_ptr = blocks_4194304[blocks_free_4194304];
         blocks_occupation_4194304[blocks_free_4194304] = 1;
         blocks_free_4194304 = 0;
      }
      else
      {
         ret_ptr = next_free(blocks_4194304, blocks_occupation_4194304);
      }
   }

   //TODO ?
   if (!ret_ptr)
   {
      rox_log("\n NOT FOUND count: %d, size: %d\n", element_count, element_size);
   }
   
#ifdef OPENROX_LOGMEMORY
   rox_memory_log_alloc(ret_ptr, global_size);
#endif

function_terminate:
   return ret_ptr;
}

void * rox_memory_reallocate(void *pointer, const Rox_Size element_size, const Rox_Size element_count)
{
   void *ret_ptr = NULL;
   Rox_Bool found = 0;
   Rox_Bool need_reallocate = 0;
   Rox_Size previous_block_size = 0;

   if (!pointer) goto function_terminate;
   if (element_count * element_size == 0)goto function_terminate;
   if ((Rox_Double) element_count * (Rox_Double)element_size >= (Rox_Double)SIZE_MAX)goto function_terminate;

   if (!blocks_ready)
   {
      rox_memory_pool_init();
      blocks_ready = 1;
   }

   //search for this pointer and check if its current block is big enough, otherwise delete and allocate a new one
   for (Rox_Ushort i = 0; !found && i < NBR_BLOCKS_128; ++i)
   {
      if (blocks_128[i] == pointer)
      {
         found = 1;
         if (element_size * element_count > 128)
         {
            //give block back
            blocks_occupation_128[i] = 0;
            blocks_free_128 = i;
            need_reallocate = 1;
            previous_block_size = 128;
         }
      }
   }
   for (Rox_Ushort i = 0; !found && i < NBR_BLOCKS_1024; ++i)
   {
      if (blocks_1024[i] == pointer)
      {
         found = 1;
         if (element_size * element_count > 1024)
         {
            //give block back
            blocks_occupation_1024[i] = 0;
            blocks_free_1024 = i;
            need_reallocate = 1;
            previous_block_size = 1024;
         }
      }
   }
   for (Rox_Ushort i = 0; !found && i < NBR_BLOCKS_8192; ++i)
   {
      if (blocks_8192[i] == pointer)
      {
         found = 1; 
         if (element_size * element_count > 8192)
         {
            //give block back
            blocks_occupation_8192[i] = 0;
            blocks_free_8192 = i;
            need_reallocate = 1;
            previous_block_size = 8192;
         }
      }
   }

   for (Rox_Ushort i = 0; !found && i < NBR_BLOCKS_65536; ++i)
   {
      if (blocks_65536[i] == pointer)
      {
         found = 1;
         if (element_size * element_count > 65536)
         {
            //give block back
            blocks_occupation_65536[i] = 0;
            blocks_free_65536 = i;
            need_reallocate = 1;
            previous_block_size = 65536;
         }
      }
   }

   for (Rox_Ushort i = 0; !found && i < NBR_BLOCKS_524288; ++i)
   {
      if (blocks_524288[i] == pointer)
      {
         found = 1;
         if (element_size * element_count > 524288)
         {
            //give block back
            blocks_occupation_524288[i] = 0;
            blocks_free_524288 = i;
            need_reallocate = 1;
            previous_block_size = 524288;
         }
      }
   }

   for (Rox_Ushort i = 0; !found && i < NBR_BLOCKS_4194304; ++i)
   {
      if (blocks_4194304[i] == pointer)
      {
         found = 1;
         if (element_size * element_count > 4194304)
         {
            //give block back
            blocks_occupation_4194304[i] = 0;
            blocks_free_4194304 = i;
            need_reallocate = 1;
            previous_block_size = 4194304;
         }
      }
   }

   //TODO if (!found)
   if (found)
   {
      //we gave back current block, reallocate new one and copy data 
      if (need_reallocate)
      {
         //to have a balanced log, we need to log this free
#ifdef OPENROX_LOGMEMORY
         rox_memory_log_delete(pointer);
#endif
         ret_ptr = rox_memory_allocate(element_size, element_count);

         //if previous block is bigger, copy the new size (rest of data is left behind)
         if (previous_block_size > element_size*element_count)
            previous_block_size = element_size*element_count;

         memcpy(ret_ptr, pointer, previous_block_size);
      }
      else //otherwise just give back the pointer as the needed size is still inferior to our current block size
      {
         ret_ptr = pointer;
      }
   }
   //else
   //{
   //   rox_log("\n NOT FOUND count: %d, size: %d\n", element_count, element_size);
   //}


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
      //search for this pointer and give back its location to the pool  
      if (pointer < blocks_1024[0])
      {
         if (blocks_freed_128 == NULL)
            blocks_freed_128 = pointer;
         else
         {
            const Rox_Ushort index = ((char*)pointer - (char*)blocks_128[0]) / 128;
            blocks_occupation_128[index] = 0;
            blocks_free_128 = index;
         }
      }
      else if (pointer < blocks_8192[0])
      {
         if (blocks_freed_1024 == NULL)
            blocks_freed_1024 = pointer;
         else
         {
            const Rox_Ushort index = ((char*)pointer - (char*)blocks_1024[0]) / 1024;
            blocks_occupation_1024[index] = 0;
            blocks_free_1024 = index;
         }
      }
      else if (pointer < blocks_65536[0])
      {
         if (blocks_freed_8192 == NULL)
            blocks_freed_8192 = pointer;
         else
         {
            const Rox_Ushort index = ((char*)pointer - (char*)blocks_8192[0]) / 8192;
            blocks_occupation_8192[index] = 0;
            blocks_free_8192 = index;
         }
      }
      else if (pointer < blocks_524288[0])
      {
         if (blocks_freed_65536 == NULL)
            blocks_freed_65536 = pointer;
         else
         {
            const Rox_Ushort index = ((char*)pointer - (char*)blocks_65536[0]) / 65536;
            blocks_occupation_65536[index] = 0;
            blocks_free_65536 = index;
         }
      }
      else if (pointer < blocks_4194304[0])
      {
         if (blocks_freed_524288 == NULL)
            blocks_freed_524288 = pointer;
         else
         {
            const Rox_Ushort index = ((char*)pointer - (char*)blocks_524288[0]) / 524288;
            blocks_occupation_524288[index] = 0;
            blocks_free_524288 = index;
         }
      }
      else
      {
         if (blocks_freed_4194304 == NULL)
            blocks_freed_4194304 = pointer;
         else
         {
            const Rox_Ushort index = ((char*)pointer - (char*)blocks_4194304[0]) / 4194304;
            blocks_occupation_4194304[index] = 0;
            blocks_free_4194304 = index;
         }
      }

   }
}

void * rox_memory_allocate_aligned(void **aligned_adress, const Rox_Size element_size, const Rox_Size element_count, const unsigned char alignment_bytes)
{
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
   aligned_ptr = (void *)((((Rox_Size)base_ptr) + alignment) & ~alignment);
   if (aligned_adress) *aligned_adress = aligned_ptr;

function_terminate:
   return base_ptr;
}

void *rox_memory_reallocate_aligned(void **aligned_adress, void *oldpointer, void *oldaligned, const Rox_Size oldsize, const Rox_Size element_size, const Rox_Size element_count, const unsigned char alignment_bytes)
{
   Rox_Size total_alloc = 0;
   Rox_Size alignment = 0;
   Rox_Size copysize = 0;
   void * base_ptr = NULL;
   void * aligned_ptr = NULL;

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
   aligned_ptr = (void *)((((Rox_Size)base_ptr) + alignment) & ~alignment);
   if (aligned_adress) *aligned_adress = aligned_ptr;

   // Copy old array content to new array
   copysize = element_size * element_count;
   if (copysize > oldsize) copysize = oldsize;
   memcpy(aligned_ptr, oldaligned, copysize);

   // dereference old pointer
   rox_memory_delete(oldpointer);

function_terminate:
   return base_ptr;
}


