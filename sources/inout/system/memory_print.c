//==============================================================================
//
//    OPENROX   : File memory_print.h
//
//    Contents  : Implementation of memory_print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "memory_print.h"

#include <system/errors/errors.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

#include <stdlib.h>
#include <limits.h>
#include <string.h>


//TODO extend to write in a file instead of only display in std output

#ifndef OPENROX_LOGMEMORY

Rox_ErrorCode rox_memory_log_alloc(void *pointer, const Rox_Size size){return ROX_ERROR_NONE;}
Rox_ErrorCode rox_memory_log_realloc(void *old_pointer, void *new_pointer, const Rox_Size size){return ROX_ERROR_NONE;}
Rox_ErrorCode rox_memory_log_delete(void *pointer){return ROX_ERROR_NONE;}
Rox_ErrorCode rox_memory_print_summary(){return ROX_ERROR_NONE;}

#else

//VS2015 implements rox_log with %zu as gcc
#if defined(_MSC_VER) && (_MSC_VER < 1900)
#define ROX_MEMLOG(TXT, VAL)   rox_log("%s : %Iu bytes\n", TXT, VAL)
#define ROX_MEMLOG2(TXT, VAL1, VAL2)   rox_log("%s : %Iu, %Iu bytes\n", TXT, VAL1, VAL2)
#else
#define ROX_MEMLOG(TXT, VAL)           ROX_INFO_FORMATTED("%s : %zu bytes\n",TXT, VAL); 
#define ROX_MEMLOG2(TXT, VAL1, VAL2)   ROX_INFO_FORMATTED("%s : %zu, %zu bytes\n", TXT, VAL1, VAL2);
#endif

#define MEM_DISPLAY_NB_SLOTS 1000

static void*         adresses[MEM_DISPLAY_NB_SLOTS];
static Rox_Size      sizes[MEM_DISPLAY_NB_SLOTS];
static Rox_Size      current_size = 0;
static Rox_Size      total_size = 0;
static Rox_Size      total_peak_size = 0;
static Rox_Size      max_size = 0;
static Rox_Ushort    max_adress = 0;
static Rox_Bool      tables_ready = 0;

int comp(const void * elem1, const void * elem2)
{
   Rox_Size f = *((Rox_Size*)elem1);
   Rox_Size s = *((Rox_Size*)elem2);
   if (f > s) return  1;
   if (f < s) return -1;
   return 0;
}

Rox_ErrorCode rox_memory_print_summary()
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ushort index = 0;
   Rox_Ushort nbr = 0;
   Rox_Size           sizes_sorted[MEM_DISPLAY_NB_SLOTS];
   memset(sizes_sorted, 0, MEM_DISPLAY_NB_SLOTS*sizeof(Rox_Size));
   //for (index = 0; index < MEM_DISPLAY_NB_SLOTS; ++index)
   //{
   //   sizes_sorted[nbr] = 0;
   //}

   ROX_INFO_FORMATTED("Entering rox_memory_print_summary()......................\n");

   //copy sizes table to sort it for display
   for (index = 0; index < MEM_DISPLAY_NB_SLOTS; ++index)
   {
      if (sizes[index] > 0)
      {
         sizes_sorted[nbr] = sizes[index];
         nbr++;
      }
   }
   qsort(sizes_sorted, sizeof(sizes_sorted) / sizeof(*sizes_sorted), sizeof(*sizes_sorted), comp);
   //display sorted sizes
   for (index = 0; index < MEM_DISPLAY_NB_SLOTS; ++index)
   {
      if (sizes_sorted[index] > 0)
      {
         ROX_MEMLOG2("index sorted", MEM_DISPLAY_NB_SLOTS-index, sizes_sorted[index]);
      }
   }

   ROX_MEMLOG("current", current_size);
   ROX_MEMLOG("peak", total_peak_size);
   ROX_MEMLOG("max size", max_size);
   ROX_MEMLOG("current nbr", nbr);
   ROX_MEMLOG("max nbr", max_adress);

   return error;
}

Rox_ErrorCode rox_memory_init()
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   for (Rox_Ushort index = 0; index < MEM_DISPLAY_NB_SLOTS; ++index)
   {
      sizes[index] = 0;
      adresses[index] = NULL;
   }
   return error;
}

Rox_ErrorCode rox_memory_log_alloc(void*ret_ptr, const Rox_Size size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ushort nbr = 0;
   Rox_Bool found = 0;

   if (ret_ptr == NULL)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   //make sure we've prepared our slots
   if (!tables_ready)
   {
      rox_memory_init();
      ROX_ERROR_CHECK_TERMINATE ( error );
      tables_ready = 1;
   }

   //search for an empty slot and update number of slots occupied
   for (Rox_Ushort index = 0; index < MEM_DISPLAY_NB_SLOTS; ++index)
   {
      if (sizes[index] != 0)
      {
         ++nbr;
      }
      else if (!found)
      {
         adresses[index] = ret_ptr;
         sizes[index] = size;
         found = 1;
         ++nbr;
      }
   }

   //we reached our limit of MEM_DISPLAY_NB_SLOTS addresses slots
   if (!found)
   {
      error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   //update globals
   current_size += size;
   total_size += size;
   if (total_peak_size < current_size)
   {
      //ROX_LOGFILE_RESET();
      //ROX_INFO_FORMATTED("\n**********************************  PEAK\n");
      //rox_memory_print_summary();
      total_peak_size = current_size;
   }
   if (max_adress < nbr)
   {
      //ROX_LOGFILE_RESET();
      //ROX_INFO_FORMATTED("\n**********************************  MAX_NBR\n");
      //rox_memory_print_summary();
      max_adress = nbr;
   }
   if (max_size < size)
   {
      //ROX_LOGFILE_RESET();
      //ROX_INFO_FORMATTED("\n**********************************  MAX_SIZE\n");
      //rox_memory_print_summary();
      max_size = size;
   }
   //ROX_LOGFILE_RESET();
   //ROX_INFO_FORMATTED("\n**********************************  ALL\n");
   //rox_memory_print_summary();

      
function_terminate:
   return error;
}
Rox_ErrorCode rox_memory_log_realloc(void *old_pointer, void *new_pointer, const Rox_Size size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ushort index = 0;
   Rox_Bool found = 0;

   if (old_pointer == NULL || new_pointer == NULL)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   //make sure we've prepared our slots
   if (!tables_ready)
   {
      rox_memory_init();
      ROX_ERROR_CHECK_TERMINATE ( error );
      tables_ready = 1;
   }

   //search for previously tracked memory address in our slots
   for (; !found && index < MEM_DISPLAY_NB_SLOTS; ++index)
   {
      if (old_pointer == adresses[index])
      {
         //update globals
         total_size += size - sizes[index];
         current_size += size - sizes[index];

         if (max_size < size)
         {
            //ROX_LOGFILE_RESET();
            ROX_INFO_FORMATTED("\n**********************************  MAX_SIZE\n");
            rox_memory_print_summary();
            max_size = size;
         }
         if (total_peak_size < current_size)
         {
            //ROX_LOGFILE_RESET();
            ROX_INFO_FORMATTED("\n**********************************  PEAK\n");
            rox_memory_print_summary();
            total_peak_size = current_size;
         }
         //update found slot
         adresses[index] = new_pointer;
         sizes[index] = size;
         //break
         found = 1;
      }
   }
   //seems like memory address was not tracked...
   if (!found)
   {
      error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
   }


function_terminate:
   return error;
}
Rox_ErrorCode rox_memory_log_delete(void *pointer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ushort index = 0;
   Rox_Bool found = 0;

   if (pointer == NULL)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );
   }

   for (; !found && index < MEM_DISPLAY_NB_SLOTS; ++index)
   {
      if (pointer == adresses[index])
      {
         //update globals
         current_size -= sizes[index];
         //reset slot for further use
         adresses[index] = NULL;
         sizes[index] = 0;
         //break
         found = 1;
      }
   }

   //seems like memory address was not tracked...
   if (!found)
   {
      error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
   }


function_terminate:
   return error;
}

#endif //OPENROX_LOGMEMORY
