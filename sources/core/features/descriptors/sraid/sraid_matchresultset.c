//==============================================================================
//
//    OPENROX   : File sraid_matchresultset.h
//
//    Contents  : Implementation of sraid_matchresultset module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sraid_matchresultset.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_sraid_matchresultset_new(Rox_SRAID_MatchResultSet * obj, Rox_Uint maxresults)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_SRAID_MatchResultSet ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   if (maxresults == 0) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_SRAID_MatchResultSet)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->results = (Rox_SRAID_MatchResult_Struct *) rox_memory_allocate(sizeof(struct Rox_SRAID_MatchResult_Struct), maxresults);
   ret->count_results = 0;
   ret->max_results = maxresults;

   if (!ret->results)
   {
      rox_sraid_matchresultset_del(&ret);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_matchresultset_del(Rox_SRAID_MatchResultSet * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_SRAID_MatchResultSet todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_memory_delete(todel->results);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_matchresultset_clear(Rox_SRAID_MatchResultSet obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->count_results = 0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_sraid_matchresultset_addresult(Rox_SRAID_MatchResultSet obj, Rox_Uint index, Rox_Uint distance)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   int posfound = -1;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint pos = 0;
   while (pos < obj->count_results && posfound <  0)
   {
      if (distance < obj->results[pos].distance)
      {
         posfound = pos;
      }
      pos++;
   }

   if (posfound < 0)
   {
      //Maybe we can add it to the end
      if (obj->count_results < obj->max_results)
      {
         obj->results[obj->count_results].index = index;
         obj->results[obj->count_results].distance = distance;
         obj->count_results++;
      }
      goto function_terminate;
   }

   // We have to shift next results
   pos = obj->count_results - 1;
   while (pos > (Rox_Uint) posfound)
   {
      obj->results[pos] = obj->results[pos - 1];
      pos--;
   }

   // Add index
   obj->results[posfound].index = index;
   obj->results[posfound].distance = distance;

function_terminate:
   return error;
}

Rox_Uint rox_sraid_matchresultset_isfull(Rox_SRAID_MatchResultSet obj)
{
   if (!obj) return 1;
   if (obj->count_results >= obj->max_results) return 1;

   return 0;
}
