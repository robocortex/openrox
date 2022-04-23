//==============================================================================
//
//    OPENROX   : File date.h
//
//    Contents  : Implementation of date module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "date.h"
#include <sys/time.h>
#include <time.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_date_new(Rox_Date * date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Date ret = NULL;
   
   if (!date) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   ret = (Rox_Date) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   error = rox_date_init(ret); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   *date = ret;
   
function_terminate:
   if(error) rox_date_del(&ret);
   return error;
}

Rox_Void rox_date_del(Rox_Date * date)
{
   Rox_Date todel;
   
   if (date != 0)
   {
      todel = *date;
      *date = 0;
      
      if(todel != 0)
      {
         rox_memory_delete(todel);
      }
   }
}

Rox_ErrorCode rox_date_init(Rox_Date date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   struct timeval tv;
   time_t timestamp;
   struct tm * t;
   
   if(!date) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   timestamp = time(NULL);
   t = localtime(&timestamp);
   gettimeofday(&tv, NULL);

   date->_year = 1900 + t->tm_year;
   date->_month = 1 + t->tm_mon; // January = 0 ... december = 11 
   date->_day = t->tm_mday;
   date->_hours = t->tm_hour;
   date->_minutes = t->tm_min;
   date->_seconds = t->tm_sec;
   date->_milliseconds = (tv.tv_usec * 1e-3);
   
function_terminate:
   return error;
}
