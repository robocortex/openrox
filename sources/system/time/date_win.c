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
#include <windows.h>
#include <winbase.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <time.h>
#include <inout/system/errors_print.h>

#pragma comment(lib, "Iphlpapi.lib")

Rox_ErrorCode rox_date_new(Rox_Date * date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!date) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   Rox_Date ret = (Rox_Date) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   error =  rox_date_init(ret); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   *date = ret;
   
function_terminate:
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
   // http://msdn.microsoft.com/en-us/library/3stkd9be%28v=vs.80%29.aspx (function gmtime_s)
   // http://msdn.microsoft.com/fr-fr/library/95e68951%28v=vs.80%29.aspx (function _ftime_s) 
   Rox_ErrorCode error = ROX_ERROR_NONE;

   struct tm t;
   struct _timeb timebuffer;
   time_t time;

   short timezone;
   unsigned short ms;
   errno_t _error;
   int dayLight;

   if(!date) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   // Set time variables 
   _tzset();
   _get_daylight(&dayLight);

   // Get date 
   _error = _ftime_s( &timebuffer );
   if(_error) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   
   // http://msdn.microsoft.com/en-us/library/90s5c885%28v=vs.80%29.aspx
   // timezone = local_time - UTC time expressed in minutes (Paris timezone is -60 minutes !!!)
   // dayLight = summer time expressed in hour 
   
   time = timebuffer.time - 60 * timebuffer.timezone; // + 3600 * dayLight // Not added to avoid wrong 1 hour shift
   ms = timebuffer.millitm;
   timezone = timebuffer.timezone;

   // Convert time_t struct to tm_t struct 
   _error = gmtime_s(&t, &time);
   if(_error) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   
   // Fill date structure 
   date->_year = t.tm_year + 1900;
   date->_month = t.tm_mon + 1; // January = 0 ... december = 11 
   date->_day = t.tm_mday;

   date->_hours = t.tm_hour;
   date->_minutes = t.tm_min;
   date->_seconds = t.tm_sec;
   date->_milliseconds = ms;

function_terminate:
   return error;
}
