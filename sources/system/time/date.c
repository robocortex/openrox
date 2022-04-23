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
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_date_get_year(Rox_Uint *year, Rox_Date date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!year || !date) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   *year = date->_year;
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_date_get_month(Rox_Uint *month, Rox_Date date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!month || !date) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   *month = date->_month;

function_terminate:
   return error;
}

Rox_ErrorCode rox_date_get_day(Rox_Uint *day, Rox_Date date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!day || !date) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   *day = date->_day;

function_terminate:
   return error; 
}

Rox_ErrorCode rox_date_get_hours(Rox_Uint *hours, Rox_Date date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!hours || !date) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   *hours = date->_hours;
   
function_terminate:
   return error; 
}

Rox_ErrorCode rox_date_get_minutes(Rox_Uint *minutes, Rox_Date date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!minutes || !date)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   *minutes = date->_minutes;

function_terminate:
   return error; 
}

Rox_ErrorCode rox_date_get_seconds(Rox_Uint *seconds, Rox_Date date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!seconds || !date) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   *seconds = date->_seconds;
   
function_terminate:
   return error; 
}

Rox_ErrorCode rox_date_get_milliseconds(Rox_Uint *milliseconds, Rox_Date date)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!milliseconds || !date) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   
   *milliseconds = date->_milliseconds;
   
function_terminate:
   return error; 
}
