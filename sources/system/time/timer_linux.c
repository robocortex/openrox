//==============================================================================
//
//    OPENROX   : File timer.h
//
//    Contents  : Implementation of timer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "timer.h"
#include <sys/time.h>
#include <inout/system/errors_print.h>

//! To be commented  
struct Rox_Timer_Struct
{
  //! To be commented  
  int started;
  //! To be commented  
  double _begin_time;
  //! To be commented  
  double _end_time;
};

Rox_ErrorCode rox_timer_new(Rox_Timer * timer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!timer) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Timer ret = (Rox_Timer)rox_memory_allocate(sizeof(struct Rox_Timer_Struct), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->_begin_time = 0;
   ret->_end_time = 0;
   ret->started = 0;

   *timer = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_timer_del(Rox_Timer * timer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!timer) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Timer todel = *timer;
   *timer = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_timer_start(Rox_Timer timer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
	struct timeval tv;

   if (!timer) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   gettimeofday(&tv, NULL);

   timer->_begin_time = tv.tv_sec + (tv.tv_usec * 1e-6);
   timer->_end_time = timer->_begin_time;
   timer->started = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_timer_stop(Rox_Timer timer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   struct timeval tv;

   if (!timer) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   
   if (!timer->started) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error); }

   gettimeofday(&tv, NULL);

   timer->_end_time = tv.tv_sec + (tv.tv_usec * 1e-6);
   timer->started = 0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_timer_get_elapsed_ms ( Rox_Double *milliseconds, Rox_Timer timer )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!timer) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   
   *milliseconds = 1000.0 * (double)(timer->_end_time - timer->_begin_time);

function_terminate:
   return error;
}
