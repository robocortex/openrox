//==============================================================================
//
//    OPENROX   : File print.c
//
//    Contents  : Implementation of print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "print.h"

#include <stdarg.h>
#include <stdio.h>

#ifdef ANDROID
   #include "android/log.h"
#endif

rox_log_callback _log_callback = NULL;

void rox_log(const char *fmt, ...)
{
   char buffer[2048];
   va_list args;
   va_start(args, fmt);
   snprintf(buffer, sizeof(buffer), fmt, args);
   if (_log_callback != NULL)
   {
      _log_callback(buffer);
   }
   else
   {
#ifdef ROX_LOGS
   #ifdef ANDROID
         __android_log_print(ANDROID_LOG_INFO, "OPENROX", "%s", buffer);
   #else
         vprintf(fmt, args);
   #endif
#else

#endif
   }
   va_end(args);
}

void rox_log_set_callback(rox_log_callback callback)
{
   _log_callback = callback;
}