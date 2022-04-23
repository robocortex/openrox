//==============================================================================
//
//    OPENROX   : File print.h
//
//  	Contents  : API of print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_PRINT__
#define __OPENROX_PRINT__

#ifdef __cplusplus
extern "C" {
#endif

   #include <system/arch/compiler.h>
   #include <generated/config.h>
   #include <stdio.h>

   //! \ingroup Utils
   //! \defgroup InOut System
   //! \brief InOut system functions.

   //! \ingroup InOut System
   //! @defgroup Log Log
   //! @{

   // Custom log functions.

   //! Log callback type
   typedef void(* rox_log_callback)(const char* message);

   //! Allows to log message on any type of devices
   //! using printf format
   //! \param fmt Format
   //! \param ... Additional message (printf-like)
   ROX_API void rox_log(const char *fmt, ...);

   //! Allows to specify a function to use to log any messages 
   ROX_API void rox_log_set_callback(rox_log_callback callback);

//! @} 

#ifdef __cplusplus
}
#endif //__cplusplus

#endif
