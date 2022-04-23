//============================================================================
//
//    OPENROX   : File errors_print.h
//
//    Contents  : API of errors_print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#ifndef __OPENROX_ERRORS_PRINT__
#define __OPENROX_ERRORS_PRINT__

#include <stdio.h>
#include <system/memory/datatypes.h>

//! \ingroup  Utils
//! \defgroup Error Error
//! \brief Error display.

//! \addtogroup Error
//! @{

#define ROX_LOG_FILEPATH "openrox.log"

#ifdef OPENROX_DISPLAY
   //! Errors
   #define rox_error_print_DEBUG(error) {\
      printf("FILE : %s ; LINE: %d; FUNCTION : %s ; ERROR : ", __FILE__ , __LINE__, __func__ );\
      rox_error_print_debug(error); }
   //! Errors
   #define ROX_ERROR_CHECK_DISPLAY(A)     error = (A); if (error) { rox_error_print_DEBUG(error); }
   //! Warnings
   #define ROX_WARNING_DISPLAY_DEBUG(warning) {\
      printf("FILE : %s ; LINE: %d; FUNCTION : %s ; WARNING : ", __FILE__ , __LINE__, __func__ );\
      rox_error_print_debug(warning); }
   //! Warnings
   #define ROX_WARNING_CHECK_DISPLAY(A)     warning = (A); if (warning) { ROX_WARNING_DISPLAY_DEBUG(warning); }
   //! Infos
   #define ROX_INFO_DISPLAY(A) {\
      printf("FILE : %s ; LINE: %d; FUNCTION : %s ; INFO : %s", __FILE__ , __LINE__, __func__, A ); }
   //! Infos formatted
   #define ROX_INFO_FORMATTED_DISPLAY(A, ...) { printf(A, ##__VA_ARGS__); }

   //! Infos array 2d, #include <inout/numeric/array2d_print.h> before using this MACRO
   #define ROX_ARRAY2D_DOUBLE_PRINT(ARRAY, TITLE, ...){\
      ROX_INFO_FORMATTED(TITLE, ##__VA_ARGS__);\
      rox_array2d_double_print(ARRAY);\
      }

#else
   //! Errors
   #define rox_error_print_DEBUG(error)
   //! Errors
   #define ROX_ERROR_CHECK_DISPLAY(A)
   //! Warnings
   #define ROX_WARNING_DISPLAY_DEBUG(warning)
   //! Warnings
   #define ROX_WARNING_CHECK_DISPLAY(A)
   //! Infos
   #define ROX_INFO_DISPLAY(A)
   //! Infos
   #define ROX_INFO_FORMATTED_DISPLAY(A, ...)
   //! Infos array 2d
   #define ROX_ARRAY2D_DOUBLE_PRINT(ARRAY, TITLE, ...)
#endif

#ifdef OPENROX_LOGFILE
   //! Errors
   #define ROX_ERROR_LOGFILE_DEBUG(error) {\
      FILE *logfile;\
      logfile = fopen(ROX_LOG_FILEPATH, "a");\
      fprintf(logfile, "FILE : %s ; LINE: %d; FUNCTION : %s ; ERROR : ", __FILE__ , __LINE__ , __func__ );\
      rox_error_logfile_debug(logfile, error);\
      fclose(logfile); }
   #define ROX_ERROR_CHECK_LOGFILE(A)      error = (A); if (error) { ROX_ERROR_LOGFILE_DEBUG(error); }
   //! Warnings
   #define ROX_WARNING_LOGFILE_DEBUG(warning) {\
      FILE *logfile;\
      logfile = fopen(ROX_LOG_FILEPATH, "a");\
      fprintf(logfile, "FILE : %s ; LINE: %d; FUNCTION : %s ; WARNING : ", __FILE__ , __LINE__ , __func__ );\
      rox_error_logfile_debug(logfile, warning);\
      fclose(logfile); }
   //! Warnings
   #define ROX_WARNING_CHECK_LOGFILE(A)      warning = (A); if (warning) { ROX_WARNING_LOGFILE_DEBUG(warning); }
   //! Infos
   #define ROX_INFO_LOGFILE(info) {\
      FILE *logfile;\
      logfile = fopen(ROX_LOG_FILEPATH, "a");\
      fprintf(logfile, "FILE : %s ; LINE: %d; FUNCTION : %s ; INFO : %s", __FILE__ , __LINE__ , __func__, info );\
      fclose(logfile); }
//! Infos formatted
#define ROX_INFO_FORMATTED_LOGFILE(A, ...) {\
      FILE *logfile;\
      logfile = fopen(ROX_LOG_FILEPATH, "a");\
      fprintf(logfile, A, ##__VA_ARGS__); \
      fclose(logfile); }

//! Erase the content of the current log file
#define ROX_LOGFILE_RESET(){\
      FILE *logfile;\
      logfile = fopen(ROX_LOG_FILEPATH, "w");\
      fprintf(logfile, "FILE : %s ; LINE: %d; FUNCTION : %s ; INFO : %s\n\n", __FILE__ , __LINE__ , __func__, "Reset log file" );\
      fclose(logfile); }

//! Errors
#define ROX_PRINTD2LOG(A)\
   { FILE *logfile;\
     logfile = fopen(ROX_LOG_FILEPATH, "a");\
     fprintf(logfile, "FILE : %s ; LINE: %d; FUNCTION : %s ; VALUE : %d \n", __FILE__ , __LINE__ , __func__ , A);\
     fclose(logfile);}

//! Errors
#define ROX_PRINTS2LOG(A)\
   { FILE *logfile;\
     logfile = fopen(ROX_LOG_FILEPATH, "a");\
     fprintf(logfile, "FILE : %s ; LINE: %d; FUNCTION : %s ; VALUE : %s \n", __FILE__ , __LINE__ , __func__ , A);\
     fclose(logfile);}

//! Infos array 2d, #include <inout/numeric/array2d_print.h> before using this MACRO
#define ROX_ARRAY2D_DOUBLE_LOG(ARRAY){\
   FILE *logfile;\
   logfile = fopen(ROX_LOG_FILEPATH, "a");\
   rox_array2d_double_fprint(logfile, ARRAY);\
   fclose(logfile);}
#else
   //! Errors
   #define ROX_ERROR_LOGFILE_DEBUG(error)
   //! Errors
   #define ROX_ERROR_CHECK_LOGFILE(A)
   //! Warnings
   #define ROX_WARNING_LOGFILE_DEBUG(warning)
   //! Warnings
   #define ROX_WARNING_CHECK_LOGFILE(A)
   //! Warnings
   #define ROX_INFO_LOGFILE(A)
   //! Infos formatted
   #define ROX_INFO_FORMATTED_LOGFILE(A, ...)
   //! Erase the content of the current log file
   #define ROX_LOGFILE_RESET()
   #define ROX_PRINTD2LOG(A)
   #define ROX_PRINTS2LOG(A)
   #define ROX_ARRAY2D_DOUBLE_LOG(ARRAY)
#endif

//! Check Error and go to function terminate
#define ROX_ERROR_CHECK_TERMINATE(A)\
   error = (A);\
   if (error) { rox_error_print_DEBUG(error); ROX_ERROR_LOGFILE_DEBUG(error); goto function_terminate; }

//! Check Error and continue to next loop iteration
#define ROX_ERROR_CHECK_CONTINUE(A)\
   error = (A);\
   if (error) { rox_error_print_DEBUG(error); ROX_ERROR_LOGFILE_DEBUG(error); continue; }

//! Check Error and force loop termination
#define ROX_ERROR_CHECK_BREAK(A)\
   error = (A);\
   if (error) { rox_error_print_DEBUG(error); ROX_ERROR_LOGFILE_DEBUG(error); break; }

//! Check Error
#define ROX_ERROR_CHECK(A)\
   error = (A);\
   if (error) { rox_error_print_DEBUG(error); ROX_ERROR_LOGFILE_DEBUG(error); }

//! Check Warning
#define ROX_ERROR_WARNING(A)\
   { Rox_ErrorCode warning = (A);\
   if (warning) { ROX_WARNING_DISPLAY_DEBUG(warning); ROX_WARNING_LOGFILE_DEBUG(warning); } }

//! Infos formatted
#define ROX_INFO_FORMATTED(A, ...)\
      {ROX_INFO_FORMATTED_LOGFILE(A, ##__VA_ARGS__);ROX_INFO_FORMATTED_DISPLAY(A, ##__VA_ARGS__);}

//! Display an human readable description of the error code
//! \param [in] error Error to be displayed
//! \return Void
ROX_API Rox_Void rox_error_print ( Rox_ErrorCode error );

//! Display an human readable description of the error code in debug mode
//! \param [in] error Error to be displayed
//! \return Void
ROX_API Rox_Void rox_error_print_debug ( Rox_ErrorCode error );

//! Write on file an human readable description of the error code in debug mode
//! \param [out] logfile
//! \param [in] error Error to be displayed
//! \return Void
ROX_API Rox_Void rox_error_logfile_debug(FILE * logfile, Rox_ErrorCode error);

ROX_API const Rox_Char * rox_error_code_to_string(Rox_ErrorCode error);

//! @}

#endif // __OPENROX_ERRORS_PRINT__
