//============================================================================
//
//    OPENROX   : File errors_print.c
//
//    Contents  : Implementation of errors_print module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "errors_print.h"

#include <stdio.h>

#include <system/errors/errors.h>
#include <system/memory/datatypes.h>
#include <inout/system/print.h>

Rox_Void rox_error_print(Rox_ErrorCode error)
{
   rox_log("%s", rox_error_code_to_string(error));
}

Rox_Void rox_error_print_debug(Rox_ErrorCode error)
{
#ifdef OPENROX_DISPLAY
   rox_log("%s", rox_error_code_to_string(error));
#endif
}

Rox_Void rox_error_logfile_debug(FILE * logfile, Rox_ErrorCode error)
{
#ifdef OPENROX_LOGFILE
   fprintf(logfile, rox_error_code_to_string(error));
#endif
}

const Rox_Char * rox_error_code_to_string(Rox_ErrorCode error)
{
   switch(error)
   {
      default:
      {
         return "ROX_ERROR undefined value \n";
      }
      case ROX_ERROR_NONE:
      {
         return "ROX_ERROR_NONE \n";
      }
      case ROX_ERROR_NULL_POINTER:
      {
         return "ROX_ERROR_NULL_POINTER \n";
      }
      case ROX_ERROR_BAD_IOSTREAM:
      {
         return "ROX_ERROR_BAD_IOSTREAM \n";
      }
      case ROX_ERROR_BAD_TYPE:
      {
         return "ROX_ERROR_BAD_TYPE \n";
      }
      case ROX_ERROR_BAD_SIZE:
      {
         return " ROX_ERROR_BAD_SIZE \n";
      }
      case ROX_ERROR_ARRAYS_NOT_MATCH:
      {
         return "ROX_ERROR_ARRAYS_NOT_MATCH \n";
      }
      case ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE:
      {
         return "ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE \n";
      }
      case ROX_ERROR_INVALID_VALUE:
      {
         return "ROX_ERROR_INVALID_VALUE \n";
      }
      case ROX_ERROR_TOO_LARGE_VALUE:
      {
         return "ROX_ERROR_TOO_LARGE_VALUE \n";
      }
      case ROX_ERROR_EXTERNAL:
      {
         return "ROX_ERROR_EXTERNAL \n";
      }
      case ROX_ERROR_VALUE_NOT_EVEN:
      {
         return "ROX_ERROR_VALUE_NOT_EVEN \n";
      }
      case ROX_ERROR_VALUE_NOT_ODD:
      {
         return "ROX_ERROR_VALUE_NOT_ODD \n";
      }
      case ROX_ERROR_INVALID:
      {
         return "ROX_ERROR_INVALID \n";
      }
      case ROX_ERROR_TO_BE_DEFINED:
      {
         return "ROX_ERROR_TO_BE_DEFINED \n";
      }
      case ROX_ERROR_INTERNAL:
      {
         return "ROX_ERROR_INTERNAL \n";
      }
      case ROX_ERROR_FILE_NOT_FOUND:
      {
         return "ROX_ERROR_FILE_NOT_FOUND \n";
      }
      case ROX_ERROR_TEMPLATE_NOT_FOUND:
      {
         return "ROX_ERROR_TEMPLATE_NOT_FOUND \n";
      }
      case ROX_ERROR_ALGORITHM_FAILURE:
      {
         return "ROX_ERROR_ALGORITHM_FAILURE \n";
      }
      case ROX_ERROR_PROCESS_FAILED:
      {
         return "ROX_ERROR_PROCESS_FAILED \n";
      }
      case ROX_ERROR_FULL_BUFFER:
      {
         return "ROX_ERROR_FULL_BUFFER \n";
      }
      case ROX_ERROR_EMPTY_BUFFER:
      {
         return "ROX_ERROR_EMPTY_BUFFER \n";
      }
      case ROX_ERROR_ZNCC_UNDEFINED:
      {
         return "ROX_ERROR_ZNCC_UNDEFINED \n";
      }
      case ROX_ERROR_DETERMINANT_NULL:
      {
         return "ROX_ERROR_DETERMINANT_NULL \n";
      }
      case ROX_ERROR_ALL_POINTS_NULL:
      {
         return "ROX_ERROR_ALL_POINTS_NULL \n";
      }
      case ROX_ERROR_INSUFFICIENT_DATA:
      {
         return "ROX_ERROR_INSUFFICIENT_DATA \n";
      }
      case ROX_ERROR_NOT_IMPLEMENTED:
      {
         return "ROX_ERROR_NOT_IMPLEMENTED \n";
      }
   }
}
