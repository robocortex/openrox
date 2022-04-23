//==============================================================================
//
//    OPENROX   : File scalar_save.c
//
//    Contents  : Implementation of scalar save module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "scalar_save.h"

#include <stdio.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_double_save ( 
   const Rox_Char * filename, 
   const Rox_Double input 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *file = NULL;

   if ( !input || !filename)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "w");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   fprintf(file, "%32.32f ", input);

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error;
}

Rox_ErrorCode rox_double_save_append ( 
   const Rox_Char * filename, 
   const Rox_Double input 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *file = NULL;

   if ( !input || !filename)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "a");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   fprintf(file, "%32.32f ", input);

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error;
}
