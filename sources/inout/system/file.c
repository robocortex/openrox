//==============================================================================
//
//    OPENROX   : File file.c
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

#include "file.h"

#include <stdio.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_file_count_lines ( Rox_Size * lines_number, const Rox_Char * filename )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE *file = NULL;

   if ( !lines_number || !filename)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   file = fopen(filename, "r");
   if ( file == NULL )
   { error = ROX_ERROR_FILE_NOT_FOUND; ROX_ERROR_CHECK_TERMINATE ( error ); }

   int count = 0;
   char temp;

   while (fscanf(file,"%c", &temp) != -1)
   {
      if (temp == 10) count++;
   }

	*lines_number = count;

function_terminate: 
   if ( file != NULL ) fclose(file);
   return error;
}