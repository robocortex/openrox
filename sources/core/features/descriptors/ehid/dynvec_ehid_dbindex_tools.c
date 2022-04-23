//==============================================================================
//
//    OPENROX   : File dynvec_ehid_dbindex.c
//
//    Contents  : Implementation of ehid_dbindex module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_ehid_dbindex_tools.h"
#include "ehid_dbindex_struct.h"

#include <stdio.h>

#include "generated/dynvec_ehid_dbindex_struct.h"
#include <inout/system/errors_print.h>
#include <system/errors/errors.h>

Rox_ErrorCode rox_dynvec_ehid_dbindex_load(Rox_DynVec_Ehid_DbIndex output, const char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * in = NULL;

  
   if (!output || !filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   in = fopen(filename, "r");
   if (!in) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint count = 0;
   Rox_Uint read = fscanf(in, "%u", &count);

   if (read < 1)
   {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_dynvec_ehid_dbindex_reset(output);

   for (Rox_Uint id = 0; id < count; id++)
   {
      Rox_Ehid_DbIndex_Struct cur;

      for ( Rox_Sint idx = 0; idx < 32; idx++)
      {

         read = fscanf(in, "%u", &cur.flag_indices[idx]); 
         if (read < 1) 
         { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }
      }

      rox_dynvec_ehid_dbindex_append(output, &cur);
   }


function_terminate:
   if (!in) fclose(in);
   return error;
}

Rox_ErrorCode rox_dynvec_ehid_dbindex_save(const char * filename, Rox_DynVec_Ehid_DbIndex input)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;

   if (!input || !filename) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   out = fopen(filename, "w");
   if (!out) {error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE(error)}

   fprintf(out, "%d\n", input->used);

   for (Rox_Uint id = 0; id < input->used; id++)
   {
      for ( Rox_Sint idx=0; idx < 32; idx++)
      {
         fprintf(out, "%d ", input->data[id].flag_indices[idx]);
      }

      fprintf(out, "\n");
   }

function_terminate:
   if (!out) fclose(out);
   return error;
}
