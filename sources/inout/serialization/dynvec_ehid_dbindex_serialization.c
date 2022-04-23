//==============================================================================
//
//    OPENROX   : File dynvec_ehid_dbindex_serialization.c
//
//    Contents  : Implementation of dynvec_ehid_dbindex_serialization module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <string.h>
#include <generated/dynvec_ehid_dbindex_struct.h>

#include <inout/system/errors_print.h>
#include <inout/serialization/dynvec_ehid_dbindex_serialization.h>

Rox_ErrorCode rox_dynvec_ehid_dbindex_serialize(Rox_Char *buffer, Rox_DynVec_Ehid_DbIndex source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint offset = 0;

   if( !buffer || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //Store indice count
   memcpy(buffer + offset, &(source->used), sizeof(source->used));
   offset += sizeof(source->used);

   //For each point, store their parameters
   for (Rox_Uint id = 0; id < source->used; id++)
   {
      for ( Rox_Sint flag_id = 0; flag_id < 32; flag_id++)
      {
         // flag
         memcpy(buffer + offset, &(source->data[id].flag_indices[flag_id]), sizeof(source->data[id].flag_indices[flag_id]));
         offset += sizeof(source->data[id].flag_indices[flag_id]);
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_ehid_dbindex_deserialize(Rox_DynVec_Ehid_DbIndex output, Rox_Char *buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0;
   Rox_Uint count;
   Rox_Ehid_DbIndex_Struct cur;

   if (!output || !buffer) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_dbindex_reset(output);

   // Read point counter
   memcpy(&count, buffer + offset, sizeof(count));
   offset += sizeof(count);

   for (Rox_Uint id = 0; id < count; id++)
   {
      for ( Rox_Sint idx = 0; idx < 32; idx++)
      {
         memcpy(&cur.flag_indices[idx], buffer + offset, sizeof(cur.flag_indices[idx]));
         offset += sizeof(cur.flag_indices[idx]);
      }
      rox_dynvec_ehid_dbindex_append(output, &cur);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_ehid_dbindex_get_structure_size(Rox_Uint *size, Rox_DynVec_Ehid_DbIndex source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint total_size = 0;
   Rox_Uint indice_size = 0;

   if ( size == 0 || source == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   *size = 0;

   indice_size = sizeof(source->data[0].flag_indices[0]) * 32;

   total_size  = sizeof(source->used);
   total_size += source->used * indice_size;

   *size = total_size;

function_terminate:
   return error;
}

