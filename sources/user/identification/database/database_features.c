//==============================================================================
//
//    OPENROX   : File database_features.c
//
//    Contents  : Implementation of database_features module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <stdio.h>
#include <string.h>

#include "database_features.h"

#include <generated/dynvec_ehid_point_struct.h>

#include <inout/system/errors_print.h>
#include <inout/serialization/dynvec_ehid_point_serialization.h>


Rox_ErrorCode rox_database_features_new(Rox_Database_Features *features)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_dynvec_ehid_point_new(features, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_features_del(Rox_Database_Features *features)
{
   return rox_dynvec_ehid_point_del(features);
}

Rox_ErrorCode rox_database_features_serialize(Rox_Char *buffer, Rox_Database_Features features)
{
   return rox_dynvec_ehid_point_serialize(buffer, features);
}

Rox_ErrorCode rox_database_features_deserialize(Rox_Database_Features features, Rox_Char *buffer)
{
   return rox_dynvec_ehid_point_deserialize(features, buffer);
}

Rox_ErrorCode rox_database_features_get_structure_size(Rox_Uint *size, Rox_Database_Features features)
{
   return rox_dynvec_ehid_point_get_structure_size(size, features);
}

Rox_ErrorCode rox_database_features_save(Rox_Char * filename, Rox_Database_Features features)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;
   Rox_Char * ser = NULL;


   if(!filename || !features) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   out = fopen(filename, "wb");
   if (!out) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint size = 0;
   error  = rox_database_features_get_structure_size(&size, features);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ser = (Rox_Char *) rox_memory_allocate(sizeof(*ser), size);

   if(!ser) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Fill the buffer
   memset(ser, 0, size);

   error = rox_database_features_serialize(ser, features);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Write the buffer
   if (fwrite(ser, sizeof(*ser), size, out) != size)
   { error = ROX_ERROR_EXTERNAL; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   if(out) fclose(out);
   rox_memory_delete(ser);

   return error;
}

Rox_ErrorCode rox_database_features_load(Rox_Database_Features features, const Rox_Char * filename)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE * in = NULL;
   Rox_Char *ser = NULL;
   Rox_Uint size = 0;


   if (!features || !filename) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_reset(features);

   in = fopen(filename, "rb");

   if (!in) 
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Read the buffer length
   if (fread(&size, sizeof(size), 1, in) != 1)
   { error = ROX_ERROR_EXTERNAL; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Reset the cursor position
   fseek(in, 0, SEEK_SET);

   // Allocate buffer memory
   ser = (Rox_Char*)rox_memory_allocate(sizeof(*ser), size);

   if (!ser) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Read buffer 
   if(fread(ser, sizeof(*ser), size, in) != size) 
   {error = ROX_ERROR_EXTERNAL; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Deserialize buffer
   error = rox_database_features_deserialize(features, ser);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if(in) fclose(in);
   rox_memory_delete(ser);

   return error;
}

Rox_ErrorCode rox_database_features_get_points_size(Rox_Uint * size, Rox_Database_Features database_features)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_dynvec_ehid_point_get_used(size, database_features);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_features_get_points_copy(Rox_Point2D_Double points, Rox_Uint size, Rox_Database_Features database_features)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;



   if (!points || !database_features) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the point list
   Rox_Ehid_Point_Struct * ehid_points = NULL;
   error = rox_dynvec_ehid_point_get_data_pointer( &ehid_points, database_features);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint nb_used = 0;
   error = rox_dynvec_ehid_point_get_used(&nb_used, database_features);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (size != nb_used)
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint i=0; i<nb_used; i++)
   {
      points[i].u = ehid_points[i].pos.u;
      points[i].v = ehid_points[i].pos.v;
   }

function_terminate:
   return error;
}