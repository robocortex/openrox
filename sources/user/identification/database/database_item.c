//==============================================================================
//
//    OPENROX   : File database_item.c
//
//    Contents  : Implementation of database_item module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "database_item.h"
#include "database_item_struct.h"

#include <string.h>

#include <system/time/timer.h>

#include <core/features/descriptors/ehid/ehid_viewpointbin.h>
#include <core/features/descriptors/ehid/ehid_viewpointbin_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/serialization/dynvec_ehid_point_serialization.h>
#include <inout/serialization/dynvec_ehid_dbindex_serialization.h>

Rox_ErrorCode rox_database_item_new(Rox_Database_Item * item)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Item ret = NULL;
   
   if(!item)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   *item = NULL;

   ret = (Rox_Database_Item) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->cols = 0;
   ret->rows = 0;

   ret->angle_max = 40.0; // in degrees
   ret->sigma = 4.0; // standard deviation of gaussian noise

   for ( Rox_Sint i = 0; i < 9; i++)
   {
      ret->scales[i] = (Rox_Double) (i+1);
   }

   ret->dbindices = NULL;
   error = rox_dynvec_ehid_dbindex_new(&ret->dbindices, 100);
   ROX_ERROR_CHECK_TERMINATE(error)

   ret->dbpoints = NULL;
   error = rox_dynvec_ehid_point_new(&ret->dbpoints, 100);
   ROX_ERROR_CHECK_TERMINATE(error)

   *item = ret;

function_terminate:
   if(error) rox_database_item_del(&ret);

   return error;
}

Rox_ErrorCode rox_database_item_del(Rox_Database_Item * item)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Database_Item todel = NULL;


   if(!item)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *item;
   *item = NULL;


   if(!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_del(&todel->dbpoints);
   rox_dynvec_ehid_dbindex_del(&todel->dbindices);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_item_learn_template (
   Rox_Database_Item item, 
   const Rox_Image image_template
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Sint nb_vpbins = 8; // 8 if we want 8 levels [was 9 before, i.e. 9 levels]
   const Rox_Sint nb_scales = 9; // must be : nb_scales >= nb_vpbins + 1
   // const Rox_Sint nb_vpbins = 9;
   // const Rox_Sint nb_scales = 10; // must be : nb_scales = nb_vpbins + 1

   const Rox_Double maxaffine = item->angle_max; // 40.0 by default
   const Rox_Double sigma = item->sigma; // 4.0 by default

   // Define the possible viewpoint bin
   Rox_Ehid_ViewpointBin vpbin[8]; //  [nb_vpbins] ;

   // Define the possible scales
   Rox_Double scales[9]; // [nb_scales]; //

   // Init pointers before anything else
   for(Rox_Sint iter = 0; iter < nb_vpbins; iter++)
   {
      vpbin[iter] = NULL;
   }

   if (!item || !image_template) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(0)
   {
      // old
      //scales[0] = 1.00; scales[1] = 1.33; scales[2] = 1.66; scales[3] = 2.00; scales[4] = 2.33; scales[5] = 2.66; scales[6] = 3.00; scales[7] = 3.33; scales[8] = 3.66; scales[9] = 4.00;

      //// test Scnheider
      //scales[0] = 0.25; scales[1] = 0.50; scales[2] = 1.00; scales[3] = 1.50; scales[4] = 2.00; scales[5] = 3.00; scales[6] = 4.00; scales[7] = 5.00; scales[8] = 6.00; scales[9] = 8.00;
   }
   else
   {
      for(Rox_Sint iter = 0; iter < nb_scales; iter++)
      {
         scales[iter] = item->scales[iter];
      }
   }

   // Reset the dynamic vectors
   rox_dynvec_ehid_point_reset(item->dbpoints);
   rox_dynvec_ehid_dbindex_reset(item->dbindices);

   Rox_Sint cols = 0, rows = 0;
   error = rox_image_get_size(&rows, &cols, image_template);

   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set template dimensions
   item->cols = cols;
   item->rows = rows;

   Rox_Sint iter = 0;

#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) private(iter) // schedule(dynamic, 8)  //schedule(static) //
#endif

   for (iter = 0; iter < nb_vpbins; iter++)
   {
      error = rox_ehid_viewpointbin_new ( &vpbin[iter], cols, rows, scales[iter], scales[iter+1], 0, maxaffine, sigma );
      // ROX_ERROR_CHECK_BREAK(error);
   }

   // Check here if fatal error then goto terminate. Cannot be done in OMP pragma
   ROX_ERROR_CHECK_TERMINATE ( error );
   
#ifdef ROX_USES_OPENMP
   #pragma omp parallel for schedule(dynamic) private(iter) // schedule(dynamic, 8)  //schedule(static) //
#endif

   for (iter = 0; iter < nb_vpbins; iter++)
   {
      error = rox_ehid_viewpointbin_process(vpbin[iter], image_template);
      // ROX_ERROR_CHECK_BREAK(error);
   }
   
   // Check here if fatal error then goto terminate. Cannot be done in OMP pragma
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Sint iter = 0; iter < nb_vpbins; iter++)
   {
      error = rox_dynvec_ehid_point_stack(item->dbpoints, vpbin[iter]->clustered);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_ehid_dbindex_stack(item->dbindices, vpbin[iter]->clustered_index);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:

   for(Rox_Sint iter = 0; iter < nb_vpbins; iter++)
   {
      rox_ehid_viewpointbin_del(&vpbin[iter]);
   }

   return error;
}

Rox_ErrorCode rox_database_item_save(const Rox_Char * filename, const Rox_Database_Item item)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   FILE * out = NULL;
   Rox_Char * ser = NULL;
   Rox_Uint size = 0;


   if(!filename || !item)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   out = fopen(filename, "wb");
   if(!out)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error  = rox_database_item_get_structure_size(&size, item);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ser = (Rox_Char *) rox_memory_allocate(sizeof(*ser), size);

   if(!ser)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Fill the buffer
   memset(ser, 0, size);

   error = rox_database_item_serialize(ser, item);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Write the buffer
   if (fwrite(ser, sizeof(*ser), size, out) != size)
   { error = ROX_ERROR_EXTERNAL; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   if(out) fclose(out);
   if(ser) rox_memory_delete(ser);

   return error;
}

Rox_ErrorCode rox_database_item_load (
   Rox_Database_Item item, 
   const Rox_Char * filename
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   FILE * in = NULL;
   Rox_Char *ser = NULL;
   Rox_Uint size = 0;


   if (!item || !filename)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_reset(item->dbpoints);
   rox_dynvec_ehid_dbindex_reset(item->dbindices);

   in = fopen(filename, "rb");

   if (!in)
   { error = ROX_ERROR_BAD_IOSTREAM; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Read the buffer length
   if (fread(&size, sizeof(size), 1, in) != 1)
   { error = ROX_ERROR_EXTERNAL; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Reset the cursor position
   fseek(in, 0, SEEK_SET);

   // Allocate buffer memory
   ser = (Rox_Char*) rox_memory_allocate(sizeof(*ser), size);
   if (!ser)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Read buffer
   if (fread(ser, sizeof(*ser), size, in) != size)
   { error = ROX_ERROR_EXTERNAL; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Deserialize buffer
   error = rox_database_item_deserialize(item, ser);
   if (error) ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (in) fclose(in);
   if (ser) rox_memory_delete(ser);

   return error;
}

Rox_ErrorCode rox_database_item_serialize ( 
   Rox_Char * buffer, 
   const Rox_Database_Item item
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0;
   Rox_Uint size = 0;

   if ( !buffer || !item ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Serialize the structure size
   error = rox_database_item_get_structure_size(&size, item);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Serialize the structure size
   memcpy(buffer + offset, &size, sizeof(size));
   offset += sizeof(size);

   // Serialize the template size
   memcpy(buffer + offset, &item->cols, sizeof(item->cols));
   offset += sizeof(item->cols);

   memcpy(buffer + offset, &item->rows, sizeof(item->rows));
   offset += sizeof(item->rows);

   // Serialize db points
   error = rox_dynvec_ehid_point_serialize(buffer + offset, item->dbpoints);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The buffer offset is equal to the dbpoint size
   error = rox_dynvec_ehid_point_get_structure_size(&size, item->dbpoints);
   ROX_ERROR_CHECK_TERMINATE ( error );

   offset += size;

   // Serialize db indices
   error = rox_dynvec_ehid_dbindex_serialize(buffer + offset, item->dbindices);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_item_deserialize (
   Rox_Database_Item item, 
   const Rox_Char * buffer
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0;
   Rox_Uint size = 0;
   Rox_Char * buffer_cur = (Rox_Char *) buffer; // Copy in a modifiable pointer


   if(!item || !buffer)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Read the buffer size
   memcpy(&size, buffer_cur + offset, sizeof(size));
   offset += sizeof(size);

   // Read the template size
   memcpy(&item->cols, buffer_cur + offset, sizeof(item->cols));
   offset += sizeof(item->cols);

   memcpy(&item->rows, buffer_cur + offset, sizeof(item->rows));
   offset += sizeof(item->rows);

   // Deserialize points
   error = rox_dynvec_ehid_point_deserialize(item->dbpoints, buffer_cur + offset);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The buffer offset is equal to the point size
   error = rox_dynvec_ehid_point_get_structure_size(&size, item->dbpoints);
   ROX_ERROR_CHECK_TERMINATE ( error );
   offset += size;

   // Deserialize indices
   error = rox_dynvec_ehid_dbindex_deserialize(item->dbindices, buffer_cur + offset);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_item_get_structure_size(Rox_Uint * size, const Rox_Database_Item item)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!size || !item) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *size = 0;

   Rox_Uint size_dbpoints = 0;
   error = rox_dynvec_ehid_point_get_structure_size(&size_dbpoints, item->dbpoints);

   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint size_dbindices = 0;
   error = rox_dynvec_ehid_dbindex_get_structure_size(&size_dbindices, item->dbindices);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *size = size_dbpoints + size_dbindices + sizeof(Rox_Uint) + 2*sizeof(Rox_Uint);

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_item_get_points_size(Rox_Uint * size, Rox_Database_Item database_item)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   error = rox_dynvec_ehid_point_get_used(size, database_item->dbpoints);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_item_get_points_copy(Rox_Point2D_Double points, Rox_Uint size, Rox_Database_Item database_item)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if(!points || !database_item)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the point list
   Rox_DynVec_Ehid_Point dbpoints = database_item->dbpoints;

   Rox_Ehid_Point_Struct * ehid_points = NULL;
   error = rox_dynvec_ehid_point_get_data_pointer(&ehid_points, dbpoints);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint nb_used = 0;
   error = rox_dynvec_ehid_point_get_used(&nb_used, dbpoints);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint i=0; i<nb_used; i++)
   {
      points[i].u = ehid_points[i].pos.u;
      points[i].v = ehid_points[i].pos.v;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_database_item_set_params(Rox_Database_Item item, const Rox_Double scales[9], const Rox_Double angle_max, const Rox_Double sigma)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!item)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (scales[0] >= 1.0)
   {
      item->scales[0] = scales[0];
   }

   for (Rox_Sint i = 1; i < 9; i++)
   {
      // Check if scales values are not decreasing
      if (scales[i] >= scales[i-1])
      {
         item->scales[i] = scales[i];
      }
   }

   if ((angle_max > 0.0) && (angle_max < 80.0))
   {
      item->angle_max = angle_max;
   }

   if (sigma > 0.0)
   {
      item->sigma = sigma;
   }

function_terminate:
   return error;
}
