//==============================================================================
//
//    OPENROX   : File dynvec_ehid_point_serialization.c
//
//    Contents  : Implementation of dynvec_ehid_point_serialization module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <string.h>
#include <generated/dynvec_ehid_point_struct.h>

#include <inout/system/errors_print.h>
#include <inout/serialization/dynvec_ehid_point_serialization.h>

#include "inout/system/print.h"

Rox_ErrorCode rox_dynvec_ehid_point_serialize(Rox_Char *buffer, Rox_DynVec_Ehid_Point source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint offset = 0;
   Rox_Float posu = 0.0, posv = 0.0, diru = 0.0, dirv = 0.0, scale = 0.0;

   if(!buffer || !source) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Store point count
   memcpy(buffer + offset, &(source->used), sizeof(source->used));
   offset += sizeof(source->used);

   // For each point, store their parameters
   for ( Rox_Uint id = 0; id < source->used; id++)
   {
        //  Cast data
      posu  = (Rox_Float)source->data[id].pos.u;
      posv  = (Rox_Float)source->data[id].pos.v;
      diru  = (Rox_Float)source->data[id].dir.u;
      dirv  = (Rox_Float)source->data[id].dir.v;
      scale = (Rox_Float)source->data[id].scale;

      //  Position
      memcpy(buffer + offset, &posu, sizeof(posu));
      offset += sizeof(posu);
      memcpy(buffer + offset, &posv, sizeof(posv));
      offset += sizeof(posv);

      //  Orientation
      memcpy(buffer + offset, &diru, sizeof(diru));
      offset += sizeof(diru);
      memcpy(buffer + offset, &dirv, sizeof(dirv));
      offset += sizeof(dirv);

      //  Scale
      memcpy(buffer + offset, &scale, sizeof(scale));
      offset += sizeof(scale);

      //  Index
      memcpy(buffer + offset, &source->data[id].index, sizeof(source->data[id].index));
      offset += sizeof(source->data[id].index);

      memcpy(buffer + offset, &source->data[id].dbid, sizeof(source->data[id].dbid));
      offset += sizeof(source->data[id].dbid);

      memcpy(buffer + offset, &source->data[id].refcount, sizeof(source->data[id].refcount));
      offset += sizeof(source->data[id].refcount);

      memcpy(buffer + offset, &source->data[id].uid, sizeof(source->data[id].uid));
      offset += sizeof(source->data[id].uid);

      //  Descriptor
      memcpy(buffer + offset, &(source->data[id].Description[0]), sizeof(source->data[id].Description[0]));
      offset += sizeof(source->data[id].Description[0]);
      memcpy(buffer + offset, &(source->data[id].Description[1]), sizeof(source->data[id].Description[1]));
      offset += sizeof(source->data[id].Description[1]);
      memcpy(buffer + offset, &(source->data[id].Description[2]), sizeof(source->data[id].Description[2]));
      offset += sizeof(source->data[id].Description[2]);
      memcpy(buffer + offset, &(source->data[id].Description[3]), sizeof(source->data[id].Description[3]));
      offset += sizeof(source->data[id].Description[3]);
      memcpy(buffer + offset, &(source->data[id].Description[4]), sizeof(source->data[id].Description[4]));
      offset += sizeof(source->data[id].Description[4]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_ehid_point_get_structure_size(Rox_Uint * size, Rox_DynVec_Ehid_Point source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint total_size = 0;
   Rox_Uint point_size = 0;

   if (size == 0 || source == 0)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *size = 0;

   //  Ehid point size
   //  Cast Rox_Double into Rox_Float
   point_size += sizeof(Rox_Float); //  posu
   point_size += sizeof(Rox_Float); //  posv
   point_size += sizeof(Rox_Float); //  diru
   point_size += sizeof(Rox_Float); //  dirv
   point_size += sizeof(Rox_Float); //  scale
   point_size += sizeof(Rox_Uint); //  index
   point_size += sizeof(Rox_Uint); //  dbid
   point_size += sizeof(Rox_Uint); //  refcount
   point_size += sizeof(Rox_Uint); //  uid

   point_size += 5*sizeof(Rox_Int64); //  Descriptor

   total_size  = sizeof(source->used);
   total_size += source->used * point_size;

   *size = total_size;

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_ehid_point_deserialize(Rox_DynVec_Ehid_Point output, Rox_Char * buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Point_Struct pt;
   Rox_Uint count = 0;
   Rox_Uint offset = 0;
   Rox_Float posu = 0, posv = 0, diru = 0, dirv = 0, scale = 0;

   if (output == 0 || buffer == 0)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_point_reset(output);

   //  Read point counter
   memcpy(&count, buffer + offset, sizeof(count));
   offset += sizeof(count);

   for ( Rox_Uint id = 0; id < count; id ++)
   {
      //  Position
      memcpy(&posu, buffer + offset, sizeof(posu));
      offset += sizeof(posu);
      memcpy(&posv, buffer + offset, sizeof(posv));
      offset += sizeof(posv);

      //  Orientation
      memcpy(&diru, buffer + offset, sizeof(diru));
      offset += sizeof(diru);
      memcpy(&dirv, buffer + offset, sizeof(dirv));
      offset += sizeof(dirv);

      //  Scale
      memcpy(&scale, buffer + offset, sizeof(scale));
      offset += sizeof(scale);

      //  Index
      memcpy(&pt.index, buffer + offset, sizeof(pt.index));
      offset += sizeof(pt.index);

      memcpy(&pt.dbid, buffer + offset, sizeof(pt.dbid));
      offset += sizeof(pt.dbid);

      memcpy(&pt.refcount, buffer + offset, sizeof(pt.refcount));
      offset += sizeof(pt.refcount);

      memcpy(&pt.uid, buffer + offset, sizeof(pt.uid));
      offset += sizeof(pt.uid);

      //  Descriptor
      memcpy(&(pt.Description[0]), buffer + offset, sizeof(pt.Description[0]));
      offset += sizeof(pt.Description[0]);
      memcpy(&(pt.Description[1]), buffer + offset, sizeof(pt.Description[1]));
      offset += sizeof(pt.Description[1]);
      memcpy(&(pt.Description[2]), buffer + offset, sizeof(pt.Description[2]));
      offset += sizeof(pt.Description[2]);
      memcpy(&(pt.Description[3]), buffer + offset, sizeof(pt.Description[3]));
      offset += sizeof(pt.Description[3]);
      memcpy(&(pt.Description[4]), buffer + offset, sizeof(pt.Description[4]));
      offset += sizeof(pt.Description[4]);

      pt.pos.u = (Rox_Double)posu;
      pt.pos.v = (Rox_Double)posv;
      pt.dir.u = (Rox_Double)diru;
      pt.dir.v = (Rox_Double)dirv;
      pt.scale = (Rox_Double)scale;

      rox_dynvec_ehid_point_append(output, &pt);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_ehid_point_save(const Rox_Char * filename, Rox_DynVec_Ehid_Point source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

// function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_ehid_point_print(Rox_DynVec_Ehid_Point source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Float posu = 0.0, posv = 0.0, diru = 0.0, dirv = 0.0, scale = 0.0;

   if (!source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Display number of ponits
   rox_log("used= %d \n", source->used);

   // For each point, store their parameters
   for ( Rox_Uint id = 0; id < source->used; id++)
   {
        //  Cast data
      posu  = (Rox_Float) source->data[id].pos.u;
      posv  = (Rox_Float) source->data[id].pos.v;
      diru  = (Rox_Float) source->data[id].dir.u;
      dirv  = (Rox_Float) source->data[id].dir.v;
      scale = (Rox_Float) source->data[id].scale;

      rox_log("posu, posv, diru, dirv, scale : %f, %f, %f, %f, %f \n", posu, posv, diru, dirv, scale);
      rox_log("index, dbid, refcount, uid : %d, %d, %d, %d \n", source->data[id].index, source->data[id].dbid, source->data[id].refcount, source->data[id].uid);
      rox_log("descriptor : %ld, %ld, %ld, %ld, %ld \n", source->data[id].Description[0], source->data[id].Description[1], source->data[id].Description[2], source->data[id].Description[3], source->data[id].Description[4]);
   }

function_terminate:
   return error;
}
