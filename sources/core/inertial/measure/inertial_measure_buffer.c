//==============================================================================
//
//    OPENROX   : File inertial_measure_buffer.h
//
//    Contents  : API of inertial measure buffer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "inertial_measure_buffer.h"
#include "inertial_measure_buffer_struct.h"

#include <core/inertial/measure/inertial_measure.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_imu_measure_buffer_new(Rox_Imu_Measure_Buffer *buffer, const Rox_Uint length)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Imu_Measure_Buffer ret = NULL;

   if(!buffer)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   *buffer = NULL;

   if(length == 0)
   {
      error = ROX_ERROR_BAD_SIZE;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   // Memory Allocation
   ret = (Rox_Imu_Measure_Buffer) rox_memory_allocate(sizeof(*ret), 1);
   if(!ret)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   // Data
   ret->length = length;
   ret->start_position = 0;
   ret->end_position = 0;
   ret->write_count = 0;
   ret->read_count = 0;

   // Memory allocation
   ret->data = (Rox_Imu_Measure *) rox_memory_allocate(sizeof(*ret->data), ret->length);
   if(ret->data == 0)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   for (Rox_Uint i = 0; i < ret->length; i++)
   {
      ret->data[i] = NULL;
   }

   for (Rox_Uint i = 0; i < ret->length; i++)
   {
      error = rox_imu_measure_new(&ret->data[i]);
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   *buffer = ret;

function_terminate:
   if(error) rox_imu_measure_buffer_del(&ret);

   return error;
}

Rox_ErrorCode rox_imu_measure_buffer_del(Rox_Imu_Measure_Buffer * buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Imu_Measure_Buffer todel = NULL;

   if(!buffer)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *buffer;
   *buffer = NULL;

   if(!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (todel->data)
   {
      for (Rox_Uint i = 0; i < todel->length; i++)
      {
         rox_imu_measure_del(&todel->data[i]);
      }

      rox_memory_delete(todel->data);
   }

function_terminate:

   rox_memory_delete(todel);

   return error;
}

Rox_ErrorCode rox_imu_measure_buffer_queue(Rox_Imu_Measure_Buffer buffer, Rox_Imu_Measure measure)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!buffer || !measure)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   if (!rox_imu_measure_buffer_full(buffer))
   {
      error = rox_imu_measure_copy(buffer->data[buffer->end_position], measure);
      ROX_ERROR_CHECK_TERMINATE(error)

      buffer->write_count++;
      buffer->end_position++;

      // Check if if end stays inside buffer
      if(buffer->end_position == buffer->length) buffer->end_position = 0;
   }
   else
   {
      error = ROX_ERROR_FULL_BUFFER;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_imu_measure_buffer_dequeue(Rox_Imu_Measure measure, Rox_Imu_Measure_Buffer buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!buffer || !measure)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Increment init
   if ( ! ( error = rox_imu_measure_buffer_empty(buffer) ) )
   {
      // Copy measure
      error = rox_imu_measure_copy(measure, buffer->data[buffer->start_position]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      buffer->start_position ++;
      buffer->read_count++;

      // Check if if init stays inside buffer
      if(buffer->start_position == buffer->length) buffer->start_position = 0;
   }
   else
   {
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_imu_measure_buffer_empty(const Rox_Imu_Measure_Buffer buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!buffer)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(buffer->read_count == buffer->write_count)
   { error = ROX_ERROR_EMPTY_BUFFER; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_imu_measure_buffer_full(const Rox_Imu_Measure_Buffer buffer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!buffer)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if((buffer->write_count - buffer->read_count) == buffer->length)
   { error = ROX_ERROR_FULL_BUFFER; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}
