//==============================================================================
//
//    OPENROX   : File inertial_measure.h
//
//    Contents  : API of inertial measure module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "inertial_measure.h"
#include "inertial_measure_struct.h"

#include <system/memory/memory.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/substract/substract.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_imu_measure_new(Rox_Imu_Measure *measure)
{
   Rox_ErrorCode error;
   Rox_Imu_Measure ret = 0;

   if(!measure)    
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   
   *measure = 0;

   // Memory Allocation 
   ret = (Rox_Imu_Measure) rox_memory_allocate(sizeof(*ret), 1);
   if(!ret)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = rox_array2d_double_new(&ret->A, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new(&ret->W, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new(&ret->M, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Set default values 
   error = rox_array2d_double_fillval(ret->A, 0.0);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_fillval(ret->W, 0.0);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_fillval(ret->M, 0.0);
   ROX_ERROR_CHECK_TERMINATE(error)

   ret->timestamp = 0;
   *measure = ret;

function_terminate:
   if(error) rox_imu_measure_del(&ret);
   return error;
}

Rox_ErrorCode rox_imu_measure_copy(Rox_Imu_Measure out, Rox_Imu_Measure inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!out || !inp)    
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }

   error = rox_array2d_double_copy(out->A, inp->A);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_copy(out->W, inp->W);
   ROX_ERROR_CHECK_TERMINATE(error)
  
   error = rox_array2d_double_copy(out->M, inp->M);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   out->timestamp = inp->timestamp;

function_terminate:
   return error;
}

Rox_ErrorCode rox_imu_measure_del(Rox_Imu_Measure *measure)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   Rox_Imu_Measure todel;

   if(!measure) 
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   
   todel = *measure;
   *measure = 0;

   if(!todel) 
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }
    rox_array2d_double_del(&todel->A);
   rox_array2d_double_del(&todel->W);
   rox_array2d_double_del(&todel->M);
   rox_memory_delete(todel);

function_terminate:
   return error;
}
