//==============================================================================
//
//    OPENROX   : File imu.c
//
//    Contents  : Implementation of imu module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "imu.h"
#include "imu_struct.h"

#include <system/memory/memory.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/substract/substract.h>

#include <core/inertial/measure/inertial_measure_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_imu_new(Rox_Imu *inertial, const Rox_Float inertial_frequency)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Imu ret = NULL;

   if(!inertial) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}
   *inertial = NULL;

   if(inertial_frequency < 0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );} 

   // Memory Allocation 
   ret = (Rox_Imu)rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Measure 
   error = rox_imu_measure_new(&ret->ext_mea);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_imu_measure_new(&ret->cur_mea);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_imu_measure_new(&ret->pre_mea);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_imu_measure_new(&ret->unb_mea);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_imu_measure_buffer_new(&ret->buf, 512);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Bias 
   error = rox_array2d_double_new(&ret->ba, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new(&ret->bw, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Gravity vector 
   error = rox_array2d_double_new(&ret->g, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   // IMU pose and velocity 
   error = rox_frame_new(&ret->Fi_mea);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_frame_new(&ret->Fi_pre);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_frame_new(&ret->Fi_est);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_fillval(ret->ba, 0.0);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_fillval(ret->bw, 0.0);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   // Set the gravity vector to g = [0.0; 0.0; 9.81];
   error = rox_array2d_double_fillval(ret->g, 0.0);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_set_value(ret->g, 2, 0, 9.81);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Sample frequency
   ret->frequency = inertial_frequency;

   *inertial = ret;

function_terminate:
   if(error) rox_imu_del(&ret);
   return error;
}

Rox_ErrorCode rox_imu_del(Rox_Imu *inertial)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Imu todel = NULL;

   if(!inertial) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   todel = *inertial;
   *inertial = NULL;

   if(!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   rox_imu_measure_del(&todel->ext_mea);
   rox_imu_measure_del(&todel->cur_mea);
   rox_imu_measure_del(&todel->pre_mea);
   rox_imu_measure_del(&todel->unb_mea);
   rox_imu_measure_buffer_del(&todel->buf);
   rox_array2d_double_del(&todel->ba);
   rox_array2d_double_del(&todel->bw);
   rox_array2d_double_del(&todel->g);
   rox_frame_del(&todel->Fi_mea);
   rox_frame_del(&todel->Fi_pre);
   rox_frame_del(&todel->Fi_est);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_imu_set_measure(Rox_Imu inertial, const Rox_Double * A, const Rox_Double * W, const Rox_Double * M, Rox_Double timestamp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!inertial || !A || !W || !M) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dA = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dA, inertial->ext_mea->A );
   Rox_Double ** dW = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dW, inertial->ext_mea->W );
   Rox_Double ** dM = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer ( &dM, inertial->ext_mea->M );

   dA[0][0] = A[0];    dA[1][0] = A[1];    dA[2][0] = A[2];
   dW[0][0] = W[0];    dW[1][0] = W[1];    dW[2][0] = W[2];
   dM[0][0] = M[0];    dM[1][0] = M[1];    dM[2][0] = M[2];

   inertial->ext_mea->timestamp = timestamp;

   error = rox_imu_measure_buffer_queue(inertial->buf, inertial->ext_mea);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_imu_update_current_measure(const Rox_Imu inertial)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!inertial) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Copy the current inertial measure to previous measure 
   error  = rox_imu_measure_copy(inertial->pre_mea, inertial->cur_mea);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Dequeue measure 
   error = rox_imu_measure_buffer_dequeue(inertial->cur_mea, inertial->buf);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_imu_update_unbiased_measure(const Rox_Imu inertial, Rox_Imu_Measure measure)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!inertial || !measure) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   // Update unbiased measure 
   error = rox_array2d_double_substract(inertial->unb_mea->A, measure->A, inertial->ba);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_substract(inertial->unb_mea->W, measure->W, inertial->bw);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_imu_get_estimated_frame(Rox_Frame Fi_est, const Rox_Imu inertial)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!inertial || !Fi_est) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_frame_copy(Fi_est, inertial->Fi_est);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:   
   return error;
}