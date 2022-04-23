//==============================================================================
//
//    OPENROX   : File calibration_vision_inertial.c
//
//    Contents  : Implementation of calibration_vision_inertial module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "calibration_vision_inertial.h"
#include <core/inertial/measure/inertial_measure_struct.h>

#include <stdio.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/transpose/transpose.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/maths/linalg/matso3.h>
#include <baseproc/maths/linalg/generators/algso3.h>

#include <inout/numeric/array2d_print.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_calibration_vision_inertial_new(Rox_Calibration_Vision_Inertial *obj, enum Rox_Calibration_Vision_Inertial_Parameters params)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_Vision_Inertial ret;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret = (Rox_Calibration_Vision_Inertial)rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Set pointers to 0 
   ret->ogi_hat = 0;
   ret->ogi_pre = 0;
   ret->ogi_meas = 0;
   ret->w = 0;
   ret->f = 0;
   ret->a = 0;
   ret->w_unb = 0;
   ret->a_unb = 0;

   ret->iTv_meas = 0;
   ret->iRv_meas = 0;
   ret->itv_meas = 0;

   ret->oTv_meas = 0;
   ret->oRv_meas = 0;
   ret->otv_meas = 0;

   ret->oTi_meas = 0;
   ret->oRi_meas = 0;
   ret->oti_meas = 0;
   
   ret->iTv_hat = 0;
   ret->iRv_hat = 0;
   ret->itv_hat = 0;

   ret->oTi_hat = 0;
   ret->oRi_hat = 0;
   ret->oti_hat = 0;
   ret->ovi_hat = 0;

   ret->bw_hat = 0;
   ret->ba_hat = 0;

   ret->bw_pre = 0;
   ret->ba_pre = 0;

   ret->oTv_pre = 0;
   ret->oRv_pre = 0;
   ret->otv_pre = 0;

   ret->oTi_pre = 0;
   ret->oRi_pre = 0;
   ret->oti_pre = 0;
   ret->ovi_pre = 0;

   ret->oTi_pre_copy = 0;
   ret->ovi_pre_copy = 0;
   ret->oTi_hat_copy = 0;
   ret->ovi_hat_copy = 0;

   ret->iTv_pre = 0;
   ret->iRv_pre = 0;
   ret->itv_pre = 0;

   ret->oTv_err = 0;
   ret->oRv_err = 0;
   ret->otv_err = 0;
   ret->oTi_err = 0;
   ret->oRi_err = 0;
   ret->oti_err = 0;
   
   ret->c_rv = 0;
   ret->c_ri = 0;
   ret->c_bw = 0;
   ret->c_tv = 0;
   ret->c_ti = 0;
   ret->c_vi = 0;
   ret->c_ba = 0;
   ret->c_gi = 0;

   ret->wbuf4x4 = 0;
   ret->wbuf3x3 = 0;
   ret->wbuf3x1 = 0;
   ret->Pa = 0;
   ret->Sk = 0;

   ret->cur_measure = 0;
   ret->pre_measure = 0;

   error = rox_array2d_double_new(&ret->ogi_meas, 3, 1);
   error = rox_array2d_double_new(&ret->ogi_pre, 3, 1);
   error = rox_array2d_double_new(&ret->ogi_hat, 3, 1);
   error = rox_array2d_double_new(&ret->f, 3, 1);
   error = rox_array2d_double_new(&ret->a, 3, 1);
   error = rox_array2d_double_new(&ret->w, 3, 1);
   error = rox_array2d_double_new(&ret->a_unb, 3, 1);
   error = rox_array2d_double_new(&ret->w_unb, 3, 1);

   error = rox_array2d_double_new(&ret->iTv_meas, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->iRv_meas, ret->iTv_meas, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->itv_meas, ret->iTv_meas, 0, 3, 3, 1);

   error = rox_array2d_double_new(&ret->oTv_meas, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->oRv_meas, ret->oTv_meas, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->otv_meas, ret->oTv_meas, 0, 3, 3, 1);

   error = rox_array2d_double_new(&ret->oTi_meas, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->oRi_meas, ret->oTi_meas, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->oti_meas, ret->oTi_meas, 0, 3, 3, 1);
   
   error = rox_array2d_double_new(&ret->iTv_hat, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->iRv_hat, ret->iTv_hat, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->itv_hat, ret->iTv_hat, 0, 3, 3, 1);

   error = rox_array2d_double_new(&ret->oTi_hat, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->oRi_hat, ret->oTi_hat, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->oti_hat, ret->oTi_hat, 0, 3, 3, 1);
   error = rox_array2d_double_new(&ret->ovi_hat, 3, 1);

   error = rox_array2d_double_new(&ret->bw_hat, 3, 1);
   error = rox_array2d_double_new(&ret->ba_hat, 3, 1);

   error = rox_array2d_double_new(&ret->bw_pre, 3, 1);
   error = rox_array2d_double_new(&ret->ba_pre, 3, 1);

   error = rox_array2d_double_new(&ret->oTv_pre, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->oRv_pre, ret->oTv_pre, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->otv_pre, ret->oTv_pre, 0, 3, 3, 1);

   error = rox_array2d_double_new(&ret->oTi_pre, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->oRi_pre, ret->oTi_pre, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->oti_pre, ret->oTi_pre, 0, 3, 3, 1);
   error = rox_array2d_double_new(&ret->ovi_pre, 3, 1);

   error = rox_array2d_double_new(&ret->oTi_pre_copy, 4, 4);
   error = rox_array2d_double_new(&ret->ovi_pre_copy, 3, 1);
   error = rox_array2d_double_new(&ret->oTi_hat_copy, 4, 4);
   error = rox_array2d_double_new(&ret->ovi_hat_copy, 3, 1);

   error = rox_array2d_double_new(&ret->iTv_pre, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->iRv_pre, ret->iTv_pre, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->itv_pre, ret->iTv_pre, 0, 3, 3, 1);

   error = rox_array2d_double_new(&ret->oTv_err, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->oRv_err, ret->oTv_err, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->otv_err, ret->oTv_err, 0, 3, 3, 1);
   
   error = rox_array2d_double_new(&ret->oTi_err, 4, 4);
   error = rox_array2d_double_new_subarray2d(&ret->oRi_err, ret->oTi_err, 0, 0, 3, 3);
   error = rox_array2d_double_new_subarray2d(&ret->oti_err, ret->oTi_err, 0, 3, 3, 1);

   error = rox_array2d_double_new(&ret->c_rv, 3, 1);
   error = rox_array2d_double_new(&ret->c_ri, 3, 1);
   error = rox_array2d_double_new(&ret->c_bw, 3, 1);
   error = rox_array2d_double_new(&ret->c_tv, 3, 1);
   error = rox_array2d_double_new(&ret->c_ti, 3, 1);
   error = rox_array2d_double_new(&ret->c_vi, 3, 1);
   error = rox_array2d_double_new(&ret->c_ba, 3, 1);
   error = rox_array2d_double_new(&ret->c_gi, 3, 1);

   error = rox_array2d_double_new(&ret->wbuf4x4, 4, 4);
   error = rox_array2d_double_new(&ret->wbuf3x3, 3, 3);
   error = rox_array2d_double_new(&ret->wbuf3x1, 3, 1);
   error = rox_array2d_double_new(&ret->Pa, 3, 1);
   error = rox_array2d_double_new(&ret->Sk, 3, 3);

   error = rox_imu_measure_new(&ret->pre_measure);
   error = rox_imu_measure_new(&ret->cur_measure);

   ret->first_measure = 1;
   ret->cur_video_timestamp = 0.0;
   ret->pre_video_timestamp = 0.0;

   error = rox_calibration_vision_inertial_set_default_values(ret);
   error = rox_calibration_vision_inertial_init_gains(ret, 0.5, 15, 0.2, 0.8, 15);

   // set function pointers 
   switch(params)
   {
      case Rox_Calibration_Vision_Inertial_bw_ri:
      {
         ret->_fptr_make_predictions = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_make_predictions_bw_ri;
         ret->_fptr_nonlin_observer = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_nonlin_observer_bw_ri;
         break;
      }
      
      case Rox_Calibration_Vision_Inertial_bw_ri_rv:
      {
         ret->_fptr_make_predictions = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_make_predictions_bw_ri_rv;
         ret->_fptr_nonlin_observer = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_nonlin_observer_bw_ri_rv;
         break;
      }
      
      case Rox_Calibration_Vision_Inertial_bw_ba_ri_ti_vi:
      {
         ret->_fptr_make_predictions = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi;
         ret->_fptr_nonlin_observer = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi;
         break;
      }
      
      case Rox_Calibration_Vision_Inertial_bw_ba_ri_ti_vi_rv_tv:
      {
         ret->_fptr_make_predictions = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi_rv_tv;
         ret->_fptr_nonlin_observer = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi_rv_tv;
         break;
      }
      
      case Rox_Calibration_Vision_Inertial_bw_ba_ri_ti_vi_gi:
      {
         ret->_fptr_make_predictions = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi_gi;
         ret->_fptr_nonlin_observer = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi_gi;
         break;
      }
      
      case Rox_Calibration_Vision_Inertial_bw_ba_ri_ti_vi_rv_tv_gi:
      {
         ret->_fptr_make_predictions = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi_rv_tv_gi;
         ret->_fptr_nonlin_observer = (Rox_ErrorCode (*)(struct Rox_Calibration_Vision_Inertial_Struct *)) rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi_rv_tv_gi;
         break;
      }
      
      default:
      {
         error = ROX_ERROR_INVALID_VALUE;
         goto function_terminate;
      }
   }
   *obj = ret;

function_terminate:
   if(error) rox_calibration_vision_inertial_del(&ret);

   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_del(Rox_Calibration_Vision_Inertial *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Calibration_Vision_Inertial todel = NULL;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if(!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_array2d_double_del(&todel->ogi_meas);
   rox_array2d_double_del(&todel->ogi_pre);
   rox_array2d_double_del(&todel->ogi_hat);
   rox_array2d_double_del(&todel->f);
   rox_array2d_double_del(&todel->a);
   rox_array2d_double_del(&todel->w);
   rox_array2d_double_del(&todel->a_unb);
   rox_array2d_double_del(&todel->w_unb);

   rox_array2d_double_del(&todel->iTv_meas);
   rox_array2d_double_del(&todel->iRv_meas);
   rox_array2d_double_del(&todel->itv_meas);

   rox_array2d_double_del(&todel->oTv_meas);
   rox_array2d_double_del(&todel->oRv_meas);
   rox_array2d_double_del(&todel->otv_meas);

   rox_array2d_double_del(&todel->oTi_meas);
   rox_array2d_double_del(&todel->oRi_meas);
   rox_array2d_double_del(&todel->oti_meas);
   
   rox_array2d_double_del(&todel->iTv_hat);
   rox_array2d_double_del(&todel->iRv_hat);
   rox_array2d_double_del(&todel->itv_hat);

   rox_array2d_double_del(&todel->oTi_hat);
   rox_array2d_double_del(&todel->oRi_hat);
   rox_array2d_double_del(&todel->oti_hat);
   rox_array2d_double_del(&todel->ovi_hat);

   rox_array2d_double_del(&todel->bw_hat);
   rox_array2d_double_del(&todel->ba_hat);

   rox_array2d_double_del(&todel->bw_pre);
   rox_array2d_double_del(&todel->ba_pre);

   rox_array2d_double_del(&todel->oTv_pre);
   rox_array2d_double_del(&todel->oRv_pre);
   rox_array2d_double_del(&todel->otv_pre);

   rox_array2d_double_del(&todel->oTi_pre);
   rox_array2d_double_del(&todel->oRi_pre);
   rox_array2d_double_del(&todel->oti_pre);
   rox_array2d_double_del(&todel->ovi_pre);

   rox_array2d_double_del(&todel->oTi_pre_copy);
   rox_array2d_double_del(&todel->ovi_pre_copy);
   rox_array2d_double_del(&todel->oTi_hat_copy);
   rox_array2d_double_del(&todel->ovi_hat_copy);

   rox_array2d_double_del(&todel->iTv_pre);
   rox_array2d_double_del(&todel->iRv_pre);
   rox_array2d_double_del(&todel->itv_pre);

   rox_array2d_double_del(&todel->oTv_err);
   rox_array2d_double_del(&todel->oRv_err);
   rox_array2d_double_del(&todel->otv_err);

   rox_array2d_double_del(&todel->oTi_err);
   rox_array2d_double_del(&todel->oRi_err);
   rox_array2d_double_del(&todel->oti_err);
   
   rox_array2d_double_del(&todel->c_rv);
   rox_array2d_double_del(&todel->c_ri);
   rox_array2d_double_del(&todel->c_bw);
   rox_array2d_double_del(&todel->c_tv);
   rox_array2d_double_del(&todel->c_ti);
   rox_array2d_double_del(&todel->c_vi);
   rox_array2d_double_del(&todel->c_ba);
   rox_array2d_double_del(&todel->c_gi);
 
   rox_array2d_double_del(&todel->wbuf3x3);
   rox_array2d_double_del(&todel->wbuf4x4);
   rox_array2d_double_del(&todel->wbuf3x1);
   rox_array2d_double_del(&todel->Pa);
   rox_array2d_double_del(&todel->Sk);
   rox_imu_measure_del(&todel->cur_measure);
   rox_imu_measure_del(&todel->pre_measure);

   rox_memory_delete(todel);
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_init_gains(Rox_Calibration_Vision_Inertial obj, Rox_Double t1, Rox_Double t2, Rox_Double t3, Rox_Double t4, Rox_Double t5)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double k_ri, k_bw, k_rv, k_ti, k_vi, k_ba, k_tv, k_gi;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   if(t1 < 0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   if(t2 < 0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   if(t3 < 0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   if(t4 < 0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   if(t5 < 0) {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}

    k_ri = 3.0 * (t1+t2) / (t1*t2);
    k_bw = 9.0 / (t1*t2);
    k_rv = k_ri;
    k_ti = 3.0 * ((t3*t4) + (t3*t5) + (t4*t5))/(t3*t4*t5);
    k_vi = 9.0 * (t3 + t4 + t5)/(t3*t4*t5);
    k_ba = 27.0 / (t3*t4*t5);
    k_tv = k_ti;
    k_gi = (k_ti * k_vi) / 10.0 - k_ba; 

    obj->k_ri = k_ri;
    obj->k_bw = k_bw;
    obj->k_rv = k_rv;
    obj->k_ti = k_ti;
    obj->k_vi = k_vi;
    obj->k_ba = k_ba;
    obj->k_tv = k_tv;
    obj->k_gi = k_gi;

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_set_inertial_measure(Rox_Calibration_Vision_Inertial obj, Rox_Array2D_Double f, Rox_Array2D_Double w, Rox_Double timestamp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj || !f || !w) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
    
   // Copy data 
   error = rox_imu_measure_copy(obj->pre_measure, obj->cur_measure);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->cur_measure->A, f);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->cur_measure->W, w);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->f, f);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->w, w);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   obj->cur_measure->timestamp = timestamp;
    
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_compute_predictions(Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Compute prediction 
   error = obj->_fptr_make_predictions(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_compute_asynchronous_predictions(Rox_Calibration_Vision_Inertial obj, Rox_Array2D_Double f, Rox_Array2D_Double w, Rox_Double timestamp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj || !f || !w) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // If it is the first measure, the prediction cannot be computed 
   if(obj->first_measure)
   {
      // Copy data 
      error = rox_array2d_double_copy(obj->cur_measure->A, f);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_copy(obj->cur_measure->W, w);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      obj->cur_measure->timestamp = timestamp;

      obj->first_measure = 0;
       
      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   // Copy data 
   error = rox_imu_measure_copy(obj->pre_measure, obj->cur_measure);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->cur_measure->A, f);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->cur_measure->W, w);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->cur_measure->timestamp = timestamp;

   // Copy data 
   error = rox_array2d_double_copy(obj->w, obj->pre_measure->W);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(obj->f, obj->pre_measure->A);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute dt 
   if(obj->cur_video_timestamp >= obj->pre_measure->timestamp)
   {
      obj->dt = obj->cur_measure->timestamp - obj->cur_video_timestamp;
   }
   else
   {
      obj->dt = obj->cur_measure->timestamp - obj->pre_measure->timestamp;
   }

   // Compute prediction 
   error = obj->_fptr_make_predictions(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_compute_corrections(Rox_Calibration_Vision_Inertial obj, Rox_Array2D_Double oTv_meas)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj || !oTv_meas) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_copy(obj->oTv_meas, oTv_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We use the last measure to apply the corrections 
   error = rox_array2d_double_copy(obj->w, obj->cur_measure->W);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(obj->f, obj->cur_measure->A);
   ROX_ERROR_CHECK_TERMINATE ( error );
     
   error = obj->_fptr_nonlin_observer(obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_compute_asynchronous_corrections(Rox_Calibration_Vision_Inertial obj, Rox_Array2D_Double oTv_meas, Rox_Double timestamp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj || !oTv_meas) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   // Copy data 
   error = rox_array2d_double_copy(obj->oTv_meas, oTv_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->pre_video_timestamp = obj->cur_video_timestamp;
   obj->cur_video_timestamp = timestamp;

   // Check timestamps

   // First case: imu and video timestamps are equals -> compute corrections 
   if(obj->cur_video_timestamp == obj->cur_measure->timestamp)
   {
      // We use the last measure to apply the corrections 
      error = rox_array2d_double_copy(obj->w, obj->cur_measure->W);
      ROX_ERROR_CHECK_TERMINATE ( error );
         
      error = rox_array2d_double_copy(obj->f, obj->cur_measure->A);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = obj->_fptr_nonlin_observer(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = ROX_ERROR_NONE;
      goto function_terminate;
   }
   // Second case: imu timestamp is lower than video timestamp 
   else if(obj->cur_measure->timestamp < obj->cur_video_timestamp)
   {
      // We use the last measure to apply the corrections 
      error = rox_array2d_double_copy(obj->w, obj->cur_measure->W);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_copy(obj->f, obj->cur_measure->A);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute dt 
      obj->dt = obj->cur_video_timestamp - obj->cur_measure->timestamp;

      // Compute prediction 
      error = obj->_fptr_make_predictions(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute corrections 
      error = obj->_fptr_nonlin_observer(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   // Third case: imu timestamp is higher than video timestamp 
   else
   {
      // Get previous prediction 
      error = rox_array2d_double_copy(obj->oTi_pre, obj->oTi_pre_copy);
      ROX_ERROR_CHECK_TERMINATE ( error );
 
      error = rox_array2d_double_copy(obj->ovi_pre, obj->ovi_pre_copy);
      ROX_ERROR_CHECK_TERMINATE ( error );
 
      error = rox_array2d_double_copy(obj->oTi_hat, obj->oTi_hat_copy);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_copy(obj->ovi_hat, obj->ovi_hat_copy);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // We use the last measure to apply the corrections 
      error = rox_array2d_double_copy(obj->w, obj->pre_measure->W);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_copy(obj->f, obj->pre_measure->A);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute dt 
      obj->dt = obj->cur_video_timestamp - obj->pre_measure->timestamp;
       
      // Compute prediction 
      error = obj->_fptr_make_predictions(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute corrections 
      error = obj->_fptr_nonlin_observer(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute dt 
      obj->dt = obj->cur_measure->timestamp - obj->cur_video_timestamp;
       
      // Compute prediction 
      error = obj->_fptr_make_predictions(obj);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = ROX_ERROR_NONE;
      goto function_terminate;
   }
    
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ri(Rox_Calibration_Vision_Inertial obj)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    // TODO: to be replaced by a constant 
    Rox_Double dt = 0.01;

    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    Rox_Double ** dPa = NULL;
    error = rox_array2d_double_get_data_pointer_to_pointer(&dPa, obj->Pa);
    ROX_ERROR_CHECK_TERMINATE ( error );

    Rox_Double ** dSk = NULL;
    error = rox_array2d_double_get_data_pointer_to_pointer(&dSk, obj->Sk);
    ROX_ERROR_CHECK_TERMINATE ( error );

    // Estimation error 
    // oRi_meas = oRv_meas * inv(iTv)
    error = rox_array2d_double_mulmatmattrans(obj->oRi_meas, obj->oRv_meas, obj->iRv_meas);
    ROX_ERROR_CHECK_TERMINATE ( error );

    //oRi_error = oRi_mes * inv(oRi_pre);
    error = rox_array2d_double_mulmatmattrans(obj->oRi_err, obj->oRi_meas, obj->oRi_pre);
    ROX_ERROR_CHECK_TERMINATE ( error );

    //Projection on so3 Lie algebra
    //Pa_oRi_error = skew(oRi_err - transpose(oRi_err))/2;
    error = rox_array2d_double_transpose(obj->wbuf3x3, obj->oRi_err);
    ROX_ERROR_CHECK_TERMINATE ( error );
    error = rox_array2d_double_substract(obj->Sk, obj->oRi_err, obj->wbuf3x3);
    ROX_ERROR_CHECK_TERMINATE ( error );
    error = rox_array2d_double_scale(obj->Sk, obj->Sk, 0.5);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    dPa[0][0] = dSk[2][1];
    dPa[1][0] = dSk[0][2];
    dPa[2][0] = dSk[1][0];
 
    // Innovation terms
    // c_ri = +k(1)*transpose(oRi_pre)*Pa_oRi_err;
    error = rox_array2d_double_mulmattransmat(obj->c_ri, obj->oRi_pre, obj->Pa);
     ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, obj->k_ri);
     ROX_ERROR_CHECK_TERMINATE ( error );
   
    // c_bw = -k(2)*transpose(oRi_pre)*Pa_oRi_err;
    error = rox_array2d_double_mulmattransmat(obj->c_bw, obj->oRi_pre, obj->Pa);
     ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, -obj->k_bw);
     ROX_ERROR_CHECK_TERMINATE ( error );
   
    // State update
    //bw_hat =  bw_pre +          c_bw*dt ;
    //  oRi_hat = oRi_pre * expmSO3(c_ri*dt);
    error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, dt);
     ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, dt);
      ROX_ERROR_CHECK_TERMINATE ( error );
  
    error = rox_array2d_double_add(obj->bw_hat, obj->bw_pre, obj->c_bw);
      ROX_ERROR_CHECK_TERMINATE ( error );
  
    error = rox_array2d_double_fillunit(obj->wbuf3x3);
     ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_matso3_update_right(obj->wbuf3x3, obj->c_ri);
     ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(obj->oRi_hat, obj->oRi_pre, obj->wbuf3x3);
     ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ri_rv(Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // TODO: to be replaced by a constant 
   Rox_Double dt = 0.01;

   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Double ** dPa = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dPa, obj->Pa);
      ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dSk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dSk, obj->Sk);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimation error 
   // oRv_pre = oRi_pre * iRv_pre;
   error = rox_array2d_double_mulmatmat(obj->oRv_pre, obj->oRi_pre, obj->iRv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //oRv_error = (oRv_mes * inv(oRv_pre);
   error = rox_array2d_double_mulmatmattrans(obj->oRv_err, obj->oRv_meas, obj->oRv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Projection on so3 Lie algebra
   error = rox_array2d_double_transpose(obj->wbuf3x3, obj->oRv_err);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_substract(obj->Sk, obj->oRv_err, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->Sk, obj->Sk, 0.5);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   dPa[0][0] = dSk[2][1];
   dPa[1][0] = dSk[0][2];
   dPa[2][0] = dSk[1][0];

   //Unbiased measure
   error = rox_array2d_double_substract(obj->w_unb, obj->w, obj->bw_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Innovation terms
   //c_rv = +k(3)*transpose(oRv_pre)*skew(Pa_oRv_err)*oRi_pre*(w_mes-bw_pre);
   error = rox_array2d_double_mulmatmat(obj->wbuf3x1, obj->oRi_pre, obj->w_unb);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmattransmat(obj->wbuf3x3, obj->oRv_pre, obj->Sk);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->c_rv, obj->wbuf3x3, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_rv, obj->c_rv, obj->k_rv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_ri = +k(1)*transpose(oRi_pre)*Pa_oRv_err - iRv_pre*c_rv;
   error = rox_array2d_double_mulmatmat(obj->wbuf3x1, obj->iRv_pre, obj->c_rv);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmattransmat(obj->c_ri, obj->oRi_pre, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, obj->k_ri);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_substract(obj->c_ri, obj->c_ri, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_bw = -k(2)*transpose(oRi_pre)*Pa_oRv_err;
   error = rox_array2d_double_mulmattransmat(obj->c_bw, obj->oRi_pre, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, -obj->k_bw);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // State update 
   //bw_hat =  bw_pre +          c_bw*dt ;
   //  oRi_hat = oRi_pre * expmSO3(c_ri*dt);
   //  iRv_hat = iRv_pre * expmSO3(c_rv*dt);

   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_rv, obj->c_rv, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_add(obj->bw_hat, obj->bw_pre, obj->c_bw);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillunit(obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matso3_update_right(obj->wbuf3x3, obj->c_ri);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->oRi_hat, obj->oRi_pre, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matso3_update_right(obj->wbuf3x3, obj->c_rv);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->iRv_hat, obj->iRv_pre, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi(Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double **dd;

   // TODO: to be replaced by a constant 
   Rox_Double dt = 0.01;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Double **dPa = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dPa, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   Rox_Double **dSk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dSk, obj->Sk);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimation error 
   // oTi_meas = oTv_meas * inv(iTv)
   error = rox_array2d_double_svdinverse(obj->wbuf4x4, obj->iTv_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmattrans(obj->oTi_meas, obj->oTv_meas, obj->wbuf4x4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //oRi_error = oRi_mes * inv(oRi_pre);
   error = rox_array2d_double_mulmatmattrans(obj->oRi_err, obj->oRi_meas, obj->oRi_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // oti_error = oti_mes - oti_pre;
   error = rox_array2d_double_substract(obj->oti_err, obj->oti_meas, obj->oti_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Projection on so3 Lie algebra
   error = rox_array2d_double_transpose(obj->wbuf3x3, obj->oRi_err);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_substract(obj->Sk, obj->oRi_err, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->Sk, obj->Sk, 0.5);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   dPa[0][0] = dSk[2][1];
   dPa[1][0] = dSk[0][2];
   dPa[2][0] = dSk[1][0];

   //Unbiased measure
   error = rox_array2d_double_substract(obj->w_unb, obj->w, obj->bw_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Innovation terms
   //c_ri = c_ri = +k(1)*transpose(oRi_pre)*Pa_oRi_err;
   error = rox_array2d_double_mulmattransmat(obj->c_ri, obj->oRi_pre, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, obj->k_ri);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_bw = -k(2)*transpose(oRi_pre)*Pa_oRi_err;
   error = rox_array2d_double_mulmattransmat(obj->c_bw, obj->oRi_pre, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, -obj->k_bw);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_ti = +k(3)*oti_err;
   error = rox_array2d_double_scale(obj->c_ti, obj->oti_err, obj->k_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_vi = +k_vi * oti_err;
   error = rox_array2d_double_scale(obj->c_vi, obj->oti_err, obj->k_vi);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_ba = -k_ba * (eye(3,3)+(1/k_ti)*skew(w_mes-bw_pre))*transpose(oRi_pre)*oti_err
   error = rox_linalg_so3generator(obj->Sk, obj->w_unb);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_mulmattransmat(obj->wbuf3x1, obj->oRi_pre, obj->oti_err);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_scale(obj->wbuf3x3, obj->Sk, 1.0 / obj->k_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dd, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   dd[0][0] += 1.0; dd[1][1] += 1.0; dd[2][2] += 1.0;
    
   error = rox_array2d_double_mulmatmat(obj->c_ba, obj->wbuf3x3, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ba, obj->c_ba, -obj->k_ba);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // State update 
   //bw_hat =  bw_pre +          c_bw*dt ;
   //  ba_hat =  ba_pre +          c_ba*dt ;
   //  oRi_hat = oRi_pre * expmSO3(c_ri*dt);
   //  oti_hat = oti_pre +          c_ti*dt ;
   //  ovi_hat = ovi_pre +          c_vi*dt ;

   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ba, obj->c_ba, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ti, obj->c_ti, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_scale(obj->c_vi, obj->c_vi, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_add(obj->bw_hat, obj->bw_pre, obj->c_bw);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_add(obj->ba_hat, obj->ba_pre, obj->c_ba);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_add(obj->oti_hat, obj->oti_pre, obj->c_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_add(obj->ovi_hat, obj->ovi_pre, obj->c_vi);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillunit(obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_matso3_update_right(obj->wbuf3x3, obj->c_ri);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->oRi_hat, obj->oRi_pre, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi_gi(Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double **dd;

   // TODO: to be replaced by a constant 
   Rox_Double dt = 0.01;

   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double **dPa = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dPa, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double **dSk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dSk, obj->Sk);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimation error 
   // oTi_meas = oTv_meas * inv(iTv)
   error = rox_array2d_double_svdinverse(obj->wbuf4x4, obj->iTv_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmatmattrans(obj->oTi_meas, obj->oTv_meas, obj->wbuf4x4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //oRi_error = oRi_mes * inv(oRi_pre);
   error = rox_array2d_double_mulmatmattrans(obj->oRi_err, obj->oRi_meas, obj->oRi_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // oti_error = oti_mes - oti_pre;
   error = rox_array2d_double_substract(obj->oti_err, obj->oti_meas, obj->oti_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Projection on so3 Lie algebra
   error = rox_array2d_double_transpose(obj->wbuf3x3, obj->oRi_err);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_substract(obj->Sk, obj->oRi_err, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   error = rox_array2d_double_scale(obj->Sk, obj->Sk, 0.5);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   dPa[0][0] = dSk[2][1];
   dPa[1][0] = dSk[0][2];
   dPa[2][0] = dSk[1][0];

   //Unbiased measure
   error = rox_array2d_double_substract(obj->w_unb, obj->w, obj->bw_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Innovation terms
   //c_ri = c_ri = +k(1)*transpose(oRi_pre)*Pa_oRi_err;
   error = rox_array2d_double_mulmattransmat(obj->c_ri, obj->oRi_pre, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, obj->k_ri);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_bw = -k(2)*transpose(oRi_pre)*Pa_oRi_err;
   error = rox_array2d_double_mulmattransmat(obj->c_bw, obj->oRi_pre, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, -obj->k_bw);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_ti = +k(3)*oti_err;
   error = rox_array2d_double_scale(obj->c_ti, obj->oti_err, obj->k_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_vi = +k_vi * oti_err;
   error = rox_array2d_double_scale(obj->c_vi, obj->oti_err, obj->k_vi);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_ba = -k_ba * (eye(3,3)+(1/k_ti)*skew(w_mes-bw_pre))*transpose(oRi_pre)*oti_err
   error = rox_linalg_so3generator(obj->Sk, obj->w_unb);
   ROX_ERROR_CHECK_TERMINATE ( error );
      
   error = rox_array2d_double_mulmattransmat(obj->wbuf3x1, obj->oRi_pre, obj->oti_err);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->wbuf3x3, obj->Sk, 1.0 / obj->k_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dd, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dd[0][0] += 1.0; dd[1][1] += 1.0; dd[2][2] += 1.0;
   
   error = rox_array2d_double_mulmatmat(obj->c_ba, obj->wbuf3x3, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ba, obj->c_ba, -obj->k_ba);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_gi = +k(6)*oti_err;
   error = rox_array2d_double_scale(obj->c_gi, obj->oti_err, obj->k_gi);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // State update 
   //bw_hat =  bw_pre +          c_bw*dt ;
   //  ba_hat =  ba_pre +          c_ba*dt ;
   //  oRi_hat = oRi_pre * expmSO3(c_ri*dt);
   //  oti_hat = oti_pre +          c_ti*dt ;
   //  ovi_hat = ovi_pre +          c_vi*dt ;
   //  ogi_hat = ogi_pre +          c_gi*dt ;

   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ba, obj->c_ba, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ti, obj->c_ti, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_vi, obj->c_vi, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_gi, obj->c_gi, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_add(obj->bw_hat, obj->bw_pre, obj->c_bw);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(obj->ba_hat, obj->ba_pre, obj->c_ba);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(obj->oti_hat, obj->oti_pre, obj->c_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(obj->ovi_hat, obj->ovi_pre, obj->c_vi);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(obj->ogi_hat, obj->ogi_pre, obj->c_gi);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matso3_update_right(obj->wbuf3x3, obj->c_ri);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->oRi_hat, obj->oRi_pre, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi_rv_tv(Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double **dd;

   // TODO: to be replaced by a constant 
   Rox_Double dt = 0.01;

   if(!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dPa = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dPa, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dSk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dSk, obj->Sk);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimation error 
   // oRv_pre = oRi_pre * iRv_pre;
   error = rox_array2d_double_mulmatmat(obj->oRv_pre, obj->oRi_pre, obj->iRv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // otv_pre = oRi_pre * itv_pre + oti_pre;
   error = rox_array2d_double_mulmatmat(obj->otv_pre, obj->oRi_pre, obj->itv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_add(obj->otv_pre, obj->otv_pre, obj->oti_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //oRv_error = (oRv_mes * inv(oRv_pre);
   error = rox_array2d_double_mulmatmattrans(obj->oRv_err, obj->oRv_meas, obj->oRv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_substract(obj->otv_err, obj->otv_meas, obj->otv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Projection on so3 Lie algebra
   error = rox_array2d_double_transpose(obj->wbuf3x3, obj->oRv_err);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_substract(obj->Sk, obj->oRv_err, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->Sk, obj->Sk, 0.5);
   ROX_ERROR_CHECK_TERMINATE ( error );
    
   dPa[0][0] = dSk[2][1];
   dPa[1][0] = dSk[0][2];
   dPa[2][0] = dSk[1][0];

   //Unbiased measure
   error = rox_array2d_double_substract(obj->w_unb, obj->w, obj->bw_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Innovation terms
   //c_rv = +k(3)*transpose(oRv_pre)*skew(Pa_oRv_err)*oRi_pre*(w_mes-bw_pre);
   error = rox_array2d_double_mulmatmat(obj->wbuf3x1, obj->oRi_pre, obj->w_unb);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmattransmat(obj->wbuf3x3, obj->oRv_pre, obj->Sk);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->c_rv, obj->wbuf3x3, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_rv, obj->c_rv, obj->k_rv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_ri = +k(1)*transpose(oRi_pre)*Pa_oRv_err - iRv_pre*c_rv;
   error = rox_array2d_double_mulmatmat(obj->wbuf3x1, obj->iRv_pre, obj->c_rv);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmattransmat(obj->c_ri, obj->oRi_pre, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, obj->k_ri);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_substract(obj->c_ri, obj->c_ri, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_bw = -k(2)*transpose(oRi_pre)*Pa_oRv_err;
   error = rox_array2d_double_mulmattransmat(obj->c_bw, obj->oRi_pre, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, -obj->k_bw);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_tv = -k_tv * skew(w_mes-bw_pre)*transpose(oRi_pre)*otv_err;
   error = rox_linalg_so3generator(obj->Sk, obj->w_unb);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmattransmat(obj->wbuf3x1, obj->oRi_pre, obj->otv_err);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->c_tv, obj->Sk, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_tv, obj->c_tv, -obj->k_tv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_ti = +k_ti * otv_err - oRi_pre * c_tv;
   error = rox_array2d_double_mulmatmat(obj->wbuf3x1, obj->oRi_pre, obj->c_tv);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ti, obj->otv_err, obj->k_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_substract(obj->c_ti, obj->c_ti, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_vi = +k_vi * otv_err;
   error = rox_array2d_double_scale(obj->c_vi, obj->otv_err, obj->k_vi);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_ba = -k_ba * (eye(3,3)+(1/k_ti)*skew(w_mes-bw_pre))*transpose(oRi_pre)*otv_err + (k_ba/k_ti)*skew(w_mes-bw_pre)*c_tv;
   error = rox_array2d_double_mulmattransmat(obj->wbuf3x1, obj->oRi_pre, obj->otv_err);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->wbuf3x3, obj->Sk, 1.0 / obj->k_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &dd, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   dd[0][0] += 1.0; dd[1][1] += 1.0; dd[2][2] += 1.0;
   
   error = rox_array2d_double_mulmatmat(obj->c_ba, obj->wbuf3x3, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ba, obj->c_ba, -obj->k_ba);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->wbuf3x1, obj->c_tv, obj->k_ba / obj->k_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(obj->c_ba, obj->c_ba, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // State update 
   //bw_hat =  bw_pre +          c_bw*dt ;
   //  ba_hat =  ba_pre +          c_ba*dt ;
   //  oRi_hat = oRi_pre * expmSO3(c_ri*dt);
   //  oti_hat = oti_pre +          c_ti*dt ;
   //  ovi_hat = ovi_pre +          c_vi*dt ;
   //  iRv_hat = iRv_pre * expmSO3(c_rv*dt);
   //  itv_hat = itv_pre +          c_tv*dt ;

   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_scale(obj->c_ba, obj->c_ba, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_ti, obj->c_ti, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_vi, obj->c_vi, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_scale(obj->c_rv, obj->c_rv, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_tv, obj->c_tv, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_add(obj->bw_hat, obj->bw_pre, obj->c_bw);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(obj->ba_hat, obj->ba_pre, obj->c_ba);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(obj->oti_hat, obj->oti_pre, obj->c_ti);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(obj->ovi_hat, obj->ovi_pre, obj->c_vi);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(obj->itv_hat, obj->itv_pre, obj->c_tv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matso3_update_right(obj->wbuf3x3, obj->c_ri);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->oRi_hat, obj->oRi_pre, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matso3_update_right(obj->wbuf3x3, obj->c_rv);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->iRv_hat, obj->iRv_pre, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi_rv_tv_gi(Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double **dd;

   // TODO: to be replaced by a constant 
   Rox_Double dt = 0.01;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   Rox_Double ** dPa = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dPa, obj->Pa);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double ** dSk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dSk, obj->Sk);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Estimation error 
   // oRv_pre = oRi_pre * iRv_pre;
   error = rox_array2d_double_mulmatmat(obj->oRv_pre, obj->oRi_pre, obj->iRv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // otv_pre = oRi_pre * itv_pre + oti_pre;
   error = rox_array2d_double_mulmatmat(obj->otv_pre, obj->oRi_pre, obj->itv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   error = rox_array2d_double_add(obj->otv_pre, obj->otv_pre, obj->oti_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //oRv_error = (oRv_mes * inv(oRv_pre);
   error = rox_array2d_double_mulmatmattrans(obj->oRv_err, obj->oRv_meas, obj->oRv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_substract(obj->otv_err, obj->otv_meas, obj->otv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Projection on so3 Lie algebra
   error = rox_array2d_double_transpose(obj->wbuf3x3, obj->oRv_err);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_substract(obj->Sk, obj->oRv_err, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->Sk, obj->Sk, 0.5);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   dPa[0][0] = dSk[2][1];
   dPa[1][0] = dSk[0][2];
   dPa[2][0] = dSk[1][0];

   //Unbiased measure
   error = rox_array2d_double_substract(obj->w_unb, obj->w, obj->bw_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //Innovation terms
   //c_rv = +k(3)*transpose(oRv_pre)*skew(Pa_oRv_err)*oRi_pre*(w_mes-bw_pre);
   error = rox_array2d_double_mulmatmat(obj->wbuf3x1, obj->oRi_pre, obj->w_unb);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmattransmat(obj->wbuf3x3, obj->oRv_pre, obj->Sk);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(obj->c_rv, obj->wbuf3x3, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_scale(obj->c_rv, obj->c_rv, obj->k_rv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_ri = +k(1)*transpose(oRi_pre)*Pa_oRv_err - iRv_pre*c_rv;
   error = rox_array2d_double_mulmatmat(obj->wbuf3x1, obj->iRv_pre, obj->c_rv);
      ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_mulmattransmat(obj->c_ri, obj->oRi_pre, obj->Pa);
      ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, obj->k_ri);
      ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_substract(obj->c_ri, obj->c_ri, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_bw = -k(2)*transpose(oRi_pre)*Pa_oRv_err;
   error = rox_array2d_double_mulmattransmat(obj->c_bw, obj->oRi_pre, obj->Pa);
      ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, -obj->k_bw);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //c_tv = -k_tv * skew(w_mes-bw_pre)*transpose(oRi_pre)*otv_err;
   error = rox_linalg_so3generator(obj->Sk, obj->w_unb);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmattransmat(obj->wbuf3x1, obj->oRi_pre, obj->otv_err);
    ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_mulmatmat(obj->c_tv, obj->Sk, obj->wbuf3x1);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_tv, obj->c_tv, -obj->k_tv);
   ROX_ERROR_CHECK_TERMINATE ( error );

    //c_ti = +k_ti * otv_err - oRi_pre * c_tv;
    error = rox_array2d_double_mulmatmat(obj->wbuf3x1, obj->oRi_pre, obj->c_tv);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_ti, obj->otv_err, obj->k_ti);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_substract(obj->c_ti, obj->c_ti, obj->wbuf3x1);
   ROX_ERROR_CHECK_TERMINATE ( error );

    //c_vi = +k_vi * otv_err;
    error = rox_array2d_double_scale(obj->c_vi, obj->otv_err, obj->k_vi);
   ROX_ERROR_CHECK_TERMINATE ( error );

    //c_ba = -k_ba * (eye(3,3)+(1/k_ti)*skew(w_mes-bw_pre))*transpose(oRi_pre)*otv_err + (k_ba/k_ti)*skew(w_mes-bw_pre)*c_tv;
    error = rox_array2d_double_mulmattransmat(obj->wbuf3x1, obj->oRi_pre, obj->otv_err);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_scale(obj->wbuf3x3, obj->Sk, 1.0 / obj->k_ti);
    ROX_ERROR_CHECK_TERMINATE ( error );
   
    error = rox_array2d_double_get_data_pointer_to_pointer( &dd, obj->wbuf3x3);
    ROX_ERROR_CHECK_TERMINATE ( error );

    dd[0][0] += 1.0; dd[1][1] += 1.0; dd[2][2] += 1.0;

    error = rox_array2d_double_mulmatmat(obj->c_ba, obj->wbuf3x3, obj->wbuf3x1);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_scale(obj->c_ba, obj->c_ba, -obj->k_ba);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_scale(obj->wbuf3x1, obj->c_tv, obj->k_ba / obj->k_ti);
    ROX_ERROR_CHECK_TERMINATE ( error );
   
    error = rox_array2d_double_add(obj->c_ba, obj->c_ba, obj->wbuf3x1);
    ROX_ERROR_CHECK_TERMINATE ( error );

    //c_gi = +k_gi * otv_err-(k_gi/k_ti) * oRi_pre * c_tv;
    error = rox_array2d_double_mulmatmat(obj->wbuf3x1, obj->oRi_pre, obj->c_tv);
    ROX_ERROR_CHECK_TERMINATE ( error );
    
    error = rox_array2d_double_scale(obj->wbuf3x1, obj->wbuf3x1, obj->k_gi / obj->k_ti);
    ROX_ERROR_CHECK_TERMINATE ( error );
   
    error = rox_array2d_double_scale(obj->c_gi, obj->otv_err, obj->k_gi);
    ROX_ERROR_CHECK_TERMINATE ( error );
   
    error = rox_array2d_double_substract(obj->c_gi, obj->c_gi, obj->wbuf3x1);
    ROX_ERROR_CHECK_TERMINATE ( error );
   
    // State update 
    //bw_hat =  bw_pre +          c_bw*dt ;
    //  ba_hat =  ba_pre +          c_ba*dt ;
    //  oRi_hat = oRi_pre * expmSO3(c_ri*dt);
    //  oti_hat = oti_pre +          c_ti*dt ;
    //  ovi_hat = ovi_pre +          c_vi*dt ;
    //  iRv_hat = iRv_pre * expmSO3(c_rv*dt);
    //  itv_hat = itv_pre +          c_tv*dt ;
    //  ogi_hat = ogi_pre +          c_gi*dt ;

    error = rox_array2d_double_scale(obj->c_bw, obj->c_bw, dt);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_ba, obj->c_ba, dt);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_ri, obj->c_ri, dt);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_ti, obj->c_ti, dt);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_vi, obj->c_vi, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
    error = rox_array2d_double_scale(obj->c_rv, obj->c_rv, dt);
     ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_scale(obj->c_tv, obj->c_tv, dt);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->c_gi, obj->c_gi, dt);
      ROX_ERROR_CHECK_TERMINATE ( error );
 
    error = rox_array2d_double_add(obj->bw_hat, obj->bw_pre, obj->c_bw);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->ba_hat, obj->ba_pre, obj->c_ba);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->oti_hat, obj->oti_pre, obj->c_ti);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->ovi_hat, obj->ovi_pre, obj->c_vi);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->itv_hat, obj->itv_pre, obj->c_tv);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->ogi_hat, obj->ogi_pre, obj->c_gi);
   ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_fillunit(obj->wbuf3x3);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_matso3_update_right(obj->wbuf3x3, obj->c_ri);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(obj->oRi_hat, obj->oRi_pre, obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

    error = rox_array2d_double_fillunit(obj->wbuf3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );
    error = rox_matso3_update_right(obj->wbuf3x3, obj->c_rv);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(obj->iRv_hat, obj->iRv_pre, obj->wbuf3x3);
    ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ri(Rox_Calibration_Vision_Inertial obj)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    // Make a copy 
    error = rox_array2d_double_copy(obj->oTi_pre_copy, obj->oTi_pre);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_copy(obj->ovi_pre_copy, obj->ovi_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );
    error = rox_array2d_double_copy(obj->oTi_hat_copy, obj->oTi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_hat_copy, obj->ovi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );
  
    error = rox_array2d_double_copy(obj->bw_pre, obj->bw_hat);
      ROX_ERROR_CHECK_TERMINATE ( error );
 
    //oRi_pre = oRi * expmSO3((w_mes-bw)*dt); % integration supposing constant velocity
    error = rox_array2d_double_substract(obj->wbuf3x1, obj->w, obj->bw_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_scale(obj->wbuf3x1, obj->wbuf3x1, obj->dt);
     ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_fillunit(obj->wbuf3x3);
     ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_matso3_update_right(obj->wbuf3x3, obj->wbuf3x1);
    ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(obj->oRi_pre, obj->oRi_hat, obj->wbuf3x3);
     ROX_ERROR_CHECK_TERMINATE ( error );
 
    // Update estimates
    error = rox_array2d_double_copy(obj->bw_hat, obj->bw_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_copy(obj->oTi_hat, obj->oTi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
 
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ri_rv(Rox_Calibration_Vision_Inertial obj)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    // Make a copy 
    error = rox_array2d_double_copy(obj->oTi_pre_copy, obj->oTi_pre);
     ROX_ERROR_CHECK_TERMINATE ( error );
    error = rox_array2d_double_copy(obj->ovi_pre_copy, obj->ovi_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_copy(obj->oTi_hat_copy, obj->oTi_hat);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_copy(obj->ovi_hat_copy, obj->ovi_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  
    //oRi_pre = oRi * expmSO3((w_mes-bw)*dt); % integration supposing constant velocity
    error = rox_array2d_double_copy(obj->bw_pre, obj->bw_hat);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_substract(obj->wbuf3x1, obj->w, obj->bw_hat);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->wbuf3x1, obj->wbuf3x1, obj->dt);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillunit(obj->wbuf3x3);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_matso3_update_right(obj->wbuf3x3, obj->wbuf3x1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(obj->oRi_pre, obj->oRi_hat, obj->wbuf3x3);
      ROX_ERROR_CHECK_TERMINATE ( error );
  
    //iRv_pre = iRv ;                     % integration supposing constant position
    error = rox_array2d_double_copy(obj->iRv_pre, obj->iRv_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  
    // Update estimates
    error = rox_array2d_double_copy(obj->bw_hat, obj->bw_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_copy(obj->oTi_hat, obj->oTi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->iTv_hat, obj->iTv_pre);
         ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi(Rox_Calibration_Vision_Inertial obj)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    // Make a copy 
    error = rox_array2d_double_copy(obj->oTi_pre_copy, obj->oTi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_pre_copy, obj->ovi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->oTi_hat_copy, obj->oTi_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_hat_copy, obj->ovi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Compute acceleration 
    //oai = oRi*(f_mes-ba) + ogoi_mes;
    error = rox_array2d_double_substract(obj->wbuf3x1, obj->f, obj->ba_hat);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(obj->a, obj->oRi_hat, obj->wbuf3x1);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_add(obj->a, obj->a, obj->ogi_meas);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Compute predictions 
    //bw_pre = bw;                            % integration supposing constant position
    error = rox_array2d_double_copy(obj->bw_pre, obj->bw_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //ba_pre = ba;                            % integration supposing constant position
    error = rox_array2d_double_copy(obj->ba_pre, obj->ba_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //oRi_pre = oRi * expmSO3((w_mes-bw)*dt); % integration supposing constant velocity
    error = rox_array2d_double_substract(obj->wbuf3x1, obj->w, obj->bw_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_scale(obj->wbuf3x1, obj->wbuf3x1, obj->dt);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_fillunit(obj->wbuf3x3);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_matso3_update_right(obj->wbuf3x3, obj->wbuf3x1);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_mulmatmat(obj->oRi_pre, obj->oRi_hat, obj->wbuf3x3);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //ovi_pre = ovi + oai*dt;                 % integration supposing constant acceleration
    error = rox_array2d_double_scale(obj->ovi_pre, obj->a, obj->dt);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_add(obj->ovi_pre, obj->ovi_pre, obj->ovi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //oti_pre = oti + ovi*dt + oai*dt^2/2;    % integration supposing constant acceleration
    error = rox_array2d_double_scale(obj->wbuf3x1, obj->a, obj->dt*obj->dt*0.5);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_scale(obj->oti_pre, obj->ovi_hat, obj->dt);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_add(obj->oti_pre, obj->oti_pre, obj->wbuf3x1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->oti_pre, obj->oti_pre, obj->oti_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Update estimates
    error = rox_array2d_double_copy(obj->bw_hat, obj->bw_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ba_hat, obj->ba_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->oTi_hat, obj->oTi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_hat, obj->ovi_pre);
        ROX_ERROR_CHECK_TERMINATE ( error );
 
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi_gi(Rox_Calibration_Vision_Inertial obj)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    // Make a copy 
    error = rox_array2d_double_copy(obj->oTi_pre_copy, obj->oTi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_pre_copy, obj->ovi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->oTi_hat_copy, obj->oTi_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_hat_copy, obj->ovi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Compute acceleration 
    //oai = oRi*(f_mes-ba) + ogoi_mes;
    error = rox_array2d_double_substract(obj->wbuf3x1, obj->f, obj->ba_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_mulmatmat(obj->a, obj->oRi_hat, obj->wbuf3x1);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_add(obj->a, obj->a, obj->ogi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Compute predictions 
    //bw_pre = bw;                            % integration supposing constant position
    error = rox_array2d_double_copy(obj->bw_pre, obj->bw_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //ba_pre = ba;                            % integration supposing constant position
    error = rox_array2d_double_copy(obj->ba_pre, obj->ba_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //oRi_pre = oRi * expmSO3((w_mes-bw)*dt); % integration supposing constant velocity
    error = rox_array2d_double_substract(obj->wbuf3x1, obj->w, obj->bw_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_scale(obj->wbuf3x1, obj->wbuf3x1, obj->dt);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_fillunit(obj->wbuf3x3);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_matso3_update_right(obj->wbuf3x3, obj->wbuf3x1);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_mulmatmat(obj->oRi_pre, obj->oRi_hat, obj->wbuf3x3);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //ovi_pre = ovi + oai*dt;                 % integration supposing constant acceleration
    error = rox_array2d_double_scale(obj->ovi_pre, obj->a, obj->dt);
        ROX_ERROR_CHECK_TERMINATE ( error );
 error = rox_array2d_double_add(obj->ovi_pre, obj->ovi_pre, obj->ovi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //oti_pre = oti + ovi*dt + oai*dt^2/2;    % integration supposing constant acceleration
    error = rox_array2d_double_scale(obj->wbuf3x1, obj->a, obj->dt*obj->dt*0.5);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_scale(obj->oti_pre, obj->ovi_hat, obj->dt);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_add(obj->oti_pre, obj->oti_pre, obj->wbuf3x1);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_add(obj->oti_pre, obj->oti_pre, obj->oti_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // ogi_pre = ogi;                          % integration supposing constant position
    error = rox_array2d_double_copy(obj->ogi_pre, obj->ogi_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  
    // Update estimates
    error = rox_array2d_double_copy(obj->bw_hat, obj->bw_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ba_hat, obj->ba_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->oTi_hat, obj->oTi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_hat, obj->ovi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ogi_hat, obj->ogi_pre);
     ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi_rv_tv(Rox_Calibration_Vision_Inertial obj)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    // Make a copy 
    error = rox_array2d_double_copy(obj->oTi_pre_copy, obj->oTi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_pre_copy, obj->ovi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->oTi_hat_copy, obj->oTi_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_hat_copy, obj->ovi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Compute acceleration 
    //oai = oRi*(f_mes-ba) + ogoi_mes;
    error = rox_array2d_double_substract(obj->wbuf3x1, obj->f, obj->ba_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_mulmatmat(obj->a, obj->oRi_hat, obj->wbuf3x1);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_add(obj->a, obj->a, obj->ogi_meas);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Compute predictions 
    //bw_pre = bw;                            % integration supposing constant position
    error = rox_array2d_double_copy(obj->bw_pre, obj->bw_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //ba_pre = ba;                            % integration supposing constant position
    error = rox_array2d_double_copy(obj->ba_pre, obj->ba_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //oRi_pre = oRi * expmSO3((w_mes-bw)*dt); % integration supposing constant velocity
    error = rox_array2d_double_substract(obj->wbuf3x1, obj->w, obj->bw_hat);
        ROX_ERROR_CHECK_TERMINATE ( error );
 error = rox_array2d_double_scale(obj->wbuf3x1, obj->wbuf3x1, obj->dt);
        ROX_ERROR_CHECK_TERMINATE ( error );
 error = rox_array2d_double_fillunit(obj->wbuf3x3);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_matso3_update_right(obj->wbuf3x3, obj->wbuf3x1);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_mulmatmat(obj->oRi_pre, obj->oRi_hat, obj->wbuf3x3);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //ovi_pre = ovi + oai*dt;                 % integration supposing constant acceleration
    error = rox_array2d_double_scale(obj->ovi_pre, obj->a, obj->dt);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->ovi_pre, obj->ovi_pre, obj->ovi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //oti_pre = oti + ovi*dt + oai*dt^2/2;    % integration supposing constant acceleration
    error = rox_array2d_double_scale(obj->wbuf3x1, obj->a, obj->dt*obj->dt*0.5);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_scale(obj->oti_pre, obj->ovi_hat, obj->dt);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->oti_pre, obj->oti_pre, obj->wbuf3x1);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_add(obj->oti_pre, obj->oti_pre, obj->oti_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //iRv_pre = iRv ;                         % integration supposing constant position
    error = rox_array2d_double_copy(obj->iRv_pre, obj->iRv_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //itv_pre = itv ;
    error = rox_array2d_double_copy(obj->itv_pre, obj->itv_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Update estimates
    error = rox_array2d_double_copy(obj->bw_hat, obj->bw_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ba_hat, obj->ba_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->oTi_hat, obj->oTi_pre);
        ROX_ERROR_CHECK_TERMINATE ( error );
 error = rox_array2d_double_copy(obj->iTv_hat, obj->iTv_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_hat, obj->ovi_pre);
        ROX_ERROR_CHECK_TERMINATE ( error );
 
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi_rv_tv_gi(Rox_Calibration_Vision_Inertial obj)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

    // Make a copy 
    error = rox_array2d_double_copy(obj->oTi_pre_copy, obj->oTi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_pre_copy, obj->ovi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->oTi_hat_copy, obj->oTi_hat);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_hat_copy, obj->ovi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Compute acceleration 
    //oai = oRi*(f_mes-ba) + ogoi_mes;
    error = rox_array2d_double_substract(obj->wbuf3x1, obj->f, obj->ba_hat);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_mulmatmat(obj->a, obj->oRi_hat, obj->wbuf3x1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->a, obj->a, obj->ogi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // Compute predictions 
    //bw_pre = bw;                            % integration supposing constant position
    error = rox_array2d_double_copy(obj->bw_pre, obj->bw_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //ba_pre = ba;                            % integration supposing constant position
    error = rox_array2d_double_copy(obj->ba_pre, obj->ba_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //oRi_pre = oRi * expmSO3((w_mes-bw)*dt); % integration supposing constant velocity
    error = rox_array2d_double_substract(obj->wbuf3x1, obj->w, obj->bw_hat);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_scale(obj->wbuf3x1, obj->wbuf3x1, obj->dt);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_fillunit(obj->wbuf3x3);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_matso3_update_right(obj->wbuf3x3, obj->wbuf3x1);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_mulmatmat(obj->oRi_pre, obj->oRi_hat, obj->wbuf3x3);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //ovi_pre = ovi + oai*dt;                 % integration supposing constant acceleration
    error = rox_array2d_double_scale(obj->ovi_pre, obj->a, obj->dt);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_add(obj->ovi_pre, obj->ovi_pre, obj->ovi_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //oti_pre = oti + ovi*dt + oai*dt^2/2;    % integration supposing constant acceleration
    error = rox_array2d_double_scale(obj->wbuf3x1, obj->a, obj->dt*obj->dt*0.5);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_scale(obj->oti_pre, obj->ovi_hat, obj->dt);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->oti_pre, obj->oti_pre, obj->wbuf3x1);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_add(obj->oti_pre, obj->oti_pre, obj->oti_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //iRv_pre = iRv ;                         % integration supposing constant position
    error = rox_array2d_double_copy(obj->iRv_pre, obj->iRv_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    //itv_pre = itv ;
    error = rox_array2d_double_copy(obj->itv_pre, obj->itv_hat);
     ROX_ERROR_CHECK_TERMINATE ( error );

    // ogi_pre = ogi;                          % integration supposing constant position
    error = rox_array2d_double_copy(obj->ogi_pre, obj->ogi_hat);
         ROX_ERROR_CHECK_TERMINATE ( error );

    // Update estimates
    error = rox_array2d_double_copy(obj->bw_hat, obj->bw_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ba_hat, obj->ba_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->oTi_hat, obj->oTi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->iTv_hat, obj->iTv_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  error = rox_array2d_double_copy(obj->ovi_hat, obj->ovi_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_copy(obj->ogi_hat, obj->ogi_pre);
       ROX_ERROR_CHECK_TERMINATE ( error );
  
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_set_default_values(Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_fillval(obj->ogi_meas, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->ogi_pre, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->ogi_hat, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->w, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->f, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->a, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->w_unb, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->a_unb, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(obj->iTv_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillunit(obj->oTv_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillunit(obj->oTi_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillunit(obj->iTv_hat);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillunit(obj->oTi_hat);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->ovi_hat, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->bw_hat, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->ba_hat, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->bw_pre, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->ba_pre, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(obj->oTv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillunit(obj->oTi_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->ovi_pre, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillunit(obj->iTv_pre);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillunit(obj->oTv_err);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->c_rv, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->c_ri, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->c_bw, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->c_tv, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(obj->c_ti, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->c_vi, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->c_ba, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(obj->pre_measure->A, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->pre_measure->W, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   error = rox_array2d_double_fillval(obj->pre_measure->M, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->pre_measure->timestamp = 0.0;

   error = rox_array2d_double_fillval(obj->cur_measure->A, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->cur_measure->W, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_array2d_double_fillval(obj->cur_measure->M, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   obj->cur_measure->timestamp = 0.0;

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_print_predictions(Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_log("obj->w\n");
   error = rox_array2d_double_print(obj->w);
   rox_log("obj->f\n");
   error = rox_array2d_double_print(obj->f);
   rox_log("obj->a\n");
   error = rox_array2d_double_print(obj->a);

   rox_log("obj->dt %6.16f\n", obj->dt);

   rox_log("obj->bw_pre\n");
   error = rox_array2d_double_print(obj->bw_pre);
   rox_log("obj->ba_pre\n");
   error = rox_array2d_double_print(obj->ba_pre);
   rox_log("obj->ogi_pre\n");
   error = rox_array2d_double_print(obj->ogi_pre);

   rox_log("obj->oTv_pre\n");
   error = rox_array2d_double_print(obj->oTv_pre);
   rox_log("obj->oTi_pre\n");
   error = rox_array2d_double_print(obj->oTi_pre);
   rox_log("obj->ovi_pre\n");
   error = rox_array2d_double_print(obj->ovi_pre);
   rox_log("obj->iTv_pre\n");
   error = rox_array2d_double_print(obj->iTv_pre);
    
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_print(Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_log("gains \n");
   rox_log("%f, %f, %f, %f, %f, %f, %f, %f\n", obj->k_ri, obj->k_bw, obj->k_rv, obj->k_ti, obj->k_vi, obj->k_ba, obj->k_tv, obj->k_gi);
   rox_log("obj->ogi_meas\n");
   error = rox_array2d_double_print(obj->ogi_meas);
   rox_log("obj->w\n");
   error = rox_array2d_double_print(obj->w);
   rox_log("obj->f\n");
   error = rox_array2d_double_print(obj->f);
   rox_log("obj->a\n");
   error = rox_array2d_double_print(obj->a);
   rox_log("obj->w_unb\n");
   error = rox_array2d_double_print(obj->w_unb);
   rox_log("obj->a_unb\n");
   error = rox_array2d_double_print(obj->a_unb);

   rox_log("obj->iTv_meas\n");
   error = rox_array2d_double_print(obj->iTv_meas);
   rox_log("obj->oTv_meas\n");
   error = rox_array2d_double_print(obj->oTv_meas);
   rox_log("obj->iTv_hat\n");
   error = rox_array2d_double_print(obj->iTv_hat);
   rox_log("obj->oTi_hat\n");
   error = rox_array2d_double_print(obj->oTi_hat);

   rox_log("obj->ovi_hat\n");
   error = rox_array2d_double_print(obj->ovi_hat);
   rox_log("obj->bw_hat\n");
   error = rox_array2d_double_print(obj->bw_hat);
   rox_log("obj->ba_hat\n");
   error = rox_array2d_double_print(obj->ba_hat);
   rox_log("obj->ogi_hat\n");
   error = rox_array2d_double_print(obj->ogi_hat);
   rox_log("obj->bw_pre\n");
   error = rox_array2d_double_print(obj->bw_pre);
   rox_log("obj->ba_pre\n");
   error = rox_array2d_double_print(obj->ba_pre);
   rox_log("obj->ogi_pre\n");
   error = rox_array2d_double_print(obj->ogi_pre);
    
   rox_log("obj->oTv_pre\n");
   error = rox_array2d_double_print(obj->oTv_pre);
   rox_log("obj->oTi_pre\n");
   error = rox_array2d_double_print(obj->oTi_pre);
   rox_log("obj->ovi_pre\n");
   error = rox_array2d_double_print(obj->ovi_pre);
   rox_log("obj->iTv_pre\n");
   error = rox_array2d_double_print(obj->iTv_pre);
   rox_log("obj->oTv_err\n");
   error = rox_array2d_double_print(obj->oTv_err);

   rox_log("obj->c_rv\n");
   error = rox_array2d_double_print(obj->c_rv);
   rox_log("obj->c_ri\n");
   error = rox_array2d_double_print(obj->c_ri);
   rox_log("obj->c_bw\n");
   error = rox_array2d_double_print(obj->c_bw);
   rox_log("obj->c_ba\n");
   error = rox_array2d_double_print(obj->c_ba);
   rox_log("obj->c_tv\n");
   error = rox_array2d_double_print(obj->c_tv);
   rox_log("obj->c_ti\n");
   error = rox_array2d_double_print(obj->c_ti);
   rox_log("obj->c_vi\n");
   error = rox_array2d_double_print(obj->c_vi);
   rox_log("obj->c_gi\n");
   error = rox_array2d_double_print(obj->c_gi);

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_set_integration_time(Rox_Calibration_Vision_Inertial obj, Rox_Double dt)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   obj->dt = dt;

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_set_gravity(Rox_Calibration_Vision_Inertial obj, Rox_Array2D_Double g)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if(!obj || !g) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_copy(obj->ogi_meas, g);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(obj->ogi_hat, g);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(obj->ogi_pre, g);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error; 
}

Rox_ErrorCode rox_calibration_vision_inertial_init_poses(Rox_Calibration_Vision_Inertial obj, Rox_Array2D_Double oTv, Rox_Array2D_Double iTv)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj || !oTv || !iTv) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_copy(obj->oTv_meas, oTv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(obj->oTv_pre, oTv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(obj->iTv_meas, iTv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(obj->iTv_pre, iTv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_copy(obj->iTv_hat, iTv);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set oTi_hat and oTi_pre 
   error = rox_array2d_double_svdinverse(obj->wbuf4x4, obj->iTv_meas);
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   error = rox_array2d_double_mulmatmat(obj->oTi_hat, obj->oTv_meas, obj->wbuf4x4);
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   error = rox_array2d_double_copy(obj->oTi_pre, obj->oTi_hat);
   ROX_ERROR_CHECK_TERMINATE ( error );
 
   error = rox_array2d_double_copy(obj->oTi_meas, obj->oTi_hat);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_get_vision_estimate(Rox_Array2D_Double oTv_hat, Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    
   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_mulmatmat(oTv_hat, obj->oTi_hat, obj->iTv_hat);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_get_inertial_estimate(Rox_Array2D_Double oTi_hat, Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
    
   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_copy(oTi_hat, obj->oTi_hat);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_calibration_vision_inertial_get_calibration_estimate(Rox_Array2D_Double iTv_hat, Rox_Calibration_Vision_Inertial obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_array2d_double_copy(iTv_hat, obj->iTv_hat);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}
