//==============================================================================
//
//    OPENROX   : File inertial_observer.c
//
//    Contents  : Implementation of inertial_observer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "inertial_observer.h"
#include "inertial_observer_struct.h"

#include <float.h>

#include <system/memory/memory.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillunit.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/multiply/mulmattransmat.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/substract/substract.h>
#include <baseproc/array/add/add.h>
#include <baseproc/array/inverse/svdinverse.h>
#include <baseproc/array/multiply/mulmatmattrans.h>
#include <baseproc/array/scale/scale.h>
#include <baseproc/array/crossprod/crossprod.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/maths/linalg/matso3.h>

#include <core/inertial/sensor/imu_struct.h>
#include <core/inertial/frame/frame_struct.h>
#include <core/inertial/measure/inertial_measure_struct.h>
#include <core/inertial/measure/inertial_measure_buffer_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_inertial_observer_new (
   Rox_Inertial_Observer * observer,  
   const Rox_Array2D_Double vTi, 
   const Rox_Array2D_Double pTm, 
   const Rox_Bool sync_flag
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Inertial_Observer ret = NULL;

   if(!observer || !vTi || !pTm) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   *observer = NULL;

   ret = (Rox_Inertial_Observer)rox_memory_allocate(sizeof(*ret), 1);
   if(!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Calibration data
   error = rox_array2d_double_new(&ret->v_T_i, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Errors
   error = rox_array2d_double_new(&ret->p_T_i_err, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // model -> local tangeant plane
   error = rox_array2d_double_new(&ret->p_T_m, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Correction terms
   error = rox_array2d_double_new(&ret->c_t, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->c_r, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->c_bw, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&ret->c_ba, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&ret->c_vt, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set default values
   error = rox_array2d_double_copy(ret->v_T_i, vTi); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(ret->p_T_m, pTm); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillunit(ret->p_T_i_err);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(ret->c_t, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(ret->c_r, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(ret->c_bw, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(ret->c_ba, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(ret->c_vt, 0.0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->k1 = 3.333333333333333*4;
   ret->k2 = 1.0*4;
   ret->k3 = 7.852941176470589*4;
   ret->k4 = 15.147058823529411*4;
   ret->k5 = 4.411764705882353*4;

   ret->frequency = 0.0;
   ret->cur_timestamp = 0.0;
   ret->pre_timestamp = 0.0;

   ret->initialized = 0;
   ret->sync = sync_flag;

   *observer = ret;

function_terminate:
   if(error) rox_inertial_observer_del(&ret);

   return error;
}

Rox_ErrorCode rox_inertial_observer_del(Rox_Inertial_Observer * observer)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Inertial_Observer todel = NULL;

   if (!observer) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *observer;
   *observer = NULL;

   if(!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_del(&todel->v_T_i);
   rox_array2d_double_del(&todel->p_T_i_err);
   rox_array2d_double_del(&todel->p_T_m);
   rox_array2d_double_del(&todel->c_t);
   rox_array2d_double_del(&todel->c_r);
   rox_array2d_double_del(&todel->c_bw);
   rox_array2d_double_del(&todel->c_ba);
   rox_array2d_double_del(&todel->c_vt);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_inertial_observer_set_frequency(Rox_Inertial_Observer observer, Rox_Double frequency)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!observer) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if(frequency <= 0) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   observer->frequency = frequency;
  
function_terminate:   
   return error;
}

Rox_ErrorCode rox_inertial_observer_set_timestamp(Rox_Inertial_Observer observer, Rox_Double timestamp)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
   
    if(!observer) 
    { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error) }

    observer->pre_timestamp = observer->cur_timestamp;
    observer->cur_timestamp = timestamp;

function_terminate:   
   return error;
}

Rox_ErrorCode rox_inertial_observer_compute_prediction(Rox_Array2D_Double vTm_pre, Rox_Inertial_Observer observer, Rox_Imu inertial)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Array2D_Double i_T_p_pre = NULL;
   Rox_Array2D_Double i_T_m = NULL;

   if (!observer || !inertial || !vTm_pre ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(observer->sync == 1)
   {
      error = rox_inertial_observer_compute_prediction_synchrone(observer, inertial, observer->frequency);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      error = rox_inertial_observer_compute_prediction_asynchrone(observer, inertial, observer->pre_timestamp, observer->cur_timestamp);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   // Update Fv_pre using inertial prediction
   if(observer->initialized)
   {
      Rox_Array2D_Double p_T_i_pre = inertial->Fi_pre->pose;
      Rox_Array2D_Double v_T_i = observer->v_T_i;
      Rox_Array2D_Double p_T_m = observer->p_T_m;

      // Compute v_T_m_pre = v_T_i * i_T_p_pre * p_T_m
      error = rox_array2d_double_new(&i_T_p_pre, 4, 4);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_new(&i_T_m, 4, 4);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_svdinverse(i_T_p_pre, p_T_i_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_mulmatmat(i_T_m, i_T_p_pre, p_T_m);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_mulmatmat(vTm_pre, v_T_i, i_T_m);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&i_T_p_pre);
   rox_array2d_double_del(&i_T_m);
   return error;
}

Rox_ErrorCode rox_inertial_observer_compute_prediction_synchrone(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Double frequency)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint integration_steps = 0;
   Rox_Float inertial_frequency = 0.0;
   Rox_Uint integration_step_max = 0;
   Rox_Double dt = 0.0;

   if(!observer || !inertial ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   inertial_frequency = inertial->frequency;
   dt = 1.0 / inertial_frequency;
   integration_step_max = (Rox_Uint)(inertial_frequency / frequency );

   while(!rox_imu_measure_buffer_empty(inertial->buf) && (integration_steps < integration_step_max))
   {
      // Dequeue Measure
      error = rox_imu_update_current_measure(inertial); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Set A, W, Au and Wu
      error = rox_imu_update_unbiased_measure(inertial, inertial->cur_mea); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute predictions
      error = rox_inertial_observer_make_predictions(inertial, dt); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      integration_steps++;
   }

function_terminate:   
   return error;
}

Rox_ErrorCode rox_inertial_observer_compute_prediction_asynchrone(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Double pre_timestamp, Rox_Double cur_timestamp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double dt = 0.0;
   Rox_Double timestamp = 0.0;
   Rox_Imu_Measure imu = 0;

   if(!observer  || !inertial) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the oldest measure in buffer
   imu = inertial->buf->data[inertial->buf->start_position];
   timestamp = imu->timestamp;

   // Check timestamps
   if(timestamp > cur_timestamp)
   {
      return ROX_ERROR_NONE;
   }

   // Dequeue useless measures
   while(!rox_imu_measure_buffer_empty(inertial->buf) && (timestamp < pre_timestamp))
   {
      error = rox_imu_update_current_measure(inertial); 
      ROX_ERROR_CHECK_TERMINATE(error)

      // Update the inertial timestamp
      imu = inertial->buf->data[inertial->buf->start_position];
      timestamp = imu->timestamp;
   }

   // ====================== First integration ========================================
   // Switch inertial measures
   error = rox_imu_update_current_measure(inertial);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Update unbiased measure using the previous inertial measure
   error = rox_imu_update_unbiased_measure(inertial, inertial->pre_mea); 
   ROX_ERROR_CHECK_TERMINATE(error)

   // Set integration time
   dt = inertial->cur_mea->timestamp - pre_timestamp;

   // Make prediction
   error = rox_inertial_observer_make_predictions(inertial, dt); 
   ROX_ERROR_CHECK_TERMINATE(error)

   // =================== Next integrations ========================================== 

   while(!rox_imu_measure_buffer_empty(inertial->buf) && (timestamp < cur_timestamp))
   {
      // Switch inertial measures
      error = rox_imu_update_current_measure(inertial); 
      ROX_ERROR_CHECK_TERMINATE(error)

      // Update unbiased measure using the previous inertial measure
      error = rox_imu_update_unbiased_measure(inertial, inertial->pre_mea); 
      ROX_ERROR_CHECK_TERMINATE(error)

      // Set integration time
      dt = inertial->cur_mea->timestamp - inertial->pre_mea->timestamp; 
      ROX_ERROR_CHECK_TERMINATE(error)

      // Make prediction
      error = rox_inertial_observer_make_predictions(inertial, dt); 
      ROX_ERROR_CHECK_TERMINATE(error)

      // Get timestamp
      imu = inertial->buf->data[inertial->buf->start_position];
      timestamp = imu->timestamp;
   }

   // ================= Last integration =============================================
   // Switch inertial measures
   error = rox_imu_update_current_measure(inertial); 
   ROX_ERROR_CHECK_TERMINATE(error)

   // Update unbiased measure using the previous inertial measure
   error = rox_imu_update_unbiased_measure(inertial, inertial->pre_mea); 
   ROX_ERROR_CHECK_TERMINATE(error)

   // Set integration time
   dt = cur_timestamp - inertial->pre_mea->timestamp;

   // Make prediction
   error = rox_inertial_observer_make_predictions(inertial, dt); 
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:   
   return error;
}

Rox_ErrorCode rox_inertial_observer_init(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Array2D_Double vTm_mea)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double v_T_i = NULL;
   Rox_Array2D_Double p_T_m = NULL;

   Rox_Array2D_Double m_T_v_mea = NULL;
   Rox_Array2D_Double m_T_i = NULL;

   if(observer == 0 || inertial == 0 || vTm_mea == 0) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   v_T_i = observer->v_T_i;
   p_T_m = observer->p_T_m;

   error = rox_array2d_double_new(&m_T_v_mea, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&m_T_i, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_svdinverse(m_T_v_mea, vTm_mea); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute p_T_i_mea = p_T_m * m_T_v_mea * v_T_i with p_T_m = I => p_T_i_mea = m_T_v_mea * v_T_i
   error = rox_array2d_double_mulmatmat(m_T_i, m_T_v_mea, v_T_i); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(inertial->Fi_mea->pose, p_T_m, m_T_i); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   //  Init estimates with real values and set velocity to zero
   error = rox_array2d_double_copy(inertial->Fi_est->pose, inertial->Fi_mea->pose); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_copy(inertial->Fi_pre->pose, inertial->Fi_mea->pose); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // The initial velocity may be different from zero
   error = rox_array2d_double_fillval(inertial->Fi_est->vt, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(inertial->Fi_pre->vt, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_fillval(inertial->ba, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(inertial->bw, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Set correction terms to zeros
   error = rox_array2d_double_fillval(observer->c_t, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(observer->c_r, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(observer->c_bw, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(observer->c_ba, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_fillval(observer->c_vt, 0.0); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   observer->initialized = 1;

function_terminate:
   rox_array2d_double_del(&m_T_v_mea);
   rox_array2d_double_del(&m_T_i);
   return error;
}

Rox_ErrorCode rox_inertial_observer_make_corrections(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Array2D_Double vTm_mea, Rox_Bool update)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if(!observer || !inertial || !vTm_mea ) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_inertial_observer_compute_corrections(observer, inertial, vTm_mea, update);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_inertial_observer_apply_corrections(observer, inertial, update);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_inertial_observer_compute_corrections(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Array2D_Double vTm_mea, Rox_Bool update)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double c_t = 0;
   Rox_Array2D_Double c_r = 0;
   Rox_Array2D_Double c_bw = 0;
   Rox_Array2D_Double c_ba = 0;
   Rox_Array2D_Double c_vt = 0;

   Rox_Array2D_Double v_T_i = 0;
   Rox_Array2D_Double p_T_m = 0;

   Rox_Array2D_Double p_t_i_err = 0;
   Rox_Array2D_Double p_R_i_err = 0;

   Rox_Array2D_Double p_T_i_pre = 0;
   Rox_Array2D_Double p_R_i_pre = 0;
   Rox_Array2D_Double p_t_i_pre = 0;

   Rox_Array2D_Double p_T_i_mea = 0;
   Rox_Array2D_Double p_R_i_mea = 0;
   Rox_Array2D_Double p_t_i_mea = 0;

   Rox_Array2D_Double Wu = 0;

   Rox_Array2D_Double m_T_v_mea = 0;
   Rox_Array2D_Double m_T_i = 0;
   Rox_Array2D_Double cross_product = 0;
   Rox_Array2D_Double R_t_tilde = 0;

   if(!observer || !inertial || !vTm_mea) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   c_t = observer->c_t;
   c_r = observer->c_r;
   c_bw = observer->c_bw;
   c_ba = observer->c_ba;
   c_vt = observer->c_vt;

   v_T_i = observer->v_T_i;
   p_T_m = observer->p_T_m;

   p_T_i_mea = inertial->Fi_mea->pose;
   p_T_i_pre = inertial->Fi_pre->pose;

   Wu = inertial->unb_mea->W;

   error = rox_array2d_double_new_subarray2d(&p_t_i_err, observer->p_T_i_err, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&p_R_i_err, observer->p_T_i_err, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&p_t_i_pre, p_T_i_pre, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&p_R_i_pre, p_T_i_pre, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&p_t_i_mea, p_T_i_mea, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&p_R_i_mea, p_T_i_mea, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new(&m_T_v_mea, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&m_T_i, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&cross_product, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // update the p_T_i_mea : p_T_i_mea = p_T_m * m_T_v_mea * v_T_i
   error = rox_array2d_double_svdinverse(m_T_v_mea, vTm_mea);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(m_T_i, m_T_v_mea, v_T_i);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_mulmatmat(p_T_i_mea, p_T_m, m_T_i);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(update != 1)
   {
      // No update -> tracking failed, the measure is initialized thanks to the prediction
      error = rox_array2d_double_copy(p_T_i_mea, p_T_i_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_fillval(c_r, 0.0);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_fillval(c_bw, 0.0);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_fillval(c_ba, 0.0);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_fillval(c_t, 0.0);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_fillval(c_vt, 0.0);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      // Compute the pose error
      error = rox_array2d_double_mulmatmattrans(p_R_i_err, p_R_i_mea, p_R_i_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_substract(p_t_i_err, p_t_i_mea, p_t_i_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute the rotation corretion c_r
      error = rox_inertial_compute_rot_corr_vec(c_r, p_R_i_pre, p_R_i_err);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute c_r = +k1 * io_R_ic_pred_transpose * vex(Pa(io_R_ic_err))
      error = rox_array2d_double_scale(c_r, c_r, observer->k1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute the rotation velocity bias corretion c_bw
      error = rox_inertial_compute_rot_corr_vec(c_bw, p_R_i_pre, p_R_i_err);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute c_r = -k2 * io_R_ic_pred_transpose * vex(Pa(io_R_ic_tilde))
      error = rox_array2d_double_scale(c_bw, c_bw, -observer->k2);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute the translation acceleration bias corretion c_ba
      // Compute the translation corretion c_t

      error = rox_array2d_double_scale(c_t, p_t_i_err, observer->k3);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute the translation velocity corretion c_vt
      error = rox_array2d_double_scale(c_vt, p_t_i_err, observer->k4);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Compute the translation acceleration bias corretion c_ba
      error = rox_array2d_double_new(&cross_product, 3, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_new(&R_t_tilde, 3, 1);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Why use the measured rotation instead of the estimated rotation
      error = rox_array2d_double_mulmattransmat(R_t_tilde, p_R_i_pre, p_t_i_err);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Update c_ba = - k5 * p_R_i_pre_t * p_t_i_tilde - k5/k3 * [w_unbiased]_x (p_R_i_pre_t * p_t_i_tilde)  

      error = rox_array2d_double_scale(c_ba, R_t_tilde, -observer->k5);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_array2d_double_crossprod(cross_product, Wu, R_t_tilde);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_scale(cross_product, cross_product, -observer->k5 / observer->k3);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_add(c_ba, c_ba, cross_product);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&cross_product);
   rox_array2d_double_del(&R_t_tilde);
   rox_array2d_double_del(&m_T_v_mea);
   rox_array2d_double_del(&m_T_i);
   return error;
}

Rox_ErrorCode rox_inertial_observer_apply_corrections(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Bool update)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double dt_video = 1.0 / 40.0;
   
   Rox_Array2D_Double p_T_i_est = 0;
   Rox_Array2D_Double p_T_i_pre = 0;

   Rox_Array2D_Double p_R_i_est = 0;
   Rox_Array2D_Double p_R_i_pre = 0;

   Rox_Array2D_Double p_t_i_est = 0;
   Rox_Array2D_Double p_t_i_pre = 0;

   Rox_Array2D_Double ba = 0;
   Rox_Array2D_Double bw = 0;

   Rox_Array2D_Double c_t = 0;
   Rox_Array2D_Double c_r = 0;
   Rox_Array2D_Double c_bw = 0;
   Rox_Array2D_Double c_ba = 0;
   Rox_Array2D_Double c_vt = 0;

   Rox_Array2D_Double vt_m_est = 0;
   Rox_Array2D_Double vt_m_pre = 0;

   Rox_Array2D_Double ba_pre = 0;
   Rox_Array2D_Double bw_pre = 0;

   if(observer == 0 || inertial == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get data
   ba = inertial->ba;
   bw = inertial->bw;

   c_t = observer->c_t;
   c_r = observer->c_r;
   c_bw = observer->c_bw;
   c_ba = observer->c_ba;
   c_vt = observer->c_vt;

   p_T_i_est = inertial->Fi_est->pose;
   p_T_i_pre = inertial->Fi_pre->pose;
   vt_m_est = inertial->Fi_est->vt;
   vt_m_pre = inertial->Fi_pre->vt;

   error = rox_array2d_double_new_subarray2d(&p_R_i_est, p_T_i_est, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&p_R_i_pre, p_T_i_pre, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&p_t_i_est, p_T_i_est, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new_subarray2d(&p_t_i_pre, p_T_i_pre, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if(update == 1)
   {
      error = rox_array2d_double_new(&ba_pre, 3, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_new(&bw_pre, 3, 1); 
      ROX_ERROR_CHECK_TERMINATE ( error );

      // ================= m_R_ic_estim = m_R_ic_pred * expm(c_r * dt_video) =============
      error = rox_array2d_double_copy(p_R_i_est, p_R_i_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // cr_dtv = c_r * dtv
      error = rox_array2d_double_scale(c_r, c_r, dt_video);
      ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_matso3_update_right(p_R_i_est, c_r);
      ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_scale(c_r, c_r, 1.0 / dt_video);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // ================= m_t_ic_estim = m_t_ic_pred + c_t * dt_video =================
      error = rox_array2d_double_scale(p_t_i_est, c_t, dt_video);
      ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_array2d_double_add(p_t_i_est, p_t_i_est, p_t_i_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // ================= v_m_estim = v_m_pred + c_vt * dt_video ======================
      error = rox_array2d_double_scale(vt_m_est, c_vt, dt_video);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_add(vt_m_est, vt_m_est, vt_m_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // ================= ba_estim = ba_pred + c_ba * dt_video =========================
      error = rox_array2d_double_scale(ba_pre, c_ba, dt_video);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_add(ba, ba, ba_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // ================= bw_estim = bw_pred + c_bw * dt_video =========================
      error = rox_array2d_double_scale(bw_pre, c_bw, dt_video);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_add(bw, bw, bw_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Initialize predict data with estimates
      error = rox_array2d_double_copy(p_T_i_pre, p_T_i_est);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_copy(vt_m_pre, vt_m_est);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   else
   {
      // Estimate = prediction
      error = rox_array2d_double_copy(p_T_i_est, p_T_i_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );
      
      error = rox_array2d_double_copy(vt_m_est, vt_m_pre);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_del(&ba_pre);
   rox_array2d_double_del(&bw_pre);
   rox_array2d_double_del(&p_R_i_pre);
   rox_array2d_double_del(&p_R_i_est);
   rox_array2d_double_del(&p_t_i_pre);
   rox_array2d_double_del(&p_t_i_est);

   return error;
}

Rox_ErrorCode rox_inertial_observer_make_predictions(Rox_Imu inertial, Rox_Double dt)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double p_T_i_pre = NULL;
   Rox_Array2D_Double p_R_i_pre = NULL;
   Rox_Array2D_Double vt = NULL;
   Rox_Array2D_Double vr = NULL;
   Rox_Array2D_Double Au = NULL;

   // Get gravity
   Rox_Array2D_Double at = NULL;
   Rox_Array2D_Double go = NULL;

   if (!inertial) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   p_T_i_pre = inertial->Fi_pre->pose;
   vt = inertial->Fi_pre->vt; // pointer to the translation velocity
   vr = inertial->unb_mea->W; // pointer to the rotation velocity
   Au = inertial->unb_mea->A;
   go = inertial->g;
   
   error = rox_array2d_double_new_subarray2d(&p_R_i_pre, p_T_i_pre, 0, 0, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new(&at, 3, 1); 
   ROX_ERROR_CHECK_TERMINATE(error)

   // =========================================================================
   // Compute at = go + p_R_i_pre * A_unbiased 
   // Au is the unbiased specific force
   error = rox_array2d_double_mulmatmat(at, p_R_i_pre, Au);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   // at is the acceleartion of the sensor
   error = rox_array2d_double_add(at, at, go);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //========================================================================
   // Predict rotation and translation
   error = rox_update_pose_mixed_velocity(p_T_i_pre, at, vt, vr, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Predict velocity
   // error = rox_vector_integration(vt, vt, at, dt); ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_scale(at, at, dt);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_add(vt, vt, at);
   ROX_ERROR_CHECK_TERMINATE ( error );

   //========================================================================

function_terminate:
   rox_array2d_double_del(&p_R_i_pre);
   rox_array2d_double_del(&at);
   return error;
}

Rox_ErrorCode rox_inertial_compute_rot_corr_vec(Rox_Array2D_Double cr, Rox_Array2D_Double m_R_ic_est, Rox_Array2D_Double m_R_ic_err)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double vex = 0;
   Rox_Double angle, axis_x, axis_y, axis_z;
   Rox_Double ** vex_data;

   if(cr == 0 || m_R_ic_est == 0 || m_R_ic_err == 0) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_new(&vex, 3, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_get_data_pointer_to_pointer( &vex_data, vex);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute Pa(m_R_ic_err) = (m_R_ic_err - transpose(m_R_ic_err))/2 =  sin(theta) * skew(axe)
   // vex(Pa) = vex(skew(axe) * sin(angle) ) = axe * sin(angle);
   
   // TODO: What the hell are we doing ? vector is simply vex(R-R')/2
   // why compute axis and angle from sinus and the recomputing axe*sinus again ?
   error = rox_transformtools_axisangle_from_rotationmatrix(&axis_x, &axis_y, &axis_z, &angle, m_R_ic_err);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute vex
   vex_data[0][0] = axis_x * sin(angle);
   vex_data[1][0] = axis_y * sin(angle);
   vex_data[2][0] = axis_z * sin(angle);
   
   // Compute c_r = m_R_ic_est_transpose * vex(Pa(m_R_ic_tilde))
   error = rox_array2d_double_mulmattransmat(cr, m_R_ic_est, vex);
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_double_del(&vex);
   return error;
}

// This function suppose : d vr_c /dt = 0 and  d (oRc vt_c) / dt = 0
Rox_ErrorCode rox_update_pose_mixed_velocity(Rox_Array2D_Double T_out, Rox_Array2D_Double at, Rox_Array2D_Double vt, Rox_Array2D_Double vr, Rox_Double dt)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Array2D_Double R_out = NULL;
   Rox_Array2D_Double t_out = NULL;

   if(!T_out || !at || !vt || !vr) 
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }
   
   if(fabs(dt) < DBL_EPSILON) return ROX_ERROR_NONE;

   error = rox_array2d_double_new_subarray2d(&R_out, T_out, 0, 0, 3, 3);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_new_subarray2d(&t_out, T_out, 0, 3, 3, 1);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Update Rotation
   // r_inc = vr * dt
   error = rox_array2d_double_scale(vr, vr, dt);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_matso3_update_right(R_out, vr);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_scale(vr, vr, 1.0 / dt);
   ROX_ERROR_CHECK_TERMINATE(error)

   // Update translation
   // t_inc = vt * dt + 0.5 * at * dt^2
   error = rox_array2d_double_scale(vt, vt, dt);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_add(t_out, t_out, vt);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_scale(vt, vt, 1.0 / dt);
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_array2d_double_scale(at, at, 0.5*dt*dt);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_add(t_out, t_out, at);
   ROX_ERROR_CHECK_TERMINATE(error)
   
   error = rox_array2d_double_scale(at, at, 1.0 / (0.5*dt*dt));
   ROX_ERROR_CHECK_TERMINATE(error)

function_terminate:
   rox_array2d_double_del(&R_out);
   rox_array2d_double_del(&t_out);
   return error;
}
