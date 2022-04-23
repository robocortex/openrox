//==============================================================================
//
//    OPENROX   : File calibration_vision_inertial.h
//
//  	Contents  : API of calibration_vision_inertial module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//==============================================================================

#ifndef __OPENROX_CALIBRATION_VISION_INERTIAL__
#define __OPENROX_CALIBRATION_VISION_INERTIAL__

#include <generated/array2d_double.h>
#include <core/inertial/measure/inertial_measure.h>

//! \addtogroup Camera_Calibration
//! @{

//! Enum 
enum Rox_Calibration_Vision_Inertial_Parameters
{
   Rox_Calibration_Vision_Inertial_bw_ri,
   Rox_Calibration_Vision_Inertial_bw_ri_rv,
   Rox_Calibration_Vision_Inertial_bw_ba_ri_ti_vi,
   Rox_Calibration_Vision_Inertial_bw_ba_ri_ti_vi_rv_tv,
   Rox_Calibration_Vision_Inertial_bw_ba_ri_ti_vi_gi,
   Rox_Calibration_Vision_Inertial_bw_ba_ri_ti_vi_rv_tv_gi
};

//! Stucture 
struct Rox_Calibration_Vision_Inertial_Struct
{
	//! The measured gravity vector 
	Rox_Array2D_Double ogi_meas;

	//! The predicted gravity vector 
	Rox_Array2D_Double ogi_pre;

	//! The estimated gravity vector 
	Rox_Array2D_Double ogi_hat;

	//! Inertial measure 
	Rox_Array2D_Double w;

  	//! To be commented  
	Rox_Array2D_Double f;

  	//! To be commented  
	Rox_Array2D_Double a;

  	//! To be commented  
	Rox_Array2D_Double w_unb;

  	//! To be commented  
	Rox_Array2D_Double a_unb;

	//! The inertial - vision pose and associated subviews
	Rox_Array2D_Double iTv_meas;

  	//! To be commented  
	Rox_Array2D_Double iRv_meas;

  	//! To be commented  
	Rox_Array2D_Double itv_meas;
	

	//! The vision pose and associated subviews
	Rox_Array2D_Double oTv_meas;

  	//! To be commented  
	Rox_Array2D_Double oRv_meas;

  	//! To be commented  
	Rox_Array2D_Double otv_meas;

	//! The inertial pose and associated subviews
	Rox_Array2D_Double oTi_meas;

  	//! To be commented  
	Rox_Array2D_Double oRi_meas;

  	//! To be commented  
	Rox_Array2D_Double oti_meas;

	//! The inertial - vision pose and associated subviews 
	Rox_Array2D_Double iTv_hat;

  	//! To be commented  
	Rox_Array2D_Double iRv_hat;

  	//! To be commented  
	Rox_Array2D_Double itv_hat;

	//! The inertial pose and velocity wrt to the local tangent plane and associated subviews 
	Rox_Array2D_Double oTi_hat;

  	//! To be commented  
	Rox_Array2D_Double oRi_hat;

  	//! To be commented  
	Rox_Array2D_Double oti_hat;

  	//! To be commented  
	Rox_Array2D_Double ovi_hat;

	//! The estimated inertial velocity biases 
	Rox_Array2D_Double bw_hat;

	//! The estimated inertial acceleration biases 
	Rox_Array2D_Double ba_hat;

	//! The predicted inertial velocity biases 
	Rox_Array2D_Double bw_pre;

	//! The predicted inertial acceleration biases 
   Rox_Array2D_Double ba_pre;

	//! The predicted camera pose 
	Rox_Array2D_Double oTv_pre;

	//! The predicted camera rotation 
	Rox_Array2D_Double oRv_pre;

	//! The predicted camera translation 
	Rox_Array2D_Double otv_pre;

	//! The predicted imu pose 
	Rox_Array2D_Double oTi_pre;

	//! The predicted imu rotation 
	Rox_Array2D_Double oRi_pre;

	//! The predicted imu translation 
	Rox_Array2D_Double oti_pre;

	//! The predicted imu linear velocity 
	Rox_Array2D_Double ovi_pre;

	//! A copy of the predicted imu pose 
	Rox_Array2D_Double oTi_pre_copy;

	//! A copy of the predicted imu rotation 
	Rox_Array2D_Double ovi_pre_copy;

	//! A copy of the predicted imu translation 
	Rox_Array2D_Double oTi_hat_copy;

	//! A copy of the predicted imu linear velocity 
	Rox_Array2D_Double ovi_hat_copy;

	//! The inertial - vision pose 
   Rox_Array2D_Double iTv_pre;

	//! The inertial - vision rotation 
	Rox_Array2D_Double iRv_pre;

	//! The inertial - vision translation 
	Rox_Array2D_Double itv_pre;

	//! The error on the camera pose 
	Rox_Array2D_Double oTv_err;

	//! The error on the camera rotation 
	Rox_Array2D_Double oRv_err;

	//! The error on the camera translation 
	Rox_Array2D_Double otv_err;

	//! The error on the imu pose 
	Rox_Array2D_Double oTi_err;

	//! The error on the imu rotation 
	Rox_Array2D_Double oRi_err;

	//! The error on the imu translation 
	Rox_Array2D_Double oti_err;

	//! Gains 
	Rox_Double k_tv;

	//! Gains 
	Rox_Double k_ti;

	//! Gains 
	Rox_Double k_rv;

	//! Gains 
	Rox_Double k_ri;

	//! Gains 
	Rox_Double k_ba;

	//! Gains 
	Rox_Double k_bw;

	//! Gains 
	Rox_Double k_vi;

	//! Gains 
	Rox_Double k_gi;

	//! Integration pitch 
	Rox_Double dt;

	//! Innovation terms 
	Rox_Array2D_Double c_rv;

	//! Innovation terms 
	Rox_Array2D_Double c_ri;

	//! Innovation terms 
	Rox_Array2D_Double c_bw;

	//! Innovation terms 
	Rox_Array2D_Double c_tv;

	//! Innovation terms 
	Rox_Array2D_Double c_ti;

	//! Innovation terms 
   Rox_Array2D_Double c_vi;

	//! Innovation terms 
   Rox_Array2D_Double c_ba;

	//! Innovation terms 
	Rox_Array2D_Double c_gi;

	//! Working buffers 
	Rox_Array2D_Double wbuf3x3;

	//! Working buffers 
	Rox_Array2D_Double wbuf4x4;

	//! Working buffers 
	Rox_Array2D_Double wbuf3x1;

	//! Working buffers 
	Rox_Array2D_Double Pa;
	//! Working buffers 
	Rox_Array2D_Double Sk;

	//! Imu current measures 
	Rox_Imu_Measure cur_measure;

	//! Imu previous measures 
	Rox_Imu_Measure pre_measure;

  	//! To be commented  
	Rox_Uint first_measure;
  	//! To be commented  
	Rox_Double cur_video_timestamp;
  	//! To be commented  
	Rox_Double pre_video_timestamp;

	//! Function pointer of the specific make function (depends on the definer usecase) 
	Rox_ErrorCode (*_fptr_make_predictions)(struct Rox_Calibration_Vision_Inertial_Struct *);

	//! Function pointer of the specific update function (depends on the definer usecase) 
	Rox_ErrorCode (*_fptr_nonlin_observer)(struct Rox_Calibration_Vision_Inertial_Struct *);
};

//! object 
typedef struct Rox_Calibration_Vision_Inertial_Struct * Rox_Calibration_Vision_Inertial;

//! To be commented
//! \param  [out]  obj 			    The pointer to the object
//! \param  [in ]  params
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_new (
	Rox_Calibration_Vision_Inertial * obj, 
	enum Rox_Calibration_Vision_Inertial_Parameters params
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo 	To be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_del (
	Rox_Calibration_Vision_Inertial * obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \param  [in ]  t1
//! \param  [in ]  t2
//! \param  [in ]  t3
//! \param  [in ]  t4
//! \param  [in ]  t5
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_init_gains (
	Rox_Calibration_Vision_Inertial obj, 
	Rox_Double t1, 
	Rox_Double t2, 
	Rox_Double t3, 
	Rox_Double t4, 
	Rox_Double t5
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \param  [in ]  oTv
//! \param  [in ]  iTv
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_init_poses (
	Rox_Calibration_Vision_Inertial obj, 
	Rox_Array2D_Double oTv, 
	Rox_Array2D_Double iTv
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_compute_predictions (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \param  [in ]  oTv_meas
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_compute_corrections (
	Rox_Calibration_Vision_Inertial obj, Rox_Array2D_Double oTv_meas
);

//! To be commented
//! \param  [out]  obj the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ri (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ri_rv (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi_rv_tv (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi_gi (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_nonlin_observer_bw_ba_ri_ti_vi_rv_tv_gi (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_set_default_values (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_print (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_print_predictions (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ri (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ri_rv (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi_rv_tv (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi_gi (
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_make_predictions_bw_ba_ri_ti_vi_rv_tv_gi (
	Rox_Calibration_Vision_Inertial obj
	);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \param  [in ]  dt
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_set_integration_time (
	Rox_Calibration_Vision_Inertial obj, 
	Rox_Double dt
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \param  [in ]  f
//! \param  [in ]  w
//! \param  [in ]  timestamp
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_set_inertial_measure (
	Rox_Calibration_Vision_Inertial obj, 
	Rox_Array2D_Double f, 
	Rox_Array2D_Double w, 
	Rox_Double timestamp
	);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \param  [in ]  g
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_set_gravity (
	Rox_Calibration_Vision_Inertial obj, 
	Rox_Array2D_Double g
);

//! To be commented
//! \param  [out]  oTv_pre
//! \param  [in ]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_get_vision_prediction (
	Rox_Array2D_Double oTv_pre, 
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  oTi_pre
//! \param  [in ]  obj the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_get_inertial_prediction (
	Rox_Array2D_Double oTi_pre, 
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  iTv_pre
//! \param  [in ]  obj 				the pointer to the object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_get_calibration_prediction (
	Rox_Array2D_Double iTv_pre, 
	Rox_Calibration_Vision_Inertial obj
);

//! To be commented
//! \param  [out]  obj 				the pointer to the object
//! \param  [in ]  f
//! \param  [in ]  w
//! \param  [in ]  timestamp
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_compute_asynchronous_predictions (
	Rox_Calibration_Vision_Inertial obj, 
	Rox_Array2D_Double f, 
	Rox_Array2D_Double w, 
	Rox_Double timestamp
);

//! To be commented
//! \param  [out]  obj 				 The pointer to the object
//! \param  [in ]  oTv_meas
//! \param  [in ]  timestamp
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_calibration_vision_inertial_compute_asynchronous_corrections (
	Rox_Calibration_Vision_Inertial obj, 
	Rox_Array2D_Double oTv_meas, 
	Rox_Double timestamp
);

//! @} 

#endif // __OPENROX_CALIBRATION_VISION_INERTIAL__
