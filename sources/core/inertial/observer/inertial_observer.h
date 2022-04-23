//==============================================================================
//
//    OPENROX   : File inertial_observer.h
//
//    Contents  : API of inertial_observer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INERTIAL_OBSERVER__
#define __OPENROX_INERTIAL_OBSERVER__

#include <core/inertial/sensor/imu.h>

//! \ingroup Odometry
//! \addtogroup Inertial_Observer
//!  @{

//! Define the pointer of the Rox_Inertial_Observer_Struct 
typedef struct Rox_Inertial_Observer_Struct * Rox_Inertial_Observer;

//! Constructor for inertial observer structure
//! \param  [out]  observer       The observer object
//! \param  [in ]  vTi            The IMU -> sensor calibration pose
//! \param  [in ]  pTm            The model pose w.r.t the local tangent plane
//! \param  [in ]  sync_flag      The synchronization flag: if 0 the system is asynchronous, otherwise the system is synchronous
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_new (
   Rox_Inertial_Observer * observer, 
   const Rox_MatSE3 vTi, 
   const Rox_MatSE3 pTm, 
   const Rox_Bool sync_flag
);

//! Destructor for Rox_Inertial_Observer structure
//! \param  [in ]  observer       The observer object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_inertial_observer_del(Rox_Inertial_Observer * observer);

//! Set the sampling frequency of the external sensor
//! \param  [in ]  observer       Rox_Inertial_Observer instance
//! \param  [in ]  frequency      The sampling frequency of the external sensor
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_set_frequency(Rox_Inertial_Observer observer, Rox_Double frequency);

//! Set the acquisition timestamp of the external sensor
//! \param  [in ]  observer       Rox_Inertial_Observer instance
//! \param  [in ]  timestamp      The acquisition timestamp of the external sensor
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_set_timestamp(Rox_Inertial_Observer observer, Rox_Double timestamp);

//! Compute one prediction for a given integration time
//! \param  [in ]  inertial       Rox_Imu instance
//! \param  [in ]  dt             Integration time expressed in seconds
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_make_predictions(Rox_Imu inertial, Rox_Double dt);

//! Compute and apply observer corrections using the external measure
//! \param  [in ]  observer       Rox_Inertial_Observer instance
//! \param  [in ]  inertial       Rox_Imu instance
//! \param  [in ]  vTm_mea        Measured pose of the external sensor
//! \param  [in ]  update         Boolean to make corrections
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_make_corrections(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Array2D_Double vTm_mea, Rox_Bool update);

//! Update the prediction using innovation terms
//! \param  [in ]  observer       Rox_Inertial_Observer instance 
//! \param  [in ]  inertial       Rox_Imu instance
//! \param  [in ]  update         Boolean to update prediction: The prediction is updated using innovation terms only if the flag is ROX_TRUE
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_apply_corrections(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Bool update);

//! Initialize the observer using the measure frame of the external sensor
//! \param  [in ]  observer       Rox_Inertial_Observer instance
//! \param  [in ]  inertial       Rox_Imu instance
//! \param  [in ]  vTm_mea        Measure pose of the external sensor
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_inertial_observer_init(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Array2D_Double vTm_mea);

//! Compute the different innovation terms using the measure from the external sensor
//! \param  [in ]  observer       Rox_Inertial_Observer instance
//! \param  [in ]  inertial       Rox_Imu instance
//! \param  [in ]  vTm_mea        Measured pose of the external sensor
//! \param  [in ]  update         Boolean to compute correction: if the odometry is lost, it is not necessary to compute the innovation terms
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_compute_corrections(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Array2D_Double vTm_mea, Rox_Bool update);

//! Compute the IMU predictions and set the external sensor prediction frame using IMU prediction frame and calibration pose between sensors
//! \param  [out]  vTm_pre        Predicted pose of the external sensor
//! \param  [in ]  observer       Rox_Inertial_Observer instance
//! \param  [in ]  inertial       Rox_Imu instance
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_compute_prediction(Rox_Array2D_Double vTm_pre, Rox_Inertial_Observer observer, Rox_Imu inertial);

//! Compute all predictions for a synchronous observer using acquisition timestamps to determine the integration period
//! \param  [in ]  observer       Rox_Inertial_Observer instance
//! \param  [in ]  inertial       Rox_Imu instance
//! \param  [in ]  frequency      External sensor sampling frequency
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_compute_prediction_synchrone(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Double frequency);

//! Compute all predictions for an asynchronous observer using acquisition timestamps to determine the integration period
//! \param  [in ]  observer       Rox_Inertial_Observer instance
//! \param  [in ]  inertial       Rox_Imu instance
//! \param  [in ]  pre_timestamp  Acquisition timestamp
//! \param  [in ]  cur_timestamp  Acquisition timestamp
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_observer_compute_prediction_asynchrone(Rox_Inertial_Observer observer, Rox_Imu inertial, Rox_Double pre_timestamp, Rox_Double cur_timestamp);

//! Compute Pa(Rest*Rerr)
//! \param  [out]  cr             Result of Pa(Rest*Rerr)
//! \param  [in ]  m_R_ic_est     Rotation Matrix
//! \param  [in ]  m_R_ic_err     Rotation Matrix
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_compute_rot_corr_vec(Rox_Array2D_Double cr, Rox_Array2D_Double m_R_ic_est, Rox_Array2D_Double m_R_ic_err);

//! Update both rotation and translation using respectively angular and translational velocities
//! \param  [out]  T_out          Updated pose
//! \param  [in ]  at             Translation acceleration
//! \param  [in ]  vt             Translation velocity
//! \param  [in ]  vr             Angular velocity
//! \param  [in ]  dt             Integration time expressed in seconds
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_update_pose_mixed_velocity(Rox_MatSE3 T_out, Rox_Array2D_Double at, Rox_Array2D_Double vt, Rox_Array2D_Double vr, Rox_Double dt);

//! @} 

#endif // __OPENROX_INERTIAL_OBSERVER__ 
