//==============================================================================
//
//    OPENROX   : File imu.h
//
//    Contents  : API of imu module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMU__
#define __OPENROX_IMU__

#include <core/inertial/frame/frame.h>
#include <core/inertial/measure/inertial_measure.h>
#include <core/inertial/measure/inertial_measure_buffer.h>

//! \addtogroup IMU
//! @{

//! Define the pointer of the Rox_Imu_Struct 
typedef struct Rox_Imu_Struct * Rox_Imu;

//! Constructor for inertial structure
//! \param [out] inertial Pointer to the inertial object
//! \param [in] frequency Sampling frequency of the IMU
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_imu_new(Rox_Imu *inertial, const Rox_Float frequency);

//! Destructor for inertial structure
//! \param [in] inertial Rox_Imu instance
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_imu_del(Rox_Imu *inertial);

//! Add an inertial measure
//! \param [out]  inertial    Rox_Imu instance
//! \param [in]   A           Accelerometer Measure [A_x A_y A_z]
//! \param [in]   W           Gyrometer Measure [W_x W_y W_z]
//! \param [in]   M           Magnetometer Measure [M_x M_y M_z]
//! \param [in]   timestamp   Measure timestamp in seconds
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_imu_set_measure(Rox_Imu inertial, const Rox_Double* A, const Rox_Double* W, const Rox_Double* M, Rox_Double timestamp);

//! Dequeue one measure from the internal buffer to update the current Rox_Imu
//! \param [in] inertial Rox_Imu instance
//! \remarks Before dequeue the Rox_Imu from the buffer, the current measure is copied to previous measure
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_imu_update_current_measure(const Rox_Imu inertial);

//! Compute unbiased measure
//! \param [in] inertial Rox_Imu instance
//! \param [in] measure Rox_Imu instance
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_imu_update_unbiased_measure(const Rox_Imu inertial, Rox_Imu_Measure measure);

ROX_API Rox_ErrorCode rox_imu_get_estimated_frame(Rox_Frame Fi_est, const Rox_Imu inertial);

//! @} 

#endif // __OPENROX_IMU__ 
