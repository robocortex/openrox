//==============================================================================
//
//    OPENROX   : File complementary_filter.h
//
//    Contents  : API of complementary_filter module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_COMPLEMENTARY_FILTER__
#define __OPENROX_COMPLEMENTARY_FILTER__

//#include <core/inertial/sensor/imu.h>
#include <baseproc/maths/linalg/matso3.h>

//! Compute one prediction for a given integration time
//! \param  [out]  p_R_i_pred     Predicted rotation
//! \param  [in ]  p_R_i_prev     Previous rotation
//! \param  [in ]  wi             Measured rotation velocity from the gyrometers of the IMU
//! \param  [in ]  dt             Integration time between previous and next rotation
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_complementary_filter_make_predictions ( 
   Rox_MatSO3 p_R_i_pred, 
   Rox_MatSO3 p_R_i_prev, 
   Rox_Array2D_Double wi, 
   Rox_Double dt
);

//! Compute and apply observer corrections using the external measure
//! \param  [out]  p_R_i_next     Rox_Inertial_Observer instance
//! \param  [in ]  p_R_i_pred     Rox_Imu instance
//! \param  [in ]  p_R_i_meas     Measured rotation from the the accelerometer and the magnetometer of the IMU (see function xxx)
//! \param  [in ]  k              Correction factor (the higher the value the higher is the weight on the measured rotation vs the measured velocity) 
//! \param  [in ]  update         Boolean to make corrections
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_complementary_filter_make_corrections (
   Rox_MatSO3 p_R_i_next, 
   Rox_MatSO3 p_R_i_pred, 
   Rox_MatSO3 p_R_i_meas, 
   Rox_Double k, 
   Rox_Bool update
);

ROX_API Rox_ErrorCode rox_matso3_from_accelerometer_magnetometer ( 
   Rox_MatSO3 p_R_i_meas, 
   Rox_Array2D_Double fi, 
   Rox_Array2D_Double mi
);

//! Compute and apply observer corrections using the external measure
//! \param  [out]  p_R_i_meas     Measured rotation matrix in SO3
//! \param  [in ]  fi             Specific acceleration in Newton/metres      
//! \param  [in ]  hi             Heading in radians
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_matso3_from_accelerometer_heading (
   Rox_MatSO3 p_R_i_meas, 
   Rox_Array2D_Double fi, 
   Rox_Double hi
);

#endif // __OPENROX_COMPLEMENTARY_FILTER__ 
