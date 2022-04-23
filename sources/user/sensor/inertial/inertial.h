//==============================================================================
//
//    OPENROX   : File inertial.h
//
//    Contents  : API of inertial module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INERTIAL__
#define __OPENROX_INERTIAL__

#include <core/inertial/sensor/imu.h>

//! \ingroup  Sensor
//! \defgroup Inertial Inertial
//! \brief Inertial structures and methods.

//! \addtogroup Inertial
//! @{

//! Define the pointer of the Rox_Imu_Struct 
typedef struct Rox_Imu_Struct * Rox_Inertial;

//! Constructor for inertial structure
//! \param  [out]  inertial         Pointer to the inertial object
//! \param  [in]   frequency        Sampling frequency of the IMU
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_new(Rox_Inertial * inertial, const Rox_Float frequency);

//! Destructor for inertial structure
//! \param  [in ]  inertial         Rox_Imu instance
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_inertial_del(Rox_Inertial * inertial);

//! Add an inertial measure
//! \param  [out]  inertial         Rox_Imu instance
//! \param  [in ]   A               Accelerometer Measure [A_x A_y A_z]
//! \param  [in ]   W               Gyrometer Measure [W_x W_y W_z]
//! \param  [in ]   M               Magnetometer Measure [M_x M_y M_z]
//! \param  [in ]   timestamp       Measure timestamp in seconds
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_inertial_set_measure(Rox_Inertial inertial, const Rox_Double * A, const Rox_Double * W, const Rox_Double * M, Rox_Double timestamp);

//! @} 

#endif // __OPENROX_INERTIAL__
