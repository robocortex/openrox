//==============================================================================
//
//    OPENROX   : File inertial_measure.h
//
//    Contents  : API of inertial_measure module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INERTIAL_MEASURE__
#define __OPENROX_INERTIAL_MEASURE__

#include <generated/array2d_double.h>

//! \ingroup Inertial
//! \addtogroup IMU
//! @{

//! Define the pointer of the Rox_Imu_Measure_Struct 
typedef struct Rox_Imu_Measure_Struct* Rox_Imu_Measure;

//! Constructor for inertial measure structure
//! \param  [out] measure           Pointer to the inertial measure object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imu_measure_new(Rox_Imu_Measure * measure);

//! Destructor for inertial measure structure
//! \param  [out measure            Object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imu_measure_del(Rox_Imu_Measure *measure);

//! Copy function
//! \param  [in] measure_out        Rox_Imu_Measure instance
//! \param  [in] measure_inp        Rox_Imu_Measure instance
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_imu_measure_copy(Rox_Imu_Measure measure_out, Rox_Imu_Measure measure_inp);

//! @} 

#endif // __OPENROX_INERTIAL_MEASURE__ 
