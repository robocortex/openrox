//==============================================================================
//
//    OPENROX   : File inertial_measure_struct.h
//
//    Contents  : Structure of inertial_measure module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_INERTIAL_MEASURE_STRUCT__
#define __OPENROX_INERTIAL_MEASURE_STRUCT__

#include <generated/array2d_double.h>

//! \ingroup Inertial
//! \addtogroup IMU
//! @{

//! Define the structure of the Rox_Imu_Measure 
struct Rox_Imu_Measure_Struct
{
   //! Accelerometer measure 
   Rox_Array2D_Double A;
   
   //! Gyrometer measure 
   Rox_Array2D_Double W;
   
   //! Magnetometer measure 
   Rox_Array2D_Double M;
   
   //! Thermometer measure 
   Rox_Double   T;
   
   //! Measure timestamp 
   Rox_Double timestamp;
};

//! @} 

#endif // __OPENROX_INERTIAL_MEASURE_STRUCT__ 
