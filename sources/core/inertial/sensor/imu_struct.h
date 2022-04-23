//==============================================================================
//
//    OPENROX   : File imu_struct.h
//
//    Contents  : Structure of imu module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_IMU_STRUCT__
#define __OPENROX_IMU_STRUCT__

#include <core/inertial/frame/frame.h>
#include <core/inertial/measure/inertial_measure.h>
#include <core/inertial/measure/inertial_measure_buffer.h>

//! \addtogroup IMU
//! @{

//! The Rox_Imu_Struct object 
struct Rox_Imu_Struct
{
   // Measure 
   //! Working object to get the data from the sensor 
   Rox_Imu_Measure ext_mea;

   //! Current IMU measure and timestamp (used by the predicter) 
   Rox_Imu_Measure cur_mea;
   
   //! Previous IMU measure and timestamp (used by the predicter) 
   Rox_Imu_Measure pre_mea;
   
   //! Current IMU unbiased measure (used by the predicter) 
   Rox_Imu_Measure unb_mea;
   
   //! Contains IMU measure and timestamp  
   Rox_Imu_Measure_Buffer buf;

   // Bias 
   //! Accelerometer bias 
   Rox_Array2D_Double ba;
   
   //! Gyrometer bias 	
   Rox_Array2D_Double bw;

   // Gravity vector 
   //! Inertial Gravity	
   Rox_Array2D_Double g;

   // Sample frequency
   //! \brief Inertial Sample frequency 
   Rox_Float frequency;

   // IMU Frame 
   //! Real frame w.r.t to the local tangeant plane 
   Rox_Frame Fi_mea;
   
   //! Predicted frame w.r.t to the local tangeant plane 
   Rox_Frame Fi_pre;
   
   //! Estimated frame w.r.t to the local tangeant plane 
   Rox_Frame Fi_est;
};

//! @} 

#endif // __OPENROX_IMU_STRUCT__ 
