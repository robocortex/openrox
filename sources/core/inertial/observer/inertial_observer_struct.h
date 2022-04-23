//==============================================================================
//
//    OPENROX   : File inertial_observer_struct.h
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

#ifndef __OPENROX_INERTIAL_OBSERVER_STRUCT__
#define __OPENROX_INERTIAL_OBSERVER_STRUCT__

#include <core/inertial/sensor/imu.h>

//! \ingroup Odometry
//! \addtogroup Inertial_Observer
//! @{

//! The Rox_Inertial_Observer_Struct object 
struct Rox_Inertial_Observer_Struct
{
   // Gain to estimate bias
   //! Gain k1   
   Rox_Double k1;
   //! Gain k2   
   Rox_Double k2;
   //! Gain k3   
   Rox_Double k3;
   //! Gain k4   
   Rox_Double k4;
   //! Gain k5   
   Rox_Double k5;

   // Correction vectors 
   //! Translation correction        
   Rox_Array2D_Double c_t;
   //! Rotation correction       
   Rox_Array2D_Double c_r;
   //! Gyroscopic bias correction       
   Rox_Array2D_Double c_bw;
   //! Accelerometric bias correction   
   Rox_Array2D_Double c_ba;
   //! Velocity correction         
   Rox_Array2D_Double c_vt;

   // Calibration matrix 
   //! Pose between inertial and visual sensor 
   Rox_Array2D_Double v_T_i;

   // Errors 
   //! Rotation error between the reprojected camera measure in imu frame and the prediction : m_R_ic_err = m_R_ic_mea * inv(m_R_ic_pre) 
   Rox_Array2D_Double p_T_i_err;

   //! Pose between the odometry model and the local tangeant plane (identity if the model is on the floor) 
   Rox_Array2D_Double p_T_m;

   //! Flag to get the observer status 
   Rox_Bool initialized;
   
   //! Flag to use synchronous or asynchronous fusion 
   Rox_Bool sync;

   // Data from sensor used to fuse data with IMU 
   //! Timestamp of the current pose 
   Rox_Double cur_timestamp;
   
   //! Timestamp of the previous pose 
   Rox_Double pre_timestamp;
   
   //! Sampling frequency 
   Rox_Double frequency;
};

//! @} 

#endif // __OPENROX_INERTIAL_OBSERVER_STRUCT__ 
