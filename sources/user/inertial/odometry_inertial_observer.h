//============================================================================
//
//    OPENROX   : File odometry_inertial_observer.h
//
//    Contents  : API of odometry_inertial_observer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_INERTIAL_PREDICTER__
#define __OPENROX_INERTIAL_PREDICTER__

#include <core/inertial/observer/inertial_observer.h>
#include <user/sensor/inertial/inertial.h>

//! \ingroup Odometry
//! \addtogroup Odometry_Inertial_Observer
//! @{

//! Define the pointer of the Rox_Inertial_Observer_Struct
typedef struct Rox_Inertial_Observer_Struct * Rox_Inertial_Predicter;

//! Constructor for inertial predicter structure
//! \param  [out]  predicter     The object to create
//! \param  [in ]  vTi           The IMU -> sensor calibration pose
//! \param  [in ]  pTm           The model pose w.r.t the local tangent plane
//! \param  [in ]  sync_flag     The synchronization flag: if 0 the system is asynchronous, otherwise the system is synchronous
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_inertial_observer_new (
   Rox_Inertial_Predicter * predicter, 
   const Rox_MatSE3 vTi, 
   const Rox_MatSE3 pTm, 
   const Rox_Bool sync_flag
);

//! Destructor for Rox_Inertial_Predicter structure
//! \param  [in ]  predicter     The object to delete
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_inertial_observer_del (
   Rox_Inertial_Predicter * predicter
);

//! Set the sampling frequency of the external sensor
//! \param  [in ]  predicter     The predicter object
//! \param  [in ]  frequency     The sampling frequency of the external sensor
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_inertial_observer_set_frequency ( 
   Rox_Inertial_Predicter predicter, 
   const Rox_Double frequency
);

//! Set the acquisition timestamp of the external sensor
//! \param  [in ]  predicter     The predicter object
//! \param  [in ]  timestamp     The acquisition timestamp of the external sensor in seconds
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_inertial_observer_set_timestamp (
   Rox_Inertial_Predicter predicter, 
   const Rox_Double timestamp
);

//! Compute and apply predicter corrections using the external measure
//! \param  [in ]  predicter     The predicter object
//! \param  [in ]  inertial      The inertial object
//! \param  [in ]  vTm_mea       Measured pose of the external sensor
//! \param  [in ]  update        Boolean to make corrections
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_inertial_observer_make_corrections (
   Rox_Inertial_Predicter predicter, 
   Rox_Inertial inertial, 
   const Rox_MatSE3 vTm_mea, 
   const Rox_Bool update
);

//! Initialize the predicter using the measure frame of the external sensor
//! \param  [in ]  predicter     The predicter object
//! \param  [in ]  inertial      The inertial object
//! \param  [in ]  vTm_mea       Measured pose of the external sensor
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_inertial_observer_init(Rox_Inertial_Predicter predicter, Rox_Inertial inertial, Rox_MatSE3 vTm_mea);

//! Compute the IMU predictions and set the external sensor prediction frame using IMU prediction frame and calibration pose between sensors
//! \param  [out]  vTm_pre       Predicted pose of the external sensor
//! \param  [in ]  predicter     The predicter object
//! \param  [in ]  inertial      The inertial object
//! \return An error code
//! \todo   to be tested
ROX_API Rox_ErrorCode rox_odometry_inertial_observer_compute_prediction(Rox_MatSE3 vTm_pre, Rox_Inertial_Predicter predicter, Rox_Inertial inertial);

//! @}

#endif // __OPENROX_INERTIAL_PREDICTER__ 
