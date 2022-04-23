//==============================================================================
//
//    OPENROX   : File kalman_ctscav.h
//
//    Contents  : API of kalman_ctscav module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_KALMAN_CTSCAV__
#define __OPENROX_KALMAN_CTSCAV__

#include <generated/array2d_double.h>

//! \ingroup Statistics
//! \defgroup Filtering Filtering

//! \ingroup Filtering
//! \defgroup Kalman Kalman

//! \ingroup Kalman
//! \addtogroup Kalman_SE3_ctscav
//! \brief Pose filtering supposing constant linear and angular velocities
//! @{

//! The Kalman structure 
struct Rox_Kalman_Ctscav_Struct
{
   //! The state of the kalman filter is a 13 x 1 vector (3 translation, 4 rotation, 3 translation velocity, 3 rotation velocity) 
   Rox_Array2D_Double state;
   
   //! The covariance of the kalman filter is a 13 x 13 matrix
   Rox_Array2D_Double covariance;

   //! To be commented  
   Rox_Array2D_Double Measure_Predicted;
 	
   //! To be commented  
   Rox_Array2D_Double State_Innovation;
  
   //! To be commented  
   Rox_Array2D_Double Covariance_Innovation;
  
   //! To be commented  
   Rox_Array2D_Double Jacobian_Prediction;
  
   //! To be commented  
   Rox_Array2D_Double Jacobian_Measure;
  
   //! To be commented  
   Rox_Array2D_Double Covariance_Prediction;
   
   //! To be commented  
   Rox_Array2D_Double Covariance_Measure;
 	
   //! To be commented  
   Rox_Array2D_Double Gain;

   //! To be commented  
   Rox_Array2D_Double Eye_SizeXSize;
  
   //! To be commented  
   Rox_Array2D_Double Buffer_SizeXSize;
 	
   //! To be commented  
   Rox_Array2D_Double Buffer_SizeXSize_2;
  
   //! To be commented  
   Rox_Array2D_Double Buffer_NXSize;
  
   //! To be commented  
   Rox_Array2D_Double Buffer_SizeXN;
  
   //! To be commented  
   Rox_Array2D_Double Buffer_NXN;
  
   //! To be commented  
   Rox_Array2D_Double Buffer_SizeX1;
  
   //! To be commented  
   Rox_Array2D_Double Buffer_3X3;
};

//! Define the pointer of the Rox_Kalman_Ctscav_Struct 
typedef struct Rox_Kalman_Ctscav_Struct * Rox_Kalman_Ctscav;

//! Create kalman object
//! \param [out] obj the pointer to the kalman object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kalman_ctscav_new(Rox_Kalman_Ctscav * obj);

//! Delete kalman object
//! \param [in] obj the pointer to the kalman object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kalman_ctscav_del(Rox_Kalman_Ctscav * obj);

//! Compute prediction given previous state
//! \param [in] obj the pointer to the kalman object
//! \param dt the time since the previous state
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kalman_ctscav_predict(Rox_Kalman_Ctscav obj, Rox_Double dt);

//! Compute innovation of the predicted measurement
//! \param [in] obj the pointer to the kalman object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kalman_ctscav_compute_innovation(Rox_Kalman_Ctscav obj);

//! Update state given a measure
//! \param [in] obj the pointer to the kalman object
//! \param [in] measure the measure vector
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kalman_ctscav_update(Rox_Kalman_Ctscav obj, Rox_Array2D_Double measure);

//! Update state given a measure
//! \param [in] obj the pointer to the kalman object
//! \param [in] index the variance index to change
//! \param [in] variance the variance value
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kalman_ctscav_set_prediction_variances(Rox_Kalman_Ctscav obj, Rox_Uint index, Rox_Double variance);

//! Update state given a measure
//! \param [in] obj the pointer to the kalman object
//! \param [in] index the variance index to change
//! \param [in] variance the variance value
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kalman_ctscav_set_measurement_variances(Rox_Kalman_Ctscav obj, Rox_Uint index, Rox_Double variance);

//! @} 

#endif
