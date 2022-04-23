//==============================================================================
//
//    OPENROX   : File tracking_params.h
//
//    Contents  : API of tracking_params module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRACKING_PARAMS__
#define __OPENROX_TRACKING_PARAMS__

#include <system/memory/datatypes.h>

//!	\ingroup Tracking
//!	\addtogroup Tracking_Params Tracking_Params
//!	\brief Parameter set of the tracking
//!	@{

//! The different predefined usecases avaiblable 
enum Rox_Tracking_UseCase
{
   //! Perform the tracking with SL3 estimation and using an external identification 
   Rox_Tracking_UseCase_SL3,

   //! Perform the tracking with TU TV S R estimation and using an external identification 
   Rox_Tracking_UseCase_tu_tv_s_r,

   //! Perform the tracking with TU TV SU SV estimation and using an external identification 
   Rox_Tracking_UseCase_tu_tv_su_sv,
};

//! The Rox_Tracking_Params_Struct object 
struct Rox_Tracking_Params_Struct
{
   //! The prediction radius 
   Rox_Sint prediction_radius;

   //! The higher pyramid level 
   Rox_Sint init_pyr;

   //! The lower pyramid level 
   Rox_Sint stop_pyr;

   //! The predefined usecase 
   enum Rox_Tracking_UseCase usecase;
};

//! Define the pointer of the Rox_Tracking_Params_Struct 
typedef struct Rox_Tracking_Params_Struct *  Rox_Tracking_Params;

//! Create a new tracking parameters object with default values
//! \param  [out]  params         The newly created object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_params_new ( Rox_Tracking_Params * params );

//! Delete a tracking parameters object
//! \param  [in ]  params         The parameters object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_params_del ( Rox_Tracking_Params * params );

//! Set the prediction radius
//! \param  [out]  params                  The parameters object
//! \param  [in ]  prediction_radius       The prediction radius
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_params_set_prediction_radius (
   Rox_Tracking_Params params, 
   const Rox_Sint prediction_radius
);

//! Set the higher pyramid level to start the tracking
//! \param  [out]  params         The parameters object
//! \param  [in ]  init_pyr       The pyramid level
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_params_set_init_pyr (
   Rox_Tracking_Params params, 
   const Rox_Sint init_pyr
);

//! Set the lower pyramid level to stop the tracking
//! \param  [out]  params         The parameters object
//! \param  [in ]  stop_pyr       The pyramid level
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_params_set_stop_pyr (
   Rox_Tracking_Params params, 
   const Rox_Sint stop_pyr
);

//! Set the usecase for the tracking object
//! \param  [out]  params         The parameters object
//! \param  [in ]  usecase        The tracking usecase
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_tracking_params_set_usecase (
   Rox_Tracking_Params params, 
   enum Rox_Tracking_UseCase usecase
);

//! @} 

#endif
