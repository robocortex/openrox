//==============================================================================
//
//    OPENROX   : File odometry_singleplane_params.h
//
//    Contents  : API of odometry_singleplane_params module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_SINGLE_PLANE_PARAMS__
#define __OPENROX_ODOMETRY_SINGLE_PLANE_PARAMS__

#include <system/memory/datatypes.h>



//! \ingroup Odometry
//! \addtogroup Odometry_Single_Plane_Params
//! \brief Parameter set of the model 2D based odometry
//! @{

//! The different predefined usecases avaiblable 
enum Rox_Odometry_Single_Plane_UseCase
{
   //! 2D model based odometry using an affine light model and an external identification
   Rox_Odometry_Single_Plane_UseCase_Affine_Light,

   //! 2D model based odometry using a robust light model and an external identification
   Rox_Odometry_Single_Plane_UseCase_Robust_Light
};

//! Define the pointer of the Rox_Odometry_Single_Plane_Params_Struct
typedef struct Rox_Odometry_Single_Plane_Params_Struct *  Rox_Odometry_Single_Plane_Params;

//! Create a new odometry parameters object and fill it with default values
//! \param [out] params is the newly created object
//! \return An error code
//! \todo To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_params_new(Rox_Odometry_Single_Plane_Params * params);

//! Delete a odometry parameters object
//! \param  [in ]  params parameters object
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_params_del(Rox_Odometry_Single_Plane_Params * params);

//! Set the prediction radius
//! \param  [out]  params                 The parameters object
//! \param  [in ]  prediction_radius      The prediction radius
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_params_set_prediction_radius (
   Rox_Odometry_Single_Plane_Params params, 
   const Rox_Sint prediction_radius
);

//! Set the higher pyramid level to start the tracking
//! \param  [out]  params      The parameters object
//! \param  [in ]  init_pyr    The pyramid level
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_params_set_init_pyr(Rox_Odometry_Single_Plane_Params params, const Rox_Sint init_pyr);

//! Set the lower pyramid level to stop the tracking
//! \param  [out]  params      The parameters object
//! \param  [in ]  stop_pyr    The pyramid level
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_params_set_stop_pyr(Rox_Odometry_Single_Plane_Params params, const Rox_Sint stop_pyr);

//! Set the usecase for the odometry object
//! \param  [out]  params      The parameters object
//! \param  [in ]   usecase     The odometry usecase
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_odometry_single_plane_params_set_usecase(Rox_Odometry_Single_Plane_Params params, enum Rox_Odometry_Single_Plane_UseCase usecase);

//! @}

#endif // __OPENROX_ODOMETRY_SINGLE_PLANE_PARAMS__
