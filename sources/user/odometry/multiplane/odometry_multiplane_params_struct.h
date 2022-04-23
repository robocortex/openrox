//==============================================================================
//
//    OPENROX   : File odometry_multiplane_params_struct.h
//
//    Contents  : Structure of odometry_multiplane_params module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ODOMETRY_SINGLE_PLANE_PARAMS_STRUCT__
#define __OPENROX_ODOMETRY_SINGLE_PLANE_PARAMS_STRUCT__

#include <system/memory/datatypes.h>


//! \ingroup Odometry
//! \addtogroup Odometry_Multi_Plane_Params
//! \brief Parameter set of the model 2D based odometry
//! @{

//! A structure to represent the parameters of the 2D model based odometry
struct Rox_Odometry_Multi_Plane_Params_Struct
{
   //! The prediction radius
   Rox_Sint prediction_radius;

   //! The higher pyramid level
   Rox_Sint init_pyr;

   //! The lower pyramid level
   Rox_Sint stop_pyr;

   //! The predefined usecase
   enum Rox_Odometry_Multi_Plane_UseCase usecase;
};

//! @}

#endif // __OPENROX_ODOMETRY_SINGLE_PLANE_PARAMS__
