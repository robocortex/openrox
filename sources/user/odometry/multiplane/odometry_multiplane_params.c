//==============================================================================
//
//    OPENROX   : File odometry_multiplane_params.c
//
//    Contents  : Implementation of odometry_multiplane_params module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "odometry_multiplane_params.h"
#include "odometry_multiplane_params_struct.h"

#include <system/memory/memory.h>
#include <system/errors/errors.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_multi_plane_params_new ( Rox_Odometry_Multi_Plane_Params * params )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Multi_Plane_Params ret = 0;

   if (!params) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   *params = NULL;

   ret = (Rox_Odometry_Multi_Plane_Params) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->prediction_radius = 16;
   ret->init_pyr = ~0;
   ret->stop_pyr = 0;
   ret->usecase = Rox_Odometry_Multi_Plane_UseCase_Affine_Light;

   *params = ret;

function_terminate:
   if (error) rox_odometry_multi_plane_params_del(&ret);
   return error;
}

Rox_ErrorCode rox_odometry_multi_plane_params_del(Rox_Odometry_Multi_Plane_Params *params)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Odometry_Multi_Plane_Params todel = NULL;

   if (!params) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *params;
   *params = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_multi_plane_params_set_init_pyr (
   Rox_Odometry_Multi_Plane_Params params, 
   const Rox_Sint init_pyr
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!params) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   params->init_pyr = init_pyr;

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_multi_plane_params_set_stop_pyr (
   Rox_Odometry_Multi_Plane_Params params, 
   const Rox_Sint stop_pyr
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!params)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   params->stop_pyr = stop_pyr;

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_multi_plane_params_set_prediction_radius (
   Rox_Odometry_Multi_Plane_Params params, 
   const Rox_Sint prediction_radius
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!params) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   params->prediction_radius = prediction_radius;

function_terminate:
   return error;
}

Rox_ErrorCode rox_odometry_multi_plane_params_set_usecase (
   Rox_Odometry_Multi_Plane_Params params, 
   enum Rox_Odometry_Multi_Plane_UseCase usecase
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!params)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   params->usecase = usecase;

function_terminate:
   return error;
}
