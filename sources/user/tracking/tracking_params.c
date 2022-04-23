//==============================================================================
//
//    OPENROX   : File tracking_params.c
//
//    Contents  : Implementation of tracking_params module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tracking_params.h"

#include <system/memory/memory.h>
#include <system/errors/errors.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_tracking_params_new (
   Rox_Tracking_Params * params
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Tracking_Params ret = NULL;

   if ( !params ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *params = NULL;

   ret = (Rox_Tracking_Params)rox_memory_allocate(sizeof(*ret), 1);
   if ( !ret ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->prediction_radius = 16;
   ret->init_pyr = ~0;
   ret->stop_pyr = 0;
   ret->usecase = Rox_Tracking_UseCase_SL3;

   *params = ret;

function_terminate:
   // Delete only if an error occurs
   if(error) rox_tracking_params_del(&ret);
   return error;
}

Rox_ErrorCode rox_tracking_params_del (
   Rox_Tracking_Params * params
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Params todel;

   if ( !params )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *params;
   *params = 0;

   if ( !todel ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_params_set_init_pyr (
   Rox_Tracking_Params params, 
   const Rox_Sint init_pyr
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !params ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   params->init_pyr = init_pyr;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_params_set_stop_pyr (
   Rox_Tracking_Params params, 
   const Rox_Sint stop_pyr
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if( !params ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   params->stop_pyr = stop_pyr;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_params_set_prediction_radius (
   Rox_Tracking_Params params, 
   const Rox_Sint prediction_radius
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !params ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   params->prediction_radius = prediction_radius;

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_params_set_usecase (
   Rox_Tracking_Params params, 
   enum Rox_Tracking_UseCase usecase
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !params ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   params->usecase = usecase;

function_terminate:
   return error;
}
