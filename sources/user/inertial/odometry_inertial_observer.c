//============================================================================
//
//    OPENROX   : File odometry_inertial_observer.c
//
//    Contents  : Implementation of odometry_inertial_observer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "odometry_inertial_observer.h"
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_odometry_inertial_observer_new ( 
    Rox_Inertial_Predicter * predicter, 
    const Rox_MatSE3 vTi, 
    const Rox_MatSE3 pTm, 
    const Rox_Bool sync_flag
)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;

    error = rox_inertial_observer_new ( predicter, vTi, pTm, sync_flag );
    ROX_ERROR_CHECK_TERMINATE ( error );
    
function_terminate:
    return error;
}

Rox_ErrorCode rox_odometry_inertial_observer_del ( Rox_Inertial_Predicter * predicter )
{
    return rox_inertial_observer_del(predicter);
}

Rox_ErrorCode rox_odometry_inertial_observer_set_frequency ( Rox_Inertial_Predicter predicter, const Rox_Double frequency )
{
    return rox_inertial_observer_set_frequency(predicter, frequency);
}

Rox_ErrorCode rox_odometry_inertial_observer_set_timestamp ( Rox_Inertial_Predicter predicter, const Rox_Double timestamp )
{
    return rox_inertial_observer_set_timestamp(predicter, timestamp);
}

Rox_ErrorCode rox_odometry_inertial_observer_make_corrections ( Rox_Inertial_Predicter predicter, Rox_Inertial inertial, const Rox_MatSE3 vTm_mea, const Rox_Bool update)
{
    return rox_inertial_observer_make_corrections(predicter, inertial, vTm_mea, update);
}

Rox_ErrorCode rox_odometry_inertial_observer_init ( Rox_Inertial_Predicter predicter, Rox_Inertial inertial, const Rox_MatSE3 vTm_mea)
{
    return rox_inertial_observer_init(predicter, inertial, vTm_mea);
}

Rox_ErrorCode rox_odometry_inertial_observer_compute_prediction ( Rox_MatSE3 vTm_pre, Rox_Inertial_Predicter predicter, Rox_Inertial inertial)
{
    return rox_inertial_observer_compute_prediction(vTm_pre, predicter, inertial);
}

