//==============================================================================
//
//    OPENROX   : File inertial.c
//
//    Contents  : Implementation of inertial module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "inertial.h"

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_inertial_new(Rox_Inertial * inertial, const Rox_Float inertial_frequency)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   error = rox_imu_new(inertial, inertial_frequency);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_inertial_del(Rox_Inertial * inertial)
{
   return rox_imu_del(inertial);
}

Rox_ErrorCode rox_inertial_set_measure(Rox_Inertial inertial, const Rox_Double * A, const Rox_Double * W, const Rox_Double * M, Rox_Double timestamp)
{
   return rox_imu_set_measure(inertial, A, W, M, timestamp);
}

