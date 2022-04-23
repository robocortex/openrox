//==============================================================================
//
//    OPENROX   : File segment3d_print.c
//
//    Contents  : Implementation Structure of segment3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "segment3d_print.h"

#include <math.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/segment/segment3d_struct.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>
#include <system/errors/errors.h>

Rox_ErrorCode rox_segment3d_print ( Rox_Segment3D segment3d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!segment3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("segment3d point1 = (%f, %f, %f) \n", segment3d->points[0].X, segment3d->points[0].Y, segment3d->points[0].Z);
   rox_log("segment3d point2 = (%f, %f, %f) \n", segment3d->points[1].X, segment3d->points[1].Y, segment3d->points[1].Z);

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment3d_float_print(Rox_Segment3D_Float segment3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!segment3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("segment3d point1 = (%f, %f, %f) \n", segment3d->points[0].X, segment3d->points[0].Y, segment3d->points[0].Z);
   rox_log("segment3d point2 = (%f, %f, %f) \n", segment3d->points[1].X, segment3d->points[1].Y, segment3d->points[1].Z);

function_terminate:
   return error;
}
