//==============================================================================
//
//    OPENROX   : File segment3d.c
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

#include "segment3d.h"

#include <math.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/segment/segment3d_struct.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>
#include <system/errors/errors.h>

Rox_ErrorCode rox_segment3d_new ( Rox_Segment3D * segment3d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Segment3D ret = NULL;

   if (!segment3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *segment3d = NULL;

   ret = (Rox_Segment3D) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->points[0].X = 1.0;
   ret->points[0].Y = 2.0;
   ret->points[0].Z = 3.0;

   ret->points[1].X = 4.0;
   ret->points[1].Y = 5.0;
   ret->points[1].Z = 6.0;

   *segment3d = ret;

function_terminate:
   if ( error ) rox_segment3d_del ( &ret );
   return error;
}

Rox_ErrorCode rox_segment3d_del ( Rox_Segment3D * segment3d )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Segment3D todel = NULL;

   if (!segment3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *segment3d;
   *segment3d = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment3d_set(Rox_Segment3D segment3d, Rox_Double X1, Rox_Double Y1, Rox_Double Z1, Rox_Double X2, Rox_Double Y2, Rox_Double Z2)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!segment3d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   segment3d->points[0].X = X1;
   segment3d->points[0].Y = Y1;
   segment3d->points[0].Z = Z1;

   segment3d->points[1].X = X2;
   segment3d->points[1].Y = Y2;
   segment3d->points[1].Z = Z2;

function_terminate:
   return error;
}
