//==============================================================================
//
//    OPENROX   : File segment_project.c
//
//    Contents  : Implementation of segment_project module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "segment_project.h"

#include <baseproc/geometry/segment/segment2d_struct.h>
#include <baseproc/geometry/segment/segment3d_struct.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_segment2d_project_segment3d (
   Rox_Segment2D segment2d, 
   const Rox_Array2D_Double matct2, 
   const Rox_Segment3D segment3d
)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !segment2d || !segment3d || !matct2 ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_point2d_double_project ( segment2d->points, segment3d->points, matct2, 2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment2d_transform_project_segment3d ( 
   Rox_Segment2D segment2d, 
   Rox_Array2D_Double matct2, 
   Rox_Array2D_Double matse3, 
   Rox_Segment3D segment3d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !segment2d || !matct2 || !matse3 || !segment3d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // Get depths of points even in unused afterwards
   Rox_Double depth[2];

   error = rox_point2d_double_transform_project(segment2d->points, depth, matct2, matse3, segment3d->points, 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}