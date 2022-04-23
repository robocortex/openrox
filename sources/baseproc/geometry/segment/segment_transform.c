//==============================================================================
//
//    OPENROX   : File segment_transform.c
//
//    Contents  : Implementation of segment_transform module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "segment_transform.h"

#include <baseproc/geometry/segment/segment2d_struct.h>
#include <baseproc/geometry/segment/segment3d_struct.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_segment3d_transform(Rox_Segment3D segment3d_out, Rox_Array2D_Double out_T_inp, Rox_Segment3D segment3d_inp)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!segment3d_out || !segment3d_inp || !out_T_inp) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_point3d_double_transform(segment3d_out->points, out_T_inp, segment3d_inp->points,2);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment2d_transform(Rox_Segment2D segment2d_out, Rox_Array2D_Double out_H_inp, Rox_Segment2D segment2d_inp)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!segment2d_out || !segment2d_inp || !out_H_inp) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_point2d_double_homography(segment2d_out->points, segment2d_inp->points, out_H_inp, 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}