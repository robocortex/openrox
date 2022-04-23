//==============================================================================
//
//    OPENROX   : File dynvec_point2d_projection_from_point3d.c
//
//    Contents  : Implementation of pointsproject module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "dynvec_point2d_projection_from_dynvec_point3d.h"

#include <generated/dynvec_double_struct.h>

#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point3d_float_struct.h>

#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_point3d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/linalg/matsl3.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point3d_struct.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_dynvec_point2d_double_project (
   Rox_DynVec_Point2D_Double output2d, 
   Rox_DynVec_Point3D_Double input, 
   Rox_Matrix calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output2d || !calib) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_double_transform_project (
   Rox_DynVec_Point2D_Double dynvec_point2d, 
   Rox_DynVec_Double depth, 
   const Rox_Matrix calib, 
   const Rox_MatSE3 pose, 
   const Rox_DynVec_Point3D_Double dynvec_point3d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!depth) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!dynvec_point2d)   
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!calib) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!pose)  
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!dynvec_point3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point2d_double_transform_project (dynvec_point2d->data, depth->data, calib, pose, dynvec_point3d->data, dynvec_point3d->used);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_float_project (
   Rox_DynVec_Point2D_Float output2d, 
   Rox_DynVec_Point3D_Float input, 
   Rox_Matrix calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!output2d || !calib) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_float_project_meters (
   Rox_DynVec_Point2D_Float output2d, 
   Rox_DynVec_Point3D_Float input
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!output2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_double_intermodel_projection (   
   Rox_DynVec_Point2D_Double res, 
   const Rox_Matrix homography, 
   const Rox_DynVec_Point3D_Double model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !model || !homography)
   { error =  ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
function_terminate:
   return error;
}

Rox_ErrorCode rox_dynvec_point2d_float_intermodel_projection (
   Rox_DynVec_Point2D_Float res, 
   const Rox_Matrix homography, 
   const Rox_DynVec_Point3D_Double model
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res || !model || !homography)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
function_terminate:
   return error;
}
