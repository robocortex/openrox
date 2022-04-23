//==============================================================================
//
//    OPENROX   : File cylinder_project.c
//
//    Contents  : Implementation of cylinder_project module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "cylinder_project.h"

#include <math.h>
#include <float.h>

#include <baseproc/geometry/segment/segment3d_struct.h>
#include <baseproc/geometry/segment/segment_project.h>

#include <baseproc/geometry/cylinder/cylinder2d_struct.h>
#include <baseproc/geometry/cylinder/cylinder3d_struct.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_cylinder2d_project_cylinder3d(Rox_Cylinder2D cylinder2d, Rox_Array2D_Double matct2, Rox_Cylinder3D cylinder3d)
{   
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder2d || !cylinder3d || !matct2 ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
      
   // Get the pose if the cylinder relative to the camera
   Rox_Segment3D_Struct segment3d_1;
   Rox_Segment3D_Struct segment3d_2;

   Rox_MatSE3 cam_T_obj = NULL;
   
   // The pose cam_T_obj is initialized to I
   error = rox_matse3_new(&cam_T_obj);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Get the tangent segments in camera frame
   error = rox_cylinder3d_get_tangent_segments(&segment3d_1, &segment3d_2, cam_T_obj, cylinder3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // 
   error = rox_segment2d_project_segment3d(cylinder2d->s1, matct2, &segment3d_1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_segment2d_project_segment3d(cylinder2d->s2, matct2, &segment3d_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matse3_del(&cam_T_obj);
   return error;
}

Rox_ErrorCode rox_cylinder2d_transform_project_cylinder3d(Rox_Cylinder2D cylinder2d, Rox_Array2D_Double matct2, Rox_Array2D_Double matse3, Rox_Cylinder3D cylinder3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder2d || !matct2 || !matse3 || !cylinder3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the pose iof the cylinder relative to the camera
   Rox_MatSE3 cam_T_obj = matse3;
   
   Rox_Segment3D_Struct segment3d_1;
   Rox_Segment3D_Struct segment3d_2;

   // Get tangent segments in camera frame 
   error = rox_cylinder3d_get_tangent_segments(&segment3d_1, &segment3d_2, cam_T_obj, cylinder3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Transform segments from cylinder frame to camera frame and project into the image 
   error = rox_segment2d_project_segment3d(cylinder2d->s1, matct2, &segment3d_1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_segment2d_project_segment3d(cylinder2d->s2, matct2, &segment3d_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}