//==============================================================================
//
//    OPENROX   : File p5points.c
//
//    Contents  : Implementation of p5points module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "p5points.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <baseproc/array/fill/fillunit.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/point/point2d_struct.h>


#include <inout/system/errors_print.h>

Rox_ErrorCode rox_pose_from_5_points (
   Rox_Point2D_Double ref2D, 
   Rox_Point2D_Double cur2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
 
   Rox_Double vx, vy, vz, normpoint, normaxis, angle;
   
   Rox_Array2D_Double R_zerotoref0, R_zerotocur0;
   
   if (!ref2D || !cur2D) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   R_zerotoref0 = 0;
   R_zerotocur0 = 0;
   
   error = rox_array2d_double_new(&R_zerotoref0, 3, 3);
   error = rox_array2d_double_new(&R_zerotocur0, 3, 3);
   
   rox_array2d_double_fillunit(R_zerotoref0);
   rox_array2d_double_fillunit(R_zerotocur0);
   
   // Compute reference rotation normalizer
   normpoint = sqrt(ref2D[0].u * ref2D[0].u + ref2D[0].v * ref2D[0].v + 1);
   normaxis = sqrt(ref2D[0].u * ref2D[0].u + ref2D[0].v * ref2D[0].v);
   if (fabs(normaxis) > DBL_EPSILON)
   {
      vx = -ref2D[0].v / normaxis;
      vy = ref2D[0].u / normaxis;
      vz = 0;
      angle = acos(1/normpoint);
         
      // Rotation
      error = rox_matso3_set_axis_angle(R_zerotoref0, vx, vy, vz, angle); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   
   // Compute current rotation normalizer
   normpoint = sqrt(cur2D[0].u * cur2D[0].u + cur2D[0].v * cur2D[0].v + 1);
   normaxis = sqrt(cur2D[0].u * cur2D[0].u + cur2D[0].v * cur2D[0].v);
   if (fabs(normaxis) > DBL_EPSILON)
   {
      vx = -cur2D[0].v / normaxis;
      vy = cur2D[0].u / normaxis;
      vz = 0;
      angle = acos(1/normpoint);
      
      // Rotation
      error = rox_matso3_set_axis_angle(R_zerotocur0, vx, vy, vz, angle); 
      ROX_ERROR_CHECK_TERMINATE ( error );
   }   
   
function_terminate:
   
   rox_array2d_double_del(&R_zerotoref0);
   rox_array2d_double_del(&R_zerotocur0);
   
   return error;
}