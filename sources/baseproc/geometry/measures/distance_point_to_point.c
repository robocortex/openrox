//==============================================================================
//
//    OPENROX   : File distance_point_to_point.h
//
//    Contents  :
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "distance_point_to_point.h"

#include <generated/dynvec_point2d_double.h>
#include <generated/dynvec_point2d_double_struct.h>

#include <baseproc/geometry/point/point2d_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_distance_point2d_to_point2d ( Rox_Double * distance, const Rox_Point2D_Double point2d_1, const Rox_Point2D_Double point2d_2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!distance || !point2d_1 || !point2d_2) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double du =  point2d_1->u - point2d_2->u;
   Rox_Double dv =  point2d_1->v - point2d_2->v;

   * distance = sqrt(du*du+dv*dv);

function_terminate:
   return error;
}


Rox_ErrorCode rox_dynvec_point2d_double_minimum_distance ( Rox_Double * distance_min, Rox_Uint * index, Rox_Point2D_Double points2D_reprojected, Rox_DynVec_Point2D_Double points2D_measured)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double distance = DBL_MAX;

   *distance_min = DBL_MAX;

   for (Rox_Uint i = 0; i < points2D_measured->used; i++)
   {
      error = rox_distance_point2d_to_point2d(&distance, points2D_reprojected, &points2D_measured->data[i]);
      ROX_ERROR_CHECK_TERMINATE(error)

      if(distance < *distance_min)
      {
         *distance_min = distance;
         *index = i;
      }
   }

function_terminate:
   return error;
}
