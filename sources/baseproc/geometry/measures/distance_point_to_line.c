//==============================================================================
//
//    OPENROX   : File distance_point_to_line.h
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

#include "distance_point_to_line.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_distance_point2d_to_epipolar_line (
   Rox_Float * distance,
   const Rox_Point2D_Float point_ref,
   const Rox_Point2D_Float point_cur,
   const Rox_Matrix fundamental
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!distance || !fundamental) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** F = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &F, fundamental);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Epipolar constraint point_cur' * fundamental * point_ref
   Rox_Line2D_Homogeneous_Struct line_cur;

   // compute epipolar line l_cur = fundamental * point_ref
   line_cur.a = F[0][0]*point_ref->u + F[0][1]*point_ref->v + F[0][2];
   line_cur.b = F[1][0]*point_ref->u + F[1][1]*point_ref->v + F[1][2];
   line_cur.c = F[2][0]*point_ref->u + F[2][1]*point_ref->v + F[2][2];

   // compute distance point_cur to line line_cur
   error = rox_distance_point2d_to_line2d(distance, point_cur, &line_cur);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_distance_point2d_to_line2d (
   Rox_Float * distance,
   const Rox_Point2D_Float point,
   const Rox_Line2D_Homogeneous line
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double length = 0;
   if (!distance) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   length = sqrt(line->a*line->a + line->b*line->b);
   if (length > 0)
   {
      *distance = (Rox_Float) (fabs(point->u*line->a + point->v*line->b + line->c) / length);
   }
   else
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

