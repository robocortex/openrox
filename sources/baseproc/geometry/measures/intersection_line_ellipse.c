//==============================================================================
//
//    OPENROX   : File intersection_line_ellipse.h
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

#include "intersection_line_ellipse.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/ellipse/ellipse2d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ellipse2d_intersection_line_center_point (
   Rox_Point2D_Double point_intersection, 
   Rox_Ellipse2D ellipse2d, 
   Rox_Point2D_Double point
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!point_intersection || !point || !ellipse2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get ellipse parameters
   Rox_Double xc = ellipse2d->xc;
   Rox_Double yc = ellipse2d->yc;
   Rox_Double nxx = ellipse2d->nxx;
   Rox_Double nyy = ellipse2d->nyy;
   Rox_Double nxy = ellipse2d->nxy;

   Rox_Double x = point->u;
   Rox_Double y = point->v;

   // pc = [xc;yc];

   Rox_Double dx = x-xc;
   Rox_Double dy = y-yc;
   Rox_Double eq = nxx*dx*dx + 2*nxy*dx*dy + nyy*dy*dy;

   // eq must be > 0 since S = [nxx, nxy; nxy, nyy] is definite positive [Not needed to test !!!]
   // Test if query point is the center
   if(sqrt(dx*dx + dy*dy) < 0.1) 
   {
      // The intersection is not defined (2 possible intersections)
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;    
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   Rox_Double k = 1.0/sqrt(eq);
   
   // pe = pc + k * (p-pc);

   point_intersection->u = xc + k * dx;
   point_intersection->v = yc + k * dy;

function_terminate:
   return error;
}

