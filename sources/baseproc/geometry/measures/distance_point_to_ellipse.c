//==============================================================================
//
//    OPENROX   : File distance_point_to_ellipse.h
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

#include "distance_point_to_ellipse.h"

#include <baseproc/geometry/ellipse/ellipse2d_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_distance_point2d_to_ellipse2d_algebraic (
   Rox_Double * distance, 
   const Rox_Point2D_Double point2d, 
   const Rox_Ellipse2D_Parametric ellipse2d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double signed_distance = 0.0;

   if (!point2d || !ellipse2d || !distance)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_signed_distance_point2d_to_ellipse2d_algebraic(&signed_distance, ellipse2d, point2d);
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   *distance = fabs(signed_distance);

function_terminate:
   return error;
}

Rox_ErrorCode rox_signed_distance_point2d_to_ellipse2d_algebraic (
   Rox_Double * signed_distance, 
   const Rox_Ellipse2D ellipse2d, const Rox_Point2D_Double point2d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!point2d || !ellipse2d || !signed_distance)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Get the ellipse parameters
   Rox_Double xc = ellipse2d->xc;
   Rox_Double yc = ellipse2d->yc;

   Rox_Double nxx = ellipse2d->nxx;
   Rox_Double nyy = ellipse2d->nyy;
   Rox_Double nxy = ellipse2d->nxy;

   Rox_Double xd = point2d->u-xc;
   Rox_Double yd = point2d->v-yc;

   *signed_distance = sqrt(xd*xd + yd*yd)*(1.0-1.0/sqrt(nxx*xd*xd + 2*nxy*xd*yd + nyy*yd*yd));

function_terminate:
   return error;
}


