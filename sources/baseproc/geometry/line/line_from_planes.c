//==============================================================================
//
//    OPENROX   : File line_from_planes.c
//
//    Contents  : Implementation of line_from_planes module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "line_from_planes.h"

#include <float.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/line/line3d_struct.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_line3d_planes_from_2_planes (
   Rox_Line3D_Planes res, 
   Rox_Plane3D_Double pl1, 
   Rox_Plane3D_Double pl2
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Update planes such that there is a unique representation for a given line

   // Make sure D2 > D1
   if (fabs(pl2->d) < fabs(pl1->d))
   {
      Rox_Plane3D_Double_Struct swap = *pl1;
      *pl1 = *pl2;
      *pl2 = swap;
   }

   // Compute the direction of the straight line as the cross product of both planes normal N1xN2
   Rox_Double a2 = pl1->b * pl2->c - pl1->c * pl2->b;
   Rox_Double b2 = pl1->c * pl2->a - pl1->a * pl2->c;
   Rox_Double c2 = pl1->a * pl2->b - pl1->b * pl2->a;
 
   // Make sure the plane 3 go through the origin (d3 = 0)
   // N3 = N2 * d1 - N1 * d2
    
   Rox_Double a1 = pl2->a * pl1->d - pl1->a * pl2->d;
   Rox_Double b1 = pl2->b * pl1->d - pl1->b * pl2->d;
   Rox_Double c1 = pl2->c * pl1->d - pl1->c * pl2->d;

   // Normalize plane one normal vector
   Rox_Double norm1 = sqrt(a1*a1 + b1*b1 + c1*c1);
   
   if (norm1 < DBL_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   pl1->a = a1 / norm1;
   pl1->b = b1 / norm1;
   pl1->c = c1 / norm1;
   pl1->d = 0;

   // N4 = (N1xN2)x(N3) : N4 orthogonal to N3 and line
   a1 = b2*pl1->c - c2*pl1->b;
   b1 = c2*pl1->a - a2*pl1->c;
   c1 = a2*pl1->b - b2*pl1->a;

   // normalize n4
   norm1 = sqrt(a1*a1 + b1*b1 + c1*c1);
   if (norm1 < DBL_EPSILON) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   a1 = a1 / norm1;
   b1 = b1 / norm1;
   c1 = c1 / norm1;

   // D4=D2/(N2^t.N4)
   pl2->d = pl2->d / (pl2->a * a1 + pl2->b * b1 + pl2->c * c1);
   pl2->a = a1;
   pl2->b = b1;
   pl2->c = c1;

   // Make sure pl2->d < 0
   if (pl2->d > 0.0)
   {
      pl2->a = -pl2->a;
      pl2->b = -pl2->b;
      pl2->c = -pl2->c;
      pl2->d = -pl2->d;
   }

   res->planes[0] = *pl1;
   res->planes[1] = *pl2;

function_terminate:
   return error;
}
