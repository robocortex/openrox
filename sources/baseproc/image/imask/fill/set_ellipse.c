//============================================================================
//
//    OPENROX   : File set_ellipse.h
//
//    Contents  : API of set_ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//============================================================================

#include "set_ellipse.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/ellipse/ellipse2d_struct.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/geometry/measures/intersection_line_ellipse.h>
#include <baseproc/geometry/measures/distance_point_to_point.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_set_centered_ellipse(Rox_Array2D_Uint mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double a, b, e;
   Rox_Double cp[2];
   Rox_Double f1[2];
   Rox_Double f2[2];

   if (!mask)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   Rox_Sint rows = 0, cols = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, mask);
   ROX_ERROR_CHECK_TERMINATE(error);

   Rox_Uint ** data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer(&data, mask);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Determine longest and smallest axis
   if (rows >= cols)
   {
      a = (Rox_Double)(rows) / 2.0f;
      b = (Rox_Double)(cols) / 2.0f;
   }
   else
   {
      a = (Rox_Double)(cols) / 2.0f;
      b = (Rox_Double)(rows) / 2.0f;
   }

   // Calculate eccentricity
   e = sqrt(a*a - b*b);

   // center point
   cp[0] = a;
   cp[1] = b;

   // focal points
   f1[0] = cp[0] + e;
   f1[1] = cp[1];

   f2[0] = cp[0] - e;
   f2[1] = cp[1];

   for (Rox_Sint r = 0; r < rows; r++)
   {
      for (Rox_Sint c = 0; c < cols; c++)
      {
         Rox_Double d1, d2;
         Rox_Double p[2];
         
         if ( rows >= cols )
         {
            p[0] = (Rox_Double)r;
            p[1] = (Rox_Double)c;
         }
         else
         {
            p[0] = (Rox_Double)c;
            p[1] = (Rox_Double)r;
         }

         d1 = sqrt((f1[0] - p[0])*(f1[0] - p[0]) + (f1[1] - p[1])*(f1[1] - p[1]));
         d2 = sqrt((f2[0] - p[0])*(f2[0] - p[0]) + (f2[1] - p[1])*(f2[1] - p[1]));

         if ((d1 + d2) >= 2 * a)
         {
            data[r][c] = 0;
         }
         else
         {
            data[r][c] = ~0;
         }
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_array2d_uint_new_ellipse(Rox_Array2D_Uint * mask, const Rox_Ellipse2D ellipse2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double xc = ellipse2d->xc;
   Rox_Double yc = ellipse2d->yc;
   Rox_Double nxx = ellipse2d->nxx;
   Rox_Double nyy = ellipse2d->nyy;
   Rox_Double nxy = ellipse2d->nxy;

   if (!mask)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   if (!ellipse2d)
   {
      error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error);
   }

   // Get bounding box
   Rox_Double d = nxx*nyy - nxy*nxy;
   Rox_Double bx = sqrt(nyy / d);
   Rox_Double by = sqrt(nxx / d);

   Rox_Sint cols = 2 * (Rox_Sint) bx, rows = 2 * (Rox_Sint) by;
   error = rox_array2d_uint_new(mask, rows, cols);
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_array2d_uint_fillval(*mask, 0);
   ROX_ERROR_CHECK_TERMINATE(error);

   Rox_Uint ** data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer(&data, *mask);
   ROX_ERROR_CHECK_TERMINATE(error);

   for (Rox_Sint y = 0; y < rows; y++)
   {
      for (Rox_Sint x = 0; x < cols; x++)
      {
         Rox_Point2D_Double_Struct point_intersection;
         Rox_Point2D_Double_Struct point_input;
         Rox_Point2D_Double_Struct point_center;

         point_center.u = bx;
         point_center.v = by;

         // Translate input point into ellipse coordinate system
         point_input.u = (Rox_Double)x - bx + xc;
         point_input.v = (Rox_Double)y - by + yc;

         if (((x - by)*(x - bx) + (y - by)*(y - by)) < 0.25)
         {
            data[y][x] = ~0;
            continue;
         }

         // Get intersection point into ellipse coordinate system
         error = rox_ellipse2d_intersection_line_center_point(&point_intersection, ellipse2d, &point_input);
         ROX_ERROR_CHECK_TERMINATE(error);

         // Translate intersection point into imask coordinate system
         point_intersection.u += bx - xc;
         point_intersection.v += by - yc;

         Rox_Double distance_center_to_intersection = 0.0;
         error = rox_distance_point2d_to_point2d(&distance_center_to_intersection, &point_intersection, &point_center);
         ROX_ERROR_CHECK_TERMINATE(error);

         // Translate back input point into imask coordinate system
         point_input.u += bx - xc;
         point_input.v += by - yc;

         Rox_Double distance_center_to_input = 0.0;
         error = rox_distance_point2d_to_point2d(&distance_center_to_input, &point_input, &point_center);
         ROX_ERROR_CHECK_TERMINATE(error);

         if (distance_center_to_input <= distance_center_to_intersection)
         {
            data[y][x] = ~0;
         }
      }
   }

function_terminate:
   return error;
}
