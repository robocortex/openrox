//==============================================================================
//
//    OPENROX   : File set_ellipse.h
//
//    Contents  : API of set_ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "set_polygon.h"

#include <baseproc/array/fill/fillval.h>
#include <baseproc/geometry/point/point2d_struct.h>
#include <baseproc/geometry/point/point2d_tools.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_array2d_uint_set_polygon(Rox_Imask mask, const Rox_Point2D_Double pts, Rox_Sint const nbpts)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!mask || !pts)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uint_get_size(&rows, &cols, mask);
   ROX_ERROR_CHECK_TERMINATE(error);

   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer(&dm, mask);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Reset mask
   error = rox_array2d_uint_fillval(mask, 0);
   ROX_ERROR_CHECK_TERMINATE(error);

   //! True if the given point is contained inside the boundary.
   //! See: http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
   //! True if the point is inside the boundary, false otherwise
   for (Rox_Sint v = 0; v < rows; v++)
   {
      for (Rox_Sint u = 0; u < cols; u++)
      {
         Rox_Bool result = 0;
         for (Rox_Sint i = 0, j = nbpts - 1; i < nbpts; j = i++)
         {
            if ((pts[i].v > v) != (pts[j].v > v) && (u < (pts[j].u - pts[i].u) * (v - pts[i].v) / (pts[j].v - pts[i].v) + pts[i].u))
            {
               result = !result;
            }
         }
         if (result) dm[v][u] = ~0;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_bounding_box_get_size ( Rox_Sint * u, Rox_Sint * v, Rox_Sint * rows, Rox_Sint * cols, const Rox_Point2D_Double bounding_box )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !u || !v || !rows || !cols ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !bounding_box ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double min_u = bounding_box[0].u;
   Rox_Double min_v = bounding_box[0].v;

   Rox_Double max_u = bounding_box[1].u;
   Rox_Double max_v = bounding_box[1].v;

   for (Rox_Sint k = 1; k < 4; k++)
   {
      if (bounding_box[k].u < min_u) min_u = bounding_box[k].u;
      if (bounding_box[k].u > max_u) max_u = bounding_box[k].u;
      if (bounding_box[k].v < min_v) min_v = bounding_box[k].v;
      if (bounding_box[k].v > max_v) max_v = bounding_box[k].v;
   }

   *cols = (Rox_Sint)(max_u - min_u);
   *rows = (Rox_Sint)(max_v - min_v);

   *u = (Rox_Sint) min_u;
   *v = (Rox_Sint) min_v;

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uint_new_polygon(Rox_Imask * mask, const Rox_Point2D_Double points_list, Rox_Sint const nb_points)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !mask || !points_list )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   // Get bounding box
   Rox_Point2D_Double_Struct bounding_box[4];

   error = rox_vector_point2d_double_compute_bounding_box(bounding_box, points_list, nb_points);
   ROX_ERROR_CHECK_TERMINATE(error);

   Rox_Sint pu = 0, pv = 0;
   Rox_Sint cols = 0, rows = 0;

   error = rox_bounding_box_get_size(&pu, &pv, &rows, &cols, bounding_box);
   ROX_ERROR_CHECK_TERMINATE(error);

   error = rox_array2d_uint_new(mask, rows, cols);
   ROX_ERROR_CHECK_TERMINATE(error);

   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer(&dm, *mask);
   ROX_ERROR_CHECK_TERMINATE(error);

   // Reset mask
   error = rox_array2d_uint_fillval(*mask, 0);
   ROX_ERROR_CHECK_TERMINATE(error);

   //! True if the given point is contained inside the boundary.
   //! See: http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
   //! True if the point is inside the boundary, false otherwise
   for (Rox_Sint v = 0; v < rows; v++)
   {
      for (Rox_Sint u = 0; u < cols; u++)
      {
         Rox_Bool result = 0;
         for (Rox_Sint i = 0, j = nb_points - 1; i < nb_points; j = i++)
         {
            if ((points_list[i].v > v + pv) != (points_list[j].v > v + pv) && (u + pu < (points_list[j].u - points_list[i].u) * (v + pv - points_list[i].v) / (points_list[j].v - points_list[i].v) + points_list[i].u))
            {
               result = !result;
            }
         }
         if (result) dm[v][u] = ~0;
      }
   }

function_terminate:
   return error;
}
