//==============================================================================
//
//    OPENROX   : File edge_ellipse.c
//
//    Contents  : Implementation of edge ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edge_ellipse.h"

#include <generated/dynvec_edge_ellipse_site_struct.h>
#include <generated/dynvec_point2d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/ellipse/ellipse3d.h>
#include <baseproc/geometry/ellipse/ellipse_project.h>
#include <baseproc/geometry/ellipse/ellipse_transform.h>
#include <baseproc/geometry/measures/distance_point_to_ellipse.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>
#include <inout/geometry/point/point2d_print.h>
#include <inout/geometry/point/dynvec_point2d_save.h>

Rox_ErrorCode rox_edge_ellipse_new(Rox_Edge_Ellipse * edge_ellipse, const Rox_Sint sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Ellipse ret = NULL;


   if (!edge_ellipse) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *edge_ellipse = NULL;

   ret = (Rox_Edge_Ellipse) rox_memory_allocate(sizeof(struct Rox_Edge_Ellipse_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->ellipse3d_o = NULL;
   error = rox_ellipse3d_new(&ret->ellipse3d_o);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->ellipse3d_c = NULL;
   error = rox_ellipse3d_new(&ret->ellipse3d_c);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->sites = NULL;
   error = rox_dynvec_edge_ellipse_site_new(&ret->sites, 1000);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->expected_density = 1;

   ret->sampling_step = sampling_step;

   *edge_ellipse = ret;

function_terminate:
   if (error) rox_edge_ellipse_del(&ret);
   return error;
}

Rox_ErrorCode rox_edge_ellipse_del(Rox_Edge_Ellipse * edge_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Ellipse todel = NULL;


   if (!edge_ellipse) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *edge_ellipse;
   *edge_ellipse = NULL;

   rox_ellipse3d_del(&todel->ellipse3d_o);
   rox_ellipse3d_del(&todel->ellipse3d_c);
   rox_dynvec_edge_ellipse_site_del(&todel->sites);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_ellipse_set_ellipse3d(Rox_Edge_Ellipse edge_ellipse, Rox_Ellipse3D ellipse3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!edge_ellipse) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ellipse3d_copy(edge_ellipse->ellipse3d_o, ellipse3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_ellipse_transform_project(Rox_Edge_Ellipse edge_ellipse, Rox_Array2D_Double pose, Rox_Array2D_Double calibration)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!edge_ellipse || !pose || !calibration) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calibration, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ellipse3d_transform(edge_ellipse->ellipse3d_c, pose, edge_ellipse->ellipse3d_o);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ellipse2d_project_ellipse3d(&(edge_ellipse->ellipse2d_pixels), calibration, edge_ellipse->ellipse3d_c);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Transform ellipse from pixels coordinates to meters (using inv(calibration))
   error = rox_ellipse2d_transform_pixels_to_meters(&edge_ellipse->ellipse2d_meters, calibration, &edge_ellipse->ellipse2d_pixels);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}

Rox_ErrorCode rox_edge_ellipse_sample(Rox_Edge_Ellipse edge_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!edge_ellipse) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_edge_ellipse_site_reset(edge_ellipse->sites);

   Rox_Ellipse2D ellipse2d = &(edge_ellipse->ellipse2d_pixels);

   Rox_DynVec_Point2D_Double dynvec_point2d = NULL;

   error = rox_dynvec_point2d_double_new(&dynvec_point2d, 1000);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint sampling_step = edge_ellipse->sampling_step;
   error = rox_ellipse2d_sample_perimeter(dynvec_point2d, ellipse2d, sampling_step);
   //Most probably the ellipse is too small on screen, not really an error
   if (error == ROX_ERROR_INSUFFICIENT_DATA)
      error = ROX_ERROR_NONE;
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint nb_samples = 0;
   error = rox_dynvec_point2d_double_get_used(&nb_samples, dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Point2D_Double point2d = NULL;
   error = rox_dynvec_point2d_double_get_data_pointer ( &point2d, dynvec_point2d );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k = 0; k < nb_samples; k++)
   {
      Rox_Edge_Ellipse_Site_Struct toadd;

      toadd.state = 0;
      toadd.previous_convolution = 0;
      toadd.coords.u = point2d[k].u;
      toadd.coords.v = point2d[k].v;
      toadd.rhostar = 1.0;             // don't know what is for
      toadd.alpha = 0.0;               // angle of the tangent to the ellipse

      // Add the point to the edge site list
      error = rox_dynvec_edge_ellipse_site_append(edge_ellipse->sites, &toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_dynvec_point2d_double_del(&dynvec_point2d);
   return error;
}

Rox_ErrorCode rox_edge_ellipse_suppress_points(Rox_Edge_Ellipse edge_ellipse)
{
#ifdef segment
   Rox_Double theta = edge_ellipse->line_image_pixels.theta;
   Rox_Double costh = cos(theta);
   Rox_Double sinth = sin(theta);

   for ( Rox_Sint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
   {
      Rox_Edge_Ellipse_Site_Struct * site = &edge_ellipse->sites->data[idsite];

      if (fabs(sinth) > 0.9)
      {
         // Vertical line
         if (site->coords.v < edge_ellipse->pmin.v || site->coords.v > edge_ellipse->pmax.v)
         {
            site->state = 1;
         }
      }
      else if (fabs(costh) > 0.9)
      {
         // Horizontal line
         if (site->coords.u < edge_ellipse->pmin.u || site->coords.u > edge_ellipse->pmax.u)
         {
            site->state = 1;
         }
      }
      else
      {
         if (site->coords.v < edge_ellipse->pmin.v || site->coords.v > edge_ellipse->pmax.v || site->coords.u < edge_ellipse->pmin.u || site->coords.u > edge_ellipse->pmax.u)
         {
            site->state = 1;
         }
      }
   }

   // Delete sites with bad states
   Rox_Uint pos = 0;
   for ( Rox_Sint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
   {
      if (edge_ellipse->sites->data[idsite].state != 0) continue;
      edge_ellipse->sites->data[pos] = edge_ellipse->sites->data[idsite];
      pos++;
   }
   edge_ellipse->sites->used = pos;
#endif
   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_edge_ellipse_clean(Rox_Edge_Ellipse edge_ellipse)
{
    Rox_Uint pos;

    // Delete sites with bad states
    pos = 0;
    for (Rox_Uint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
    {
        if (edge_ellipse->sites->data[idsite].state != 0) continue;
        edge_ellipse->sites->data[pos] = edge_ellipse->sites->data[idsite];
        pos++;
    }
    edge_ellipse->sites->used = pos;

    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_edge_ellipse_get_valid_sites(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Edge_Ellipse edge_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!edge_ellipse || !dynvec_point2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_double_reset(dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
   {
      if (edge_ellipse->sites->data[idsite].state) continue;
      // Add here the site to the dynvec

      Rox_Point2D_Double_Struct point2d;

      point2d = edge_ellipse->sites->data[idsite].coords;

      error = rox_dynvec_point2d_double_append(dynvec_point2d, &point2d);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }


function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_ellipse_append_valid_sites(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Edge_Ellipse edge_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!edge_ellipse || !dynvec_point2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Uint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
   {
      if (edge_ellipse->sites->data[idsite].state) continue;
      // Add here the site to the dynvec

      Rox_Point2D_Double_Struct point2d;

      point2d = edge_ellipse->sites->data[idsite].coords;

      error = rox_dynvec_point2d_double_append(dynvec_point2d, &point2d);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_ellipse_separated_inliers_outliers(Rox_Edge_Ellipse edge_ellipse, Rox_Ellipse2D_Parametric ellipse2d_parametric, const Rox_Double distance_threshold)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!edge_ellipse || !ellipse2d_parametric) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

    // Loop over all ellipse sites
   for (Rox_Uint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
   {
      Rox_Edge_Ellipse_Site_Struct * site = &edge_ellipse->sites->data[idsite];

      // Is site is already bad, ignore
      if (site->state > 0) continue;

      Rox_Double distance = 0.0;

      error = rox_distance_point2d_to_ellipse2d_algebraic(&distance, &site->coords, ellipse2d_parametric);
      if (distance > distance_threshold)
      {
         site->state = 1;
      }
   }

function_terminate:
   return error;
}
