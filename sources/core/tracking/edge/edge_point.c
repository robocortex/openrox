//==============================================================================
//
//    OPENROX   : File edge_point.c
//
//    Contents  : Implementation of edge point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edge_point.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/segment/segment2d.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>

#include <generated/dynvec_edge_point_site_struct.h>

#include <baseproc/geometry/line/line_from_points.h>
#include <baseproc/geometry/line/line_transform.h>
#include <baseproc/geometry/line/line_project.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edge_point_new ( Rox_Edge_Point * edge_point )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Point ret = NULL;

   if (!edge_point) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *edge_point = NULL;

   ret = (Rox_Edge_Point) rox_memory_allocate(sizeof(struct Rox_Edge_Point_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->sites = NULL;
   error = rox_dynvec_edge_point_site_new ( &ret->sites, 1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   *edge_point = ret;

function_terminate:
   if (error) rox_edge_point_del(&ret);
   return error;
}

Rox_ErrorCode rox_edge_point_del ( Rox_Edge_Point * edge_point )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Point todel = NULL;

   if ( !edge_point ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *edge_point;
   *edge_point = NULL;

   rox_dynvec_edge_point_site_del(&todel->sites);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_point_set_point (
   Rox_Edge_Point edge_point,
   const Rox_Point2D_Double point2d,
   const Rox_Line3D_Planes line3d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !edge_point ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !point2d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !line3d ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   edge_point->point2d_image = *point2d;
   edge_point->line3d_world = *line3d;

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_point_transform_project (
   Rox_Edge_Point edge_point,
   Rox_MatSE3 pose,
   Rox_MatUT3 calibration
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !edge_point || !pose || !calibration ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Compute parametric line representation in camera space
   error = rox_line3d_planes_transform ( &edge_point->line3d_camera, &edge_point->line3d_world, pose );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute rho theta projection of the line in meters
   error = rox_line3d_planes_project_meters ( &edge_point->line2d_image_meters, &(edge_point->line3d_camera) );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute rho theta projection of the line in pixels
   error = rox_line2d_transform_meters_to_pixels ( &edge_point->line2d_image_pixels, &edge_point->line2d_image_meters, calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}

Rox_ErrorCode rox_edge_point_sample ( Rox_Edge_Point edge_point )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if ( !edge_point ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_edge_point_site_reset ( edge_point->sites );

   Rox_Edge_Point_Site_Struct toadd;

   toadd.state = 0;
   toadd.previous_convolution = 0;
   toadd.coords.u = edge_point->point2d_image.u;
   toadd.coords.v = edge_point->point2d_image.v;
   toadd.rhostar = 1.0;                                  // don't know what is for
   toadd.alpha = edge_point->line2d_image_pixels.theta;  // angle of the tangent to the edge

   // Add the point to the edge site list
   error = rox_dynvec_edge_point_site_append ( edge_point->sites, &toadd );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_point_suppress_points ( Rox_Edge_Point edge_point )
{
   Rox_Double theta = edge_point->line2d_image_pixels.theta;
   Rox_Double costh = cos(theta);
   Rox_Double sinth = sin(theta);

   for (Rox_Uint idsite = 0; idsite < edge_point->sites->used; idsite++)
   {
      Rox_Edge_Point_Site_Struct * site = &edge_point->sites->data[idsite];

      if (fabs(sinth) > 0.9)
      {
         // Vertical line
         if (site->coords.v < edge_point->pmin.v || site->coords.v > edge_point->pmax.v)
         {
            site->state = 1;
         }
      }
      else if (fabs(costh) > 0.9)
      {
         // Horizontal line
         if (site->coords.u < edge_point->pmin.u || site->coords.u > edge_point->pmax.u)
         {
            site->state = 1;
         }
      }
      else
      {
         if (site->coords.v < edge_point->pmin.v || site->coords.v > edge_point->pmax.v || site->coords.u < edge_point->pmin.u || site->coords.u > edge_point->pmax.u)
         {
            site->state = 1;
         }
      }
   }

   // Delete sites with bad states
   Rox_Uint pos = 0;
   for (Rox_Uint idsite = 0; idsite < edge_point->sites->used; idsite++)
   {
      if (edge_point->sites->data[idsite].state != 0) continue;
      edge_point->sites->data[pos] = edge_point->sites->data[idsite];
      pos++;
   }
   edge_point->sites->used = pos;

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_edge_point_clean(Rox_Edge_Point edge_point)
{    
    // Delete sites with bad states
    Rox_Uint pos = 0;
    for (Rox_Uint idsite = 0; idsite < edge_point->sites->used; idsite++)
    {
        if (edge_point->sites->data[idsite].state != 0) continue;
        edge_point->sites->data[pos] = edge_point->sites->data[idsite];
        pos++;
    }
    edge_point->sites->used = pos;

    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_edge_point_get_valid_measures ( Rox_Sint * valid_measures, Rox_Edge_Point edge_point)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!edge_point || !valid_measures) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *valid_measures = 0;
   for (Rox_Uint idsite = 0; idsite < edge_point->sites->used; idsite++)
   {
      if (edge_point->sites->data[idsite].state) continue;

      (*valid_measures)++;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_point_log_points ( Rox_Edge_Point seg, Rox_Sint id, Rox_Sint iter )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   char filename[FILENAME_MAX];
   Rox_Uint nbr_sites = 0;
   Rox_Edge_Point_Site_Struct * sites = NULL;
   FILE* file = NULL;

   //write lines planes
   sprintf(filename, "planes_line3d_%04d_iter_%04d.txt", id, iter);

   file = fopen(filename, "a");
   fprintf(file, "%f %f %f %f\n\n", seg->line3d_camera.planes[0].a, seg->line3d_camera.planes[0].b, seg->line3d_camera.planes[0].c, seg->line3d_camera.planes[0].d);
   fprintf(file, "%f %f %f %f\n\n", seg->line3d_camera.planes[1].a, seg->line3d_camera.planes[1].b, seg->line3d_camera.planes[1].c, seg->line3d_camera.planes[1].d);
   fclose(file);

   // write points
   sprintf(filename, "point2d_%04d_iter_%04d.txt", id, iter);

   file = fopen(filename, "a");
   fprintf(file, "%f %f \n\n", seg->point2d_image.u, seg->point2d_image.v);
   fclose(file);

   // write segments sites
   sprintf(filename, "segment_sites_%04d_iter_%04d.txt", id, iter);

   file = fopen(filename, "a");
   error = rox_dynvec_edge_point_site_get_used ( &nbr_sites, seg->sites);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_edge_point_site_get_data_pointer ( &sites, seg->sites );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Uint idpt = 0; idpt < nbr_sites; idpt++)
   {
      fprintf(file, "%f %f\n\n", sites[idpt].coords.u, sites[idpt].coords.v);
   }
   fclose(file);

   // write segments sites measures
   // sprintf(filename, "segment_measures_%04d.txt", id);

function_terminate:
   return error;
}
