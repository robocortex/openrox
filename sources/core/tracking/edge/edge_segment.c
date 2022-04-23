//==============================================================================
//
//    OPENROX   : File edge_segment.c
//
//    Contents  : Implementation of edge segment module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edge_segment.h"

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/segment/segment2d.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/point/point2d_projection_from_point3d.h>

#include <generated/dynvec_edge_segment_site_struct.h>

#include <baseproc/geometry/line/line_from_points.h>
#include <baseproc/geometry/line/line_project.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edge_segment_new ( Rox_Edge_Segment * edge_segment, const Rox_Sint sampling_step )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Segment ret = NULL;

   if (!edge_segment) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *edge_segment = NULL;

   ret = (Rox_Edge_Segment) rox_memory_allocate(sizeof(struct Rox_Edge_Segment_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->sites = NULL;
   error = rox_dynvec_edge_segment_site_new ( &ret->sites, 1000 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->expected_density = 1;
   ret->sampling_step = sampling_step;

   *edge_segment = ret;

function_terminate:
   if (error) rox_edge_segment_del(&ret);
   return error;
}

Rox_ErrorCode rox_edge_segment_del ( Rox_Edge_Segment * edge_segment )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Segment todel = NULL;

   if ( !edge_segment ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *edge_segment;
   *edge_segment = NULL;

   rox_dynvec_edge_segment_site_del(&todel->sites);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_segment_set_segment3d (
   Rox_Edge_Segment edge_segment,
   Rox_Segment3D segment3d
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!edge_segment) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!segment3d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   edge_segment->segment_world = *segment3d;


function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_segment_transform_project (
   Rox_Edge_Segment edge_segment,
   Rox_MatSE3 pose,
   Rox_MatUT3 calibration
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !edge_segment || !pose || !calibration ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Transform segment extremities in camera space
   error = rox_point3d_double_transform ( edge_segment->segment_cam.points, pose, edge_segment->segment_world.points, 2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute parametric line representation in camera space
   error = rox_line3d_planes_from_2_point3d ( &edge_segment->line, &edge_segment->segment_cam.points[0], &edge_segment->segment_cam.points[1]);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute rho theta projection of the line in meters
   error = rox_line3d_planes_project_meters ( &edge_segment->line_image_meters, &(edge_segment->line) );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute rho theta projection of the line in pixels
   error = rox_line2d_transform_meters_to_pixels ( &edge_segment->line_image_pixels, &edge_segment->line_image_meters, calibration );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute extremities of image segment by projecting points into the image
   error = rox_point2d_double_project ( edge_segment->segment_image.points, edge_segment->segment_cam.points, calibration, 2 );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
  return error;
}

Rox_ErrorCode rox_edge_segment_sample ( Rox_Edge_Segment edge_segment )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!edge_segment) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_edge_segment_site_reset ( edge_segment->sites );

   Rox_Segment2D segment2d = &(edge_segment->segment_image);

   Rox_DynVec_Point2D_Double dynvec_point2d = NULL;

   error = rox_dynvec_point2d_double_new ( &dynvec_point2d, 1000 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Double sampling_step = edge_segment->sampling_step;

   error = rox_segment2d_sample ( dynvec_point2d, segment2d, sampling_step );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint nb_samples = 0;
   error = rox_dynvec_point2d_double_get_used ( &nb_samples, dynvec_point2d );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Rox_Double expected_density = nb_samples;

   Rox_Point2D_Double point2d = NULL;
   error = rox_dynvec_point2d_double_get_data_pointer ( &point2d, dynvec_point2d );
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k = 0; k < nb_samples; k++)
   {
      Rox_Edge_Segment_Site_Struct toadd;

      toadd.state = 0;
      toadd.previous_convolution = 0;
      toadd.coords.u = point2d[k].u;
      toadd.coords.v = point2d[k].v;
      toadd.rhostar = 1.0;                                  // don't know what is for
      toadd.alpha = edge_segment->line_image_pixels.theta;  // angle of the tangent to the egde

      // Add the point to the edge site list
      error = rox_dynvec_edge_segment_site_append(edge_segment->sites, &toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_dynvec_point2d_double_del(&dynvec_point2d);
   return error;
}

Rox_ErrorCode rox_edge_segment_suppress_points ( Rox_Edge_Segment edge_segment )
{
   Rox_Double theta = edge_segment->line_image_pixels.theta;
   Rox_Double costh = cos(theta);
   Rox_Double sinth = sin(theta);

   for (Rox_Uint idsite = 0; idsite < edge_segment->sites->used; idsite++)
   {
      Rox_Edge_Segment_Site_Struct * site = &edge_segment->sites->data[idsite];

      if (fabs(sinth) > 0.9)
      {
         // Vertical line
         if (site->coords.v < edge_segment->pmin.v || site->coords.v > edge_segment->pmax.v)
         {
            site->state = 1;
         }
      }
      else if (fabs(costh) > 0.9)
      {
         // Horizontal line
         if (site->coords.u < edge_segment->pmin.u || site->coords.u > edge_segment->pmax.u)
         {
            site->state = 1;
         }
      }
      else
      {
         if (site->coords.v < edge_segment->pmin.v || site->coords.v > edge_segment->pmax.v || site->coords.u < edge_segment->pmin.u || site->coords.u > edge_segment->pmax.u)
         {
            site->state = 1;
         }
      }
   }

   // Delete sites with bad states
   Rox_Uint pos = 0;
   for (Rox_Uint idsite = 0; idsite < edge_segment->sites->used; idsite++)
   {
      if (edge_segment->sites->data[idsite].state != 0) continue;
      edge_segment->sites->data[pos] = edge_segment->sites->data[idsite];
      pos++;
   }
   edge_segment->sites->used = pos;

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_edge_segment_clean(Rox_Edge_Segment edge_segment)
{    
    // Delete sites with bad states
    Rox_Uint pos = 0;
    for (Rox_Uint idsite = 0; idsite < edge_segment->sites->used; idsite++)
    {
        if (edge_segment->sites->data[idsite].state != 0) continue;
        edge_segment->sites->data[pos] = edge_segment->sites->data[idsite];
        pos++;
    }
    edge_segment->sites->used = pos;

    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_edge_segment_get_valid_measures ( Rox_Sint * valid_measures, Rox_Edge_Segment edge_segment)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!edge_segment || !valid_measures) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *valid_measures = 0;
   for (Rox_Uint idsite = 0; idsite < edge_segment->sites->used; idsite++)
   {
      if (edge_segment->sites->data[idsite].state) continue;

      (*valid_measures)++;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_segment_log_segments ( Rox_Edge_Segment seg, Rox_Sint id, Rox_Sint iter )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   char filename[FILENAME_MAX];
   Rox_Uint nbr_sites = 0;
   Rox_Edge_Segment_Site_Struct * sites = NULL;
   FILE* file = NULL;

   //write lines planes
   sprintf(filename, "planesSegment_%04d_iter_%04d.txt", id, iter);

   file = fopen(filename, "a");
   fprintf(file, "%f %f %f %f\n\n", seg->line.planes[0].a, seg->line.planes[0].b, seg->line.planes[0].c, seg->line.planes[0].d);
   fprintf(file, "%f %f %f %f\n\n", seg->line.planes[1].a, seg->line.planes[1].b, seg->line.planes[1].c, seg->line.planes[1].d);
   fclose(file);

   // write segments
   sprintf(filename, "segment_%04d_iter_%04d.txt", id, iter);

   file = fopen(filename, "a");
   fprintf(file, "%f %f %f %f\n\n", seg->segment_image.points[0].u, seg->segment_image.points[0].v, seg->segment_image.points[1].u, seg->segment_image.points[1].v);
   fclose(file);

   // write segments sites
   sprintf(filename, "segment_sites_%04d_iter_%04d.txt", id, iter);

   file = fopen(filename, "a");
   error = rox_dynvec_edge_segment_site_get_used(&nbr_sites, seg->sites);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_edge_segment_site_get_data_pointer( &sites, seg->sites );
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
