//==============================================================================
//
//    OPENROX   : File edge_cylinder.c
//
//    Contents  : Implementation of edge cylinder module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edge_cylinder.h"

#include <baseproc/maths/maths_macros.h>

#include <generated/dynvec_edge_cylinder_site_struct.h>
#include <generated/dynvec_point2d_double_struct.h>

#include <baseproc/maths/linalg/matsl3.h>

#include <baseproc/geometry/line/line2d_struct.h>
#include <baseproc/geometry/segment/segment2d.h>
#include <baseproc/geometry/cylinder/cylinder2d_struct.h>
#include <baseproc/geometry/cylinder/cylinder2d.h>
#include <baseproc/geometry/cylinder/cylinder3d.h>
#include <baseproc/geometry/cylinder/cylinder_project.h>
#include <baseproc/geometry/cylinder/cylinder_transform.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edge_cylinder_new(Rox_Edge_Cylinder * edge_cylinder, const Rox_Sint sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Cylinder ret = NULL;


   if (!edge_cylinder) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *edge_cylinder = NULL;

   ret = (Rox_Edge_Cylinder) rox_memory_allocate(sizeof(struct Rox_Edge_Cylinder_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->cylinder3d_o = NULL;
   error = rox_cylinder3d_new(&ret->cylinder3d_o);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->cylinder3d_c = NULL;
   error = rox_cylinder3d_new(&ret->cylinder3d_c);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->cylinder2d_pixels = NULL;
   error = rox_cylinder2d_new(&ret->cylinder2d_pixels);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->cylinder2d_meters = NULL;
   error = rox_cylinder2d_new(&ret->cylinder2d_meters);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->sites_segment_1 = NULL;
   error = rox_dynvec_edge_cylinder_site_new(&ret->sites_segment_1, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->sites_segment_2 = NULL;
   error = rox_dynvec_edge_cylinder_site_new(&ret->sites_segment_2, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->sites_ellipse_1 = NULL;
   error = rox_dynvec_edge_cylinder_site_new(&ret->sites_ellipse_1, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->sites_ellipse_2 = NULL;
   error = rox_dynvec_edge_cylinder_site_new(&ret->sites_ellipse_2, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->expected_density = 1;

   ret->sampling_step = sampling_step;

   *edge_cylinder = ret;

function_terminate:
   if (error) rox_edge_cylinder_del(&ret);
   return error;
}

Rox_ErrorCode rox_edge_cylinder_del(Rox_Edge_Cylinder * edge_cylinder)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Edge_Cylinder todel = NULL;


   if (!edge_cylinder) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *edge_cylinder;
   *edge_cylinder = NULL;

   rox_cylinder3d_del(&todel->cylinder3d_o);
   rox_cylinder3d_del(&todel->cylinder3d_c);
   rox_cylinder2d_del(&todel->cylinder2d_pixels);
   rox_cylinder2d_del(&todel->cylinder2d_meters);
   rox_dynvec_edge_cylinder_site_del(&todel->sites_segment_1);
   rox_dynvec_edge_cylinder_site_del(&todel->sites_segment_2);
   rox_dynvec_edge_cylinder_site_del(&todel->sites_ellipse_1);
   rox_dynvec_edge_cylinder_site_del(&todel->sites_ellipse_2);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_cylinder_set_cylinder3d(Rox_Edge_Cylinder edge_cylinder, Rox_Cylinder3D cylinder3d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!edge_cylinder) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_cylinder3d_copy(edge_cylinder->cylinder3d_o, cylinder3d);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_edge_cylinder_transform_project(Rox_Edge_Cylinder edge_cylinder, Rox_Array2D_Double pose, Rox_Array2D_Double calibration)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!edge_cylinder || !pose || !calibration) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calibration, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Define a calibration matrix equal to identity
   Rox_MatSL3 I_3x3 = NULL;
   error = rox_matsl3_new(&I_3x3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_MatSE3 I_4x4 = NULL;
   error = rox_matse3_new(&I_4x4);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
if(0)
{
   // Transform the cylinder in the camera frame
   error = rox_cylinder3d_transform(edge_cylinder->cylinder3d_c, pose, edge_cylinder->cylinder3d_o);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Project the 3D cylinder in the image : 2D cylinder in pixels coordinates
   error = rox_cylinder2d_project_cylinder3d(edge_cylinder->cylinder2d_meters, I_3x3, edge_cylinder->cylinder3d_c);
   ROX_ERROR_CHECK_TERMINATE ( error );
}
else
{
   // Transform the cylinder in the camera frame
   error = rox_cylinder3d_transform(edge_cylinder->cylinder3d_c, pose, edge_cylinder->cylinder3d_o);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_cylinder2d_transform_project_cylinder3d(edge_cylinder->cylinder2d_meters, I_3x3, pose, edge_cylinder->cylinder3d_o);
   ROX_ERROR_CHECK_TERMINATE ( error );
}

   // Tranform the cylinder from pixels to meters coordinates
   error = rox_cylinder2d_transform(edge_cylinder->cylinder2d_pixels, calibration, edge_cylinder->cylinder2d_meters);
   ROX_ERROR_CHECK_TERMINATE ( error );


   // Get tangent segments in camera frame
   error = rox_cylinder3d_get_tangent_segments(&edge_cylinder->tangent_segment3d_1, &edge_cylinder->tangent_segment3d_2, I_4x4, edge_cylinder->cylinder3d_c);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   rox_matsl3_del(&I_3x3);
   rox_matsl3_del(&I_4x4);
 return error;
}

Rox_ErrorCode rox_edge_cylinder_sample(Rox_Edge_Cylinder edge_cylinder)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!edge_cylinder) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_edge_cylinder_site_reset(edge_cylinder->sites_segment_1);
   rox_dynvec_edge_cylinder_site_reset(edge_cylinder->sites_segment_2);

   //rox_dynvec_edge_cylinder_site_reset(edge_cylinder->sites_ellipse_1);
   //rox_dynvec_edge_cylinder_site_reset(edge_cylinder->sites_ellipse_2);

   Rox_Cylinder2D cylinder2d = edge_cylinder->cylinder2d_pixels;
   Rox_Point2D_Double point2d = NULL;

   Rox_Sint sampling_step = edge_cylinder->sampling_step;
   Rox_Uint nb_samples = 0;

   //---------------------------------------------------------------------------
   Rox_Segment2D segment2d_1 = cylinder2d->s1;
   // Sample the segment 1
   Rox_DynVec_Point2D_Double dynvec_point2d_segment2d_1 = NULL;
   error = rox_dynvec_point2d_double_new(&dynvec_point2d_segment2d_1, 200);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_segment2d_sample(dynvec_point2d_segment2d_1, segment2d_1, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_get_used(&nb_samples, dynvec_point2d_segment2d_1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_get_data_pointer ( &point2d, dynvec_point2d_segment2d_1 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Line2D_Normal_Struct line2d_1_struct;

   error = rox_segment2d_get_line2d_normal(&line2d_1_struct, segment2d_1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k = 0; k < nb_samples; k++)
   {
      Rox_Edge_Cylinder_Site_Struct toadd;

      toadd.state = 0;
      toadd.previous_convolution = 0;
      toadd.coords.u = point2d[k].u;
      toadd.coords.v = point2d[k].v;
      toadd.rhostar = 1.0;                // don't know what is for
      toadd.alpha = line2d_1_struct.theta;  // angle of the tangent to the cylinder

      // Add the point to the edge site list
      error = rox_dynvec_edge_cylinder_site_append(edge_cylinder->sites_segment_1, &toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   //---------------------------------------------------------------------------
   Rox_Segment2D segment2d_2 = cylinder2d->s2;
   // Sample the segment 2
   Rox_DynVec_Point2D_Double dynvec_point2d_segment2d_2 = NULL;
   error = rox_dynvec_point2d_double_new(&dynvec_point2d_segment2d_2, 100);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_segment2d_sample(dynvec_point2d_segment2d_2, segment2d_2, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_get_used(&nb_samples, dynvec_point2d_segment2d_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_get_data_pointer ( &point2d, dynvec_point2d_segment2d_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Line2D_Normal_Struct line2d_2_struct;

   error = rox_segment2d_get_line2d_normal(&line2d_2_struct, segment2d_2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint k = 0; k < nb_samples; k++)
   {
      Rox_Edge_Cylinder_Site_Struct toadd;

      toadd.state = 0;
      toadd.previous_convolution = 0;
      toadd.coords.u = point2d[k].u;
      toadd.coords.v = point2d[k].v;
      toadd.rhostar = 1.0;             // don't know what is for
      toadd.alpha = line2d_2_struct.theta;               // angle of the tangent to the edge

      // Add the point to the edge site list
      error = rox_dynvec_edge_cylinder_site_append(edge_cylinder->sites_segment_2, &toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   //---------------------------------------------------------------------------
   // TODO : Sample the ellipses

function_terminate:
   rox_dynvec_point2d_double_del(&dynvec_point2d_segment2d_1);
   rox_dynvec_point2d_double_del(&dynvec_point2d_segment2d_2);

   return error;
}

Rox_ErrorCode rox_edge_cylinder_suppress_points(Rox_Edge_Cylinder edge_cylinder)
{
#ifdef segment
   Rox_Double theta = edge_cylinder->line_image_pixels.theta;
   Rox_Double costh = cos(theta);
   Rox_Double sinth = sin(theta);

   for ( Rox_Sint idsite = 0; idsite < edge_cylinder->sites->used; idsite++)
   {
      Rox_Edge_Cylinder_Site_Struct * site = &edge_cylinder->sites->data[idsite];

      if (fabs(sinth) > 0.9)
      {
         // Vertical line
         if (site->coords.v < edge_cylinder->pmin.v || site->coords.v > edge_cylinder->pmax.v)
         {
            site->state = 1;
         }
      }
      else if (fabs(costh) > 0.9)
      {
         // Horizontal line
         if (site->coords.u < edge_cylinder->pmin.u || site->coords.u > edge_cylinder->pmax.u)
         {
            site->state = 1;
         }
      }
      else
      {
         if (site->coords.v < edge_cylinder->pmin.v || site->coords.v > edge_cylinder->pmax.v || site->coords.u < edge_cylinder->pmin.u || site->coords.u > edge_cylinder->pmax.u)
         {
            site->state = 1;
         }
      }
   }

   // Delete sites with bad states
   Rox_Uint pos = 0;
   for ( Rox_Sint idsite = 0; idsite < edge_cylinder->sites->used; idsite++)
   {
      if (edge_cylinder->sites->data[idsite].state != 0) continue;
      edge_cylinder->sites->data[pos] = edge_cylinder->sites->data[idsite];
      pos++;
   }
   edge_cylinder->sites->used = pos;
#endif

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_edge_cylinder_clean(Rox_Edge_Cylinder edge_cylinder)
{
    Rox_Uint pos;

    // Delete sites with bad states in segment_1
    pos = 0;
    for (Rox_Uint idsite = 0; idsite < edge_cylinder->sites_segment_1->used; idsite++)
    {
        if (edge_cylinder->sites_segment_1->data[idsite].state != 0) continue;
        edge_cylinder->sites_segment_1->data[pos] = edge_cylinder->sites_segment_1->data[idsite];
        pos++;
    }
    edge_cylinder->sites_segment_1->used = pos;

    // Delete sites with bad states in segment_2
    pos = 0;
    for (Rox_Uint idsite = 0; idsite < edge_cylinder->sites_segment_2->used; idsite++)
    {
        if (edge_cylinder->sites_segment_2->data[idsite].state != 0) continue;
        edge_cylinder->sites_segment_2->data[pos] = edge_cylinder->sites_segment_2->data[idsite];
        pos++;
    }
    edge_cylinder->sites_segment_2->used = pos;

    return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_edge_cylinder_get_valid_measures(Rox_Sint * valid_measures, Rox_Edge_Cylinder edge_cylinder)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!edge_cylinder || !valid_measures) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *valid_measures = 0;

   // Count valid measures for segment 1
   for (Rox_Uint idsite = 0; idsite < edge_cylinder->sites_segment_1->used; idsite++)
   {
      if (edge_cylinder->sites_segment_1->data[idsite].state) continue;
      (*valid_measures)++;
   }

   // Count valid measures for segment 2
   for (Rox_Uint idsite = 0; idsite < edge_cylinder->sites_segment_2->used; idsite++)
   {
      if (edge_cylinder->sites_segment_2->data[idsite].state) continue;
      (*valid_measures)++;
   }

   // TODO : Count valid measures for ellipses 1 and 2

function_terminate:
   return error;
}
