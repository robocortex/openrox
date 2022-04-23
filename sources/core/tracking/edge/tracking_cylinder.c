//==============================================================================
//
//    OPENROX   : File tracking_cylinder.c
//
//    Contents  : Implementation of tracking_cylinder module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

#include "tracking_cylinder.h"

#include <generated/dynvec_edge_cylinder_site_struct.h>

#include <baseproc/geometry/cylinder/cylinder2d.h>

#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_tracking_cylinder_new(Rox_Tracking_Cylinder * tracking_cylinder, const Rox_Tracking_Cylinder_Method method, const Rox_Sint search_range, const Rox_Double contrast_threshold)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Cylinder ret = NULL;


   if (!tracking_cylinder)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *tracking_cylinder = NULL;

   ret = (Rox_Tracking_Cylinder)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->medge = NULL;
   ret->params = NULL;
   ret->search_edge = NULL;
   ret->method = method;

   switch (method)
   {
      default:
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      break;

      case RoxTrackingCylinderMethod_Moving_Edge:
      {
         error = rox_moving_edge_params_new(&ret->params, search_range, contrast_threshold);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_moving_edge_new(&ret->medge, ret->params);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      break;

      case RoxTrackingCylinderMethod_Search_Edge:
      {

         error = rox_search_edge_new(&ret->search_edge, search_range, (Rox_Uint)contrast_threshold);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      break;
   }

   *tracking_cylinder = ret;

function_terminate:
   if (error) rox_tracking_cylinder_del(&ret);
   return error;
}

Rox_ErrorCode rox_tracking_cylinder_del(Rox_Tracking_Cylinder * tracking_cylinder)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Cylinder todel = NULL;


   if (!tracking_cylinder)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *tracking_cylinder;
   *tracking_cylinder = NULL;

   rox_search_edge_del(&todel->search_edge);
   rox_moving_edge_del(&todel->medge);
   rox_moving_edge_params_del(&todel->params);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_cylinder_make (
   Rox_Tracking_Cylinder tracking_cylinder,
   Rox_Image image,
   Rox_Edge_Cylinder edge_cylinder
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!tracking_cylinder || !image || !edge_cylinder)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Loop over all cylinder sites for edgelet track for segment 1
   for (Rox_Uint  idsite = 0; idsite < edge_cylinder->sites_segment_1->used; idsite++)
   {
      Rox_Edge_Cylinder_Site_Struct * site = &edge_cylinder->sites_segment_1->data[idsite];

      // Is site is already bad, ignore
      if (site->state > 0) continue;

      switch (tracking_cylinder->method)
      {
      default:
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      break;

      case RoxTrackingCylinderMethod_Moving_Edge:
      {
         if (tracking_cylinder->medge == NULL)
         { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Set current site coordinates to the moving edge
         error = rox_moving_edge_set_coordinates(tracking_cylinder->medge, site->coords.u, site->coords.v, site->alpha, site->previous_convolution);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Perform tracking
         error = rox_moving_edge_track(tracking_cylinder->medge, image, 0);
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         //Store result
         site->previous_convolution = tracking_cylinder->medge->_convolution;
         site->coords = tracking_cylinder->medge->_coords;
      }
      break;

      case RoxTrackingCylinderMethod_Search_Edge:
      {
         if (tracking_cylinder->search_edge == NULL)
         { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Set the angle of the edge
         // Rox_Double angle = 0.0;
         // error = rox_cylinder2d_get_normal_angle(&angle, &edge_cylinder->cylinder2d_pixels, site->coords.u, site->coords.v); // edge_cylinder->line_image_pixels.theta;
         // ROX_ERROR_CHECK_TERMINATE ( error );
         // tracking_cylinder->search_edge->_angle = angle;
         tracking_cylinder->search_edge->_angle = site->alpha;

         // Perform tracking
         error = rox_search_edge_track(tracking_cylinder->search_edge, image, &site->coords);
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_cylinder->search_edge->_convolution; // TODO not computed, remove struct member
         site->coords = tracking_cylinder->search_edge->_coords;
      }
      break;
      }

   }

   // Loop over all cylinder sites for edgelet track for segment 1
   for (Rox_Uint  idsite = 0; idsite < edge_cylinder->sites_segment_2->used; idsite++)
   {
      Rox_Edge_Cylinder_Site_Struct * site = &edge_cylinder->sites_segment_2->data[idsite];

      // Is site is already bad, ignore
      if (site->state > 0) continue;

      switch (tracking_cylinder->method)
      {
      default:
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      break;

      case RoxTrackingCylinderMethod_Moving_Edge:
      {
         if (tracking_cylinder->medge == NULL)
         { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Set current site coordinates to the moving edge
         error = rox_moving_edge_set_coordinates(tracking_cylinder->medge, site->coords.u, site->coords.v, site->alpha, site->previous_convolution);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Perform tracking
         error = rox_moving_edge_track(tracking_cylinder->medge, image, 0);
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         //Store result
         site->previous_convolution = tracking_cylinder->medge->_convolution;
         site->coords = tracking_cylinder->medge->_coords;
      }
      break;

      case RoxTrackingCylinderMethod_Search_Edge:
      {
         if (tracking_cylinder->search_edge == NULL)
         { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Set the angle of the edge
         // Rox_Double angle = 0.0;
         // error = rox_cylinder2d_get_normal_angle(&angle, &edge_cylinder->cylinder2d_pixels, site->coords.u, site->coords.v); // edge_cylinder->line_image_pixels.theta;
         // ROX_ERROR_CHECK_TERMINATE ( error );
         // tracking_cylinder->search_edge->_angle = angle;
         tracking_cylinder->search_edge->_angle = site->alpha;

         // Perform tracking
         error = rox_search_edge_track(tracking_cylinder->search_edge, image, &site->coords);
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_cylinder->search_edge->_convolution; // TODO not computed, remove struct member
         site->coords = tracking_cylinder->search_edge->_coords;
      }
      break;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_cylinder_initialize (
   Rox_Tracking_Cylinder tracking_cylinder,
   Rox_Image image,
   Rox_Edge_Cylinder edge_cylinder
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!tracking_cylinder || !image || !edge_cylinder)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   switch (tracking_cylinder->method)
   {
   default:
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   break;

   case RoxTrackingCylinderMethod_Moving_Edge:
   {
      if (tracking_cylinder->medge == NULL)
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      for (Rox_Uint idsite = 0; idsite < edge_cylinder->sites_segment_1->used; idsite++)
      {
         Rox_Edge_Cylinder_Site_Struct * site = &edge_cylinder->sites_segment_1->data[idsite];

         // Set moving edges parameters
         error = rox_moving_edge_set_coordinates(tracking_cylinder->medge, site->coords.u, site->coords.v, site->alpha, 0.0);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }

      for ( Rox_Uint idsite = 0; idsite < edge_cylinder->sites_segment_2->used; idsite++)
      {
         Rox_Edge_Cylinder_Site_Struct * site = &edge_cylinder->sites_segment_2->data[idsite];

         // Set moving edges parameters
         error = rox_moving_edge_set_coordinates(tracking_cylinder->medge, site->coords.u, site->coords.v, site->alpha, 0.0);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }
   break;

   case RoxTrackingCylinderMethod_Search_Edge:
   {
      if (tracking_cylinder->search_edge == NULL)

      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      // Do nothing
   }
   break;
   }

function_terminate:
   return error;
}


