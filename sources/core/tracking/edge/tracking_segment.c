//==============================================================================
//
//    OPENROX   : File tracking_segment.c
//
//    Contents  : Implementation of tracking_segment module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//=============================================================================

#include "tracking_segment.h"
#include <generated/dynvec_edge_segment_site_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_tracking_segment_new (
   Rox_Tracking_Segment * tracking_segment,
   const Rox_Tracking_Segment_Method method,
   const Rox_Sint search_range,
   const Rox_Double contrast_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Segment ret = NULL;

   if (!tracking_segment)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *tracking_segment = NULL;

   ret = (Rox_Tracking_Segment)rox_memory_allocate(sizeof(*ret), 1);
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

      case RoxTrackingSegmentMethod_Moving_Edge:
      {
         error = rox_moving_edge_params_new(&ret->params, search_range, contrast_threshold);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_moving_edge_new(&ret->medge, ret->params);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      break;

      case RoxTrackingSegmentMethod_Search_Edge:
      {
         error = rox_search_edge_new ( &ret->search_edge, search_range, (Rox_Uint) contrast_threshold);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      break;
   }

   *tracking_segment = ret;

function_terminate:
   if (error) rox_tracking_segment_del(&ret);
   return error;
}

Rox_ErrorCode rox_tracking_segment_del (
   Rox_Tracking_Segment * tracking_segment
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Segment todel = NULL;

   if ( !tracking_segment )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *tracking_segment;
   *tracking_segment = NULL;

   rox_search_edge_del(&todel->search_edge);
   rox_moving_edge_del(&todel->medge);
   rox_moving_edge_params_del(&todel->params);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_segment_make (
   Rox_Tracking_Segment tracking_segment,
   Rox_Image image,
   Rox_Edge_Segment segment
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!tracking_segment || !image || !segment)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Loop over all segment sites for edgelet track
   for (Rox_Uint  idsite = 0; idsite < segment->sites->used; idsite++)
   {
      Rox_Edge_Segment_Site_Struct * site = &segment->sites->data[idsite];

      // Is site is already bad, ignore
      if (site->state > 0) continue;

      switch (tracking_segment->method)
      {
      default:
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      break;

      case RoxTrackingSegmentMethod_Moving_Edge:
      {
         if (tracking_segment->medge == NULL)
         {
            error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
         }

         // Set current site coordinates to the moving edge
         error = rox_moving_edge_set_coordinates ( tracking_segment->medge, site->coords.u, site->coords.v, site->alpha, site->previous_convolution);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Perform tracking
         error = rox_moving_edge_track ( tracking_segment->medge, image, 0 );
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_segment->medge->_convolution;
         site->coords = tracking_segment->medge->_coords;
      }
      break;

      case RoxTrackingSegmentMethod_Search_Edge:
      {
         if (tracking_segment->search_edge == NULL)
         { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Set the angle of the edge = alpha
         tracking_segment->search_edge->_angle = segment->line_image_pixels.theta;

         // Perform tracking
         error = rox_search_edge_track ( tracking_segment->search_edge, image, &site->coords);
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_segment->search_edge->_convolution; //TODO not computed, remove struct member
         site->coords = tracking_segment->search_edge->_coords;
      }
      break;
      }

   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_segment_make_gradient (
   Rox_Tracking_Segment tracking_segment,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle,
   const Rox_Edge_Segment segment
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!tracking_segment || !gradient_scale || !gradient_angle || !segment)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Loop over all segment sites for edgelet track
   for (Rox_Uint  idsite = 0; idsite < segment->sites->used; idsite++)
   {
      Rox_Edge_Segment_Site_Struct * site = &segment->sites->data[idsite];

      // Is site is already bad, ignore
      if (site->state > 0) continue;

      switch (tracking_segment->method)
      {
      default:
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      break;

      case RoxTrackingSegmentMethod_Moving_Edge:
      {
         if (tracking_segment->medge == NULL)
         {
            error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
         }

         // Set current site coordinates to the moving edge
         error = rox_moving_edge_set_coordinates ( tracking_segment->medge, site->coords.u, site->coords.v, site->alpha, site->previous_convolution);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Perform tracking
         // error = rox_moving_edge_track ( tracking_segment->medge, image, 0 );
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_segment->medge->_convolution;
         site->coords = tracking_segment->medge->_coords;
      }
      break;

      case RoxTrackingSegmentMethod_Search_Edge:
      {
         if (tracking_segment->search_edge == NULL)
         {
            error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
         }

         // Set the angle of the edge = alpha
         tracking_segment->search_edge->_angle = segment->line_image_pixels.theta;

         // Perform tracking
         error = rox_search_edge_track_gradient ( tracking_segment->search_edge, gradient_scale, gradient_angle, &site->coords);
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_segment->search_edge->_convolution; //TODO not computed, remove struct member
         site->coords = tracking_segment->search_edge->_coords;
      }
      break;
      }

   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_segment_initialize (
   Rox_Tracking_Segment tracking_segment,
   Rox_Edge_Segment segment
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !tracking_segment || !segment )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   switch (tracking_segment->method)
   {
   default:
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   break;

   case RoxTrackingSegmentMethod_Moving_Edge:
   {
      if (tracking_segment->medge == NULL)
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      for ( Rox_Uint idsite = 0; idsite < segment->sites->used; idsite++)
      {
         Rox_Edge_Segment_Site_Struct * site = &segment->sites->data[idsite];

         // Set moving edges parameters
         error = rox_moving_edge_set_coordinates(tracking_segment->medge, site->coords.u, site->coords.v, site->alpha, 0.0);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }
   break;

   case RoxTrackingSegmentMethod_Search_Edge:
   {
      if (tracking_segment->search_edge == NULL)

      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      // Do nothing
   }
   break;
   }

function_terminate:
   return error;
}
