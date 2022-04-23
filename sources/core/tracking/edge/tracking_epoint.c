//==============================================================================
//
//    OPENROX   : File tracking_epoint.c
//
//    Contents  : Implementation of tracking_epoint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license S.A.S.
//
//=============================================================================

#include "tracking_epoint.h"
#include <generated/dynvec_edge_point_site_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_tracking_epoint_new (
   Rox_Tracking_EPoint * tracking_epoint,
   const Rox_Tracking_EPoint_Method method,
   const Rox_Sint search_range,
   const Rox_Double contrast_threshold
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_EPoint ret = NULL;

   if (!tracking_epoint)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *tracking_epoint = NULL;

   ret = (Rox_Tracking_EPoint) rox_memory_allocate(sizeof(*ret), 1);
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

      case RoxTrackingEPointMethod_Moving_Edge:
      {
         error = rox_moving_edge_params_new(&ret->params, search_range, contrast_threshold);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_moving_edge_new(&ret->medge, ret->params);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      break;

      case RoxTrackingEPointMethod_Search_Edge:
      {
         error = rox_search_edge_new ( &ret->search_edge, search_range, (Rox_Uint) contrast_threshold);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      break;
   }

   *tracking_epoint = ret;

function_terminate:
   if (error) rox_tracking_epoint_del(&ret);
   return error;
}

Rox_ErrorCode rox_tracking_epoint_del (
   Rox_Tracking_EPoint * tracking_epoint
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_EPoint todel = NULL;

   if ( !tracking_epoint )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *tracking_epoint;
   *tracking_epoint = NULL;

   rox_search_edge_del(&todel->search_edge);
   rox_moving_edge_del(&todel->medge);
   rox_moving_edge_params_del(&todel->params);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_epoint_make (
   Rox_Tracking_EPoint tracking_epoint,
   Rox_Image image,
   Rox_Edge_Point point
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!tracking_epoint || !image || !point)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Loop over all point sites for edgelet track
   for (Rox_Uint  idsite = 0; idsite < point->sites->used; idsite++)
   {
      Rox_Edge_Point_Site_Struct * site = &point->sites->data[idsite];

      // Is site is already bad, ignore
      if (site->state > 0) continue;

      switch (tracking_epoint->method)
      {
      default:
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      break;

      case RoxTrackingEPointMethod_Moving_Edge:
      {
         if (tracking_epoint->medge == NULL)
         {
            error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
         }

         // Set current site coordinates to the moving edge
         error = rox_moving_edge_set_coordinates ( tracking_epoint->medge, site->coords.u, site->coords.v, site->alpha, site->previous_convolution);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Perform tracking
         error = rox_moving_edge_track ( tracking_epoint->medge, image, 0 );
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_epoint->medge->_convolution;
         site->coords = tracking_epoint->medge->_coords;
      }
      break;

      case RoxTrackingEPointMethod_Search_Edge:
      {
         if (tracking_epoint->search_edge == NULL)
         { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Set the angle of the edge = alpha
         tracking_epoint->search_edge->_angle = point->line2d_image_pixels.theta;

         // Perform tracking
         error = rox_search_edge_track ( tracking_epoint->search_edge, image, &site->coords);
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_epoint->search_edge->_convolution; //TODO not computed, remove struct member
         site->coords = tracking_epoint->search_edge->_coords;
      }
      break;
      }

   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_epoint_make_gradient (
   Rox_Tracking_EPoint tracking_epoint,
   const Rox_Array2D_Uint gradient_scale,
   const Rox_Array2D_Float gradient_angle,
   const Rox_Edge_Point point
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!tracking_epoint || !gradient_scale || !gradient_angle || !point)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Loop over all point sites for edgelet track
   for (Rox_Uint idsite = 0; idsite < point->sites->used; idsite++)
   {
      Rox_Edge_Point_Site_Struct * site = &point->sites->data[idsite];

      // Is site is already bad, ignore
      if (site->state > 0) continue;

      switch (tracking_epoint->method)
      {
      default:
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      break;

      case RoxTrackingEPointMethod_Moving_Edge:
      {
         if (tracking_epoint->medge == NULL)
         {
            error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
         }

         // Set current site coordinates to the moving edge
         error = rox_moving_edge_set_coordinates ( tracking_epoint->medge, site->coords.u, site->coords.v, site->alpha, site->previous_convolution);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Perform tracking
         // error = rox_moving_edge_track ( tracking_epoint->medge, image, 0 );
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_epoint->medge->_convolution;
         site->coords = tracking_epoint->medge->_coords;
      }
      break;

      case RoxTrackingEPointMethod_Search_Edge:
      {
         if (tracking_epoint->search_edge == NULL)
         {
            error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error );
         }

         // Set the angle of the edge = alpha
         tracking_epoint->search_edge->_angle = point->line2d_image_pixels.theta;

         // Perform tracking
         error = rox_search_edge_track_gradient ( tracking_epoint->search_edge, gradient_scale, gradient_angle, &site->coords);
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_epoint->search_edge->_convolution; //TODO not computed, remove struct member
         site->coords = tracking_epoint->search_edge->_coords;
      }
      break;
      }

   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_epoint_initialize (
   Rox_Tracking_EPoint tracking_epoint,
   Rox_Edge_Point point
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !tracking_epoint || !point )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   switch (tracking_epoint->method)
   {
   default:
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   break;

   case RoxTrackingEPointMethod_Moving_Edge:
   {
      if (tracking_epoint->medge == NULL)
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      for ( Rox_Uint idsite = 0; idsite < point->sites->used; idsite++)
      {
         Rox_Edge_Point_Site_Struct * site = &point->sites->data[idsite];

         // Set moving edges parameters
         error = rox_moving_edge_set_coordinates ( tracking_epoint->medge, site->coords.u, site->coords.v, site->alpha, 0.0 );
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }
   break;

   case RoxTrackingEPointMethod_Search_Edge:
   {
      if (tracking_epoint->search_edge == NULL)

      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      // Do nothing
   }
   break;
   }

function_terminate:
   return error;
}
