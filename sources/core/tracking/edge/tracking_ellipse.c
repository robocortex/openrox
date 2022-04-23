//==============================================================================
//
//    OPENROX   : File tracking_ellipse.c
//
//    Contents  : Implementation of tracking_ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//=============================================================================

#include "tracking_ellipse.h"

#include <generated/dynvec_edge_ellipse_site_struct.h>

#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/ellipse/ellipse_from_points.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>


Rox_ErrorCode rox_tracking_ellipse_new (
   Rox_Tracking_Ellipse * tracking_ellipse,
   const Rox_Tracking_Ellipse_Method method,
   const Rox_Sint search_range,
   const Rox_Double contrast_threshold)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Ellipse ret = NULL;


   if (!tracking_ellipse)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *tracking_ellipse = NULL;

   ret = (Rox_Tracking_Ellipse)rox_memory_allocate(sizeof(*ret), 1);
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

      case RoxTrackingEllipseMethod_Moving_Edge:
      {
         error = rox_moving_edge_params_new(&ret->params, search_range, contrast_threshold);
         ROX_ERROR_CHECK_TERMINATE ( error );

         error = rox_moving_edge_new(&ret->medge, ret->params);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      break;

      case RoxTrackingEllipseMethod_Search_Edge:
      {

         error = rox_search_edge_new(&ret->search_edge, search_range, (Rox_Uint) contrast_threshold);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
      break;
   }

   *tracking_ellipse = ret;

function_terminate:
   if (error) rox_tracking_ellipse_del(&ret);
   return error;
}

Rox_ErrorCode rox_tracking_ellipse_del(Rox_Tracking_Ellipse * tracking_ellipse)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tracking_Ellipse todel = NULL;


   if (!tracking_ellipse)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *tracking_ellipse;
   *tracking_ellipse = NULL;

   rox_search_edge_del(&todel->search_edge);
   rox_moving_edge_del(&todel->medge);
   rox_moving_edge_params_del(&todel->params);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_ellipse_make (
   Rox_Tracking_Ellipse tracking_ellipse,
   Rox_Image image,
   Rox_Edge_Ellipse edge_ellipse
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!tracking_ellipse || !image || !edge_ellipse)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Loop over all ellipse sites for edgelet track
   for ( Rox_Uint  idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
   {
      Rox_Edge_Ellipse_Site_Struct * site = &edge_ellipse->sites->data[idsite];

      // Is site is already bad, ignore
      if (site->state > 0) continue;

      switch (tracking_ellipse->method)
      {
      default:
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      break;

      case RoxTrackingEllipseMethod_Moving_Edge:
      {
         if (tracking_ellipse->medge == NULL)
         { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Set current site coordinates to the moving edge
         error = rox_moving_edge_set_coordinates(tracking_ellipse->medge, site->coords.u, site->coords.v, site->alpha, site->previous_convolution);
         ROX_ERROR_CHECK_TERMINATE ( error );

         // Perform tracking
         error = rox_moving_edge_track(tracking_ellipse->medge, image, 0);
         if (error)
         {
            site->state = 1;
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_ellipse->medge->_convolution;
         site->coords = tracking_ellipse->medge->_coords;
      }
      break;

      case RoxTrackingEllipseMethod_Search_Edge:
      {
         if (tracking_ellipse->search_edge == NULL)
         { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

         // Set the angle of the edge
         Rox_Double angle = 0.0;
         error = rox_ellipse2d_get_normal_angle ( &angle, &edge_ellipse->ellipse2d_pixels, site->coords.u, site->coords.v); // edge_ellipse->line_image_pixels.theta;
         ROX_ERROR_CHECK_TERMINATE ( error );

         tracking_ellipse->search_edge->_angle = angle;

         // Perform tracking
         error = rox_search_edge_track(tracking_ellipse->search_edge, image, &site->coords);
         if (error)
         {
            site->state = 1; // measure not found or invalid
            error = ROX_ERROR_NONE;
            continue;
         }

         // Store result
         site->previous_convolution = tracking_ellipse->search_edge->_convolution; // TODO not computed, remove struct member ???
         site->coords = tracking_ellipse->search_edge->_coords;
      }
      break;
      }
   }

   // ------------------------------------------------------------------------------------------------------- //
   if(edge_ellipse->sites->used > 5)
   {
      // Get the valid points
      Rox_DynVec_Point2D_Double dynvec_point2d = NULL;

      error = rox_dynvec_point2d_double_new(&dynvec_point2d, edge_ellipse->sites->used);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_edge_ellipse_get_valid_sites(dynvec_point2d, edge_ellipse);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Estimate ellipse from all valid measured points
      Rox_Ellipse2D_Parametric ellipse2d_parametric = NULL;
      error = rox_ellipse2d_new(&ellipse2d_parametric);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_ellipse2d_parametric_from_n_point2d(ellipse2d_parametric, dynvec_point2d);
      ROX_ERROR_CHECK_TERMINATE ( error );

      //error = rox_ellipse2d_parametric_separate_inliers_outliers(ellipse2d_parametric, dynvec_point2d);
      //ROX_ERROR_CHECK_TERMINATE ( error );

      const Rox_Double distance_threshold = 3.0;

      error = rox_edge_ellipse_separated_inliers_outliers(edge_ellipse, ellipse2d_parametric, distance_threshold);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point2d_double_del(&dynvec_point2d);
      ROX_ERROR_CHECK_TERMINATE ( error );

   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_tracking_ellipse_initialize (
   Rox_Tracking_Ellipse tracking_ellipse,
   Rox_Image image,
   Rox_Edge_Ellipse edge_ellipse
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!tracking_ellipse || !image || !edge_ellipse)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   switch (tracking_ellipse->method)
   {
   default:
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   break;

   case RoxTrackingEllipseMethod_Moving_Edge:
   {
      if (tracking_ellipse->medge == NULL)
      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      for ( Rox_Uint idsite = 0; idsite < edge_ellipse->sites->used; idsite++)
      {
         Rox_Edge_Ellipse_Site_Struct * site = &edge_ellipse->sites->data[idsite];

         // Set moving edges parameters
         error = rox_moving_edge_set_coordinates(tracking_ellipse->medge, site->coords.u, site->coords.v, site->alpha, 0.0);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }
   break;

   case RoxTrackingEllipseMethod_Search_Edge:
   {
      if (tracking_ellipse->search_edge == NULL)

      { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
      // Do nothing
   }
   break;
   }

function_terminate:
   return error;
}
