//==============================================================================
//
//    OPENROX   : File moving_edge.c
//
//    Contents  : Implementation of moving_edge module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "moving_edge.h"

#include <generated/dynvec_double_struct.h>
#include <generated/dynvec_point2d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/image/image.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_moving_edge_new(Rox_Moving_Edge * obj, Rox_Moving_Edge_Params params)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Moving_Edge ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!params) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Moving_Edge)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->_params = params;
   ret->_convolution = 0;
   ret->_mask_sign = 1;
   ret->_sites = NULL;
   ret->_likelihoods = NULL;

   error = rox_dynvec_point2d_double_new(&ret->_sites, params->_search_range * 2 + 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_double_new(&ret->_likelihoods, params->_search_range * 2 + 2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_moving_edge_set_coordinates(ret, 0, 0, 0, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_moving_edge_del(&ret);
   return error;
}

Rox_ErrorCode rox_moving_edge_del(Rox_Moving_Edge * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Moving_Edge todel = NULL;

   if (!obj) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_double_del(&todel->_likelihoods);
   rox_dynvec_point2d_double_del(&todel->_sites);

   rox_memory_delete(todel);

function_terminate:
    return error;
}

Rox_ErrorCode rox_moving_edge_set_coordinates(Rox_Moving_Edge obj, Rox_Double u, Rox_Double v, Rox_Double alpha, Rox_Double prevconvolution)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double costh, sinth;
   Rox_Point2D_Double_Struct pt;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->_angle = alpha;
   obj->_coords.u = u;
   obj->_coords.v = v;
   obj->_convolution = prevconvolution;

   // Compute the coordinates of sampled points
   costh = cos(alpha);
   sinth = sin(alpha);
   rox_dynvec_point2d_double_reset(obj->_sites);
   for (Rox_Sint k = -obj->_params->_search_range ; k <= obj->_params->_search_range ; k++)
   {
      pt.v = v + k * sinth;
      pt.u = u + k * costh;

      rox_dynvec_point2d_double_append(obj->_sites, &pt);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_moving_edge_track(Rox_Moving_Edge obj, Rox_Image image, Rox_Uint check_contrast)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint idmask, idsite;
   Rox_Double theta, convolution, contraste, diff;
   Rox_Array2D_Double mask;
   Rox_Double ** dm;
   Rox_Uchar ** di;
   Rox_Sint width, height;
   Rox_Point2D_Double_Struct pt;
   Rox_Sint halfsize, size;
   Rox_Sint i, j, hi, hj;
   Rox_Double max, maxconv;
   Rox_Sint maxrank;


   if (!image || !obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   size = obj->_params->_mask_size;
   halfsize = (size - 1) >> 1;

   // Compute tangent
   theta  = obj->_angle+ROX_PI/2;
   while (theta<0) theta += ROX_PI;
   while (theta>ROX_PI) theta -= ROX_PI;

   // Retrieve the convolution mask
   idmask = (Rox_Sint) ((theta / ROX_PI) * obj->_params->_count_masks + 0.5);
   if (idmask < 0) idmask = 0;
   if (idmask >= (Rox_Sint) obj->_params->_count_masks) idmask = (Rox_Sint) (obj->_params->_count_masks - 1);

   mask = rox_array2d_double_collection_get(obj->_params->_masks, idmask);


   error = rox_array2d_uchar_get_size(&height, &width, image); 
   ROX_ERROR_CHECK_TERMINATE ( error ); 

   error = rox_array2d_uchar_get_data_pointer_to_pointer( &di, image);
   error = rox_array2d_double_get_data_pointer_to_pointer( &dm, mask);

   diff = 1e6;
   maxrank = -1;
   max = 0;
   maxconv = 0;

   // For each possible location
   for (idsite = 0; idsite < (Rox_Sint) obj->_sites->used; idsite++)
   {
      pt = obj->_sites->data[idsite];

      hi = (Rox_Sint) (pt.v - halfsize);
      hj = (Rox_Sint) (pt.u - halfsize);

      // Check if position is valid
      if (hi < 0) continue;
      if (hj < 0) continue;
      if (hi + size >= height) continue;
      if (hj + size >= width) continue;

      // Convolve with mask
      convolution = 0;
      for (i = 0; i < size; i++)
      {
         for (j = 0; j < size; j++)
         {
            convolution += obj->_mask_sign * dm[i][j] * di[hi + i][hj + j];
         }
      }

      // Compute score
      if (check_contrast)
      {
         obj->_likelihoods->data[idsite] = fabs(obj->_convolution + convolution);

         if (obj->_likelihoods->data[idsite] > obj->_params->_contrast_threshold)
         {
            contraste = convolution / obj->_convolution;

            if (contraste > obj->_params->_contrast_min && contraste < obj->_params->_contrast_max && fabs(1.0 - contraste) < diff)
            {
               diff = fabs(1.0 - contraste);
               maxconv = convolution;
               max = obj->_likelihoods->data[idsite];
               maxrank = idsite;
            }
         }
      }
      else
      {
         obj->_likelihoods->data[idsite] = fabs(2 * convolution);
         if (obj->_likelihoods->data[idsite] > max && obj->_likelihoods->data[idsite] > obj->_params->_contrast_threshold)
         {
            maxconv = convolution;
            max = obj->_likelihoods->data[idsite];
            maxrank = idsite;
         }
      }
   }

   if (maxrank < 0)
   {
      obj->_norm_gradient = 0;
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   obj->_norm_gradient = maxconv * maxconv;
   obj->_convolution = maxconv;
   obj->_coords = obj->_sites->data[maxrank];

function_terminate:
   return error;
}
