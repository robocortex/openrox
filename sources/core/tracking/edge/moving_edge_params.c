//==============================================================================
//
//    OPENROX   : File moving_edge_params.h
//
//    Contents  : Implementation of moving_edge_params module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "moving_edge_params.h"
#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>
#include <baseproc/geometry/line/line_clip.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode
rox_createMasks(Rox_Moving_Edge_Params obj);

Rox_ErrorCode rox_moving_edge_params_new(Rox_Moving_Edge_Params * obj, const Rox_Sint search_range, const Rox_Double contrast_threshold)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Moving_Edge_Params ret = NULL;

   if (!obj) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Moving_Edge_Params)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->_search_range = search_range;
   ret->_contrast_threshold = contrast_threshold;

   ret->_count_masks = 180;
   ret->_mask_size = 5;
   ret->_contrast_min = 1.0 - 0.5;
   ret->_contrast_max = 1.0 + 0.5;
   ret->_masks = NULL;

   error = rox_createMasks(ret);

   *obj = ret;
   error = ROX_ERROR_NONE;

function_terminate:
   // Delete only if an error occurred
   if (error) rox_moving_edge_params_del(&ret);
   return error;
}

Rox_ErrorCode rox_moving_edge_params_del(Rox_Moving_Edge_Params * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Moving_Edge_Params todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_collection_del(&todel->_masks);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode deleteMasks(Rox_Moving_Edge_Params obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_double_collection_del(&obj->_masks);
   obj->_masks = NULL;

function_terminate:
   return error;
}

Rox_ErrorCode rox_point2d_alignBox(Rox_Point2D_Double  pt, Rox_Rect_Real box)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!pt) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // If point is outside the box, clamp its values

   if (fabs(pt->u - box->x) < 1e-4) pt->u = box->x;
   if (fabs(pt->u - (box->x + box->width)) < 1e-4) pt->u = box->x + box->width;
   if (fabs(pt->v - box->y) < 1e-4) pt->v = box->y;
   if (fabs(pt->v - (box->y + box->height)) < 1e-4) pt->v = box->y + box->height;

function_terminate:
   return error;
}

Rox_ErrorCode rox_box_relativeareas (
   Rox_Double * relval,
   Rox_Point2D_Double P,
   Rox_Point2D_Double Q,
   Rox_Rect_Real rect)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double Xmin, Xmax, Ymin, Ymax;
   Rox_Point2D_Double_Struct swap;


   if (!relval) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if(Q->u < P->u)
   {
      swap = *Q;
      Q = P;
      *P = swap;
   }

   // For a given pixel rect, compute the relative area in p1

   Xmin = rect->x;
   Xmax = rect->x + rect->width;
   Ymin = rect->y;
   Ymax = rect->y + rect->height;

   rox_point2d_alignBox(P, rect);
   rox_point2d_alignBox(Q, rect);

   if (fabs(P->u-Xmin) < DBL_EPSILON && fabs(Q->u-Xmax) < DBL_EPSILON)
   {
      *relval = fabs(Ymax+Ymin-P->v-Q->v);
      error = ROX_ERROR_NONE; goto function_terminate;
   }

   if (((fabs(P->v-Ymin) < DBL_EPSILON) && (fabs(Q->v-Ymax) < DBL_EPSILON)) || ((fabs(Q->v-Ymin) < DBL_EPSILON) && (fabs(P->v-Ymax) < DBL_EPSILON)))
   {
      *relval = fabs(Xmax+Xmin-P->u-Q->u);
      error = ROX_ERROR_NONE; goto function_terminate;
   }

   if (fabs(P->u-Xmin) < DBL_EPSILON && fabs(Q->v-Ymax) < DBL_EPSILON)
   {
      *relval = 1.0-(Ymax-P->v)*(Q->u-Xmin);
      error = ROX_ERROR_NONE; goto function_terminate;
   }

   if (fabs(P->u-Xmin) < DBL_EPSILON && fabs(Q->v-Ymin) < DBL_EPSILON)
   {
      *relval = 1.0-(P->v-Ymin)*(Q->u-Xmin);
      error = ROX_ERROR_NONE; goto function_terminate;
   }

   if (fabs(P->v-Ymin) < DBL_EPSILON && fabs(Q->u-Xmax) < DBL_EPSILON)
   {
      *relval = 1.0-(Xmax-P->u)*(Q->v-Ymin);
      error = ROX_ERROR_NONE; goto function_terminate;
   }

   if (fabs(P->v-Ymax) < DBL_EPSILON && fabs(Q->u-Xmax) < DBL_EPSILON)
   {
      *relval = 1.0-(Xmax-P->u)*(Ymax-Q->v);
      error = ROX_ERROR_NONE; goto function_terminate;
   }

   error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;

function_terminate:
   return error;
}

Rox_ErrorCode rox_createMasks(Rox_Moving_Edge_Params obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Double theta, costh, sinth;
   Rox_Point2D_Double_Struct p, q;
   Rox_Array2D_Double curmask = NULL;
   Rox_Line2D_Homogeneous_Struct line;
   Rox_Double radius, x, y, sign, val;
   Rox_Double ** dm;
   Rox_Rect_Real_Struct rect;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   deleteMasks(obj);

   radius = ((double)obj->_mask_size) / 2.0;

   error = rox_array2d_double_collection_new(&obj->_masks, obj->_count_masks, obj->_mask_size, obj->_mask_size);
   if (error)
   {
      goto function_terminate;
   }

   for (Rox_Uint idmask = 0; idmask < obj->_count_masks; idmask++)
   {
      theta = idmask * (ROX_PI / obj->_count_masks);
      costh = cos(theta);
      sinth = sin(theta);

      line.a = sinth;
      line.b = -costh;
      line.c = 0;

      curmask = rox_array2d_double_collection_get(obj->_masks, idmask);
      if (!curmask)
      { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

      error = rox_array2d_double_get_data_pointer_to_pointer( &dm, curmask);

      y = - radius + 0.5;
      for (Rox_Uint i = 0; i < obj->_mask_size; i++)
      {
         x = - radius + 0.5;
         for (Rox_Uint j = 0; j < obj->_mask_size; j++)
         {
            sign = ROX_SIGN(costh * y - sinth * x);

            rect.x = x - .5;
            rect.y = y - .5;
            rect.width = 1;
            rect.height = 1;

            error = rox_line2d_clip_inside_rectangle(&p, &q, &line, &rect);
            if (!error)
            {
               error = rox_box_relativeareas(&val, &p, &q, &rect);
               ROX_ERROR_CHECK_TERMINATE ( error );
            }
            else
            {
               val = 1.0;
            }

            dm[i][j] = sign * val;

            x += 1.0;
         }

         y += 1.0;
      }
   }

   error = ROX_ERROR_NONE;

function_terminate:
   if (error) deleteMasks(obj);
   return error;
}
