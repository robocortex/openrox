//==============================================================================
//
//    OPENROX   : File edgedraw.c
//
//    Contents  : Implementation of edgedraw module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edgedraw.h"

#include <generated/dynvec_edgel_struct.h>
#include <generated/dynvec_edgeturn_struct.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/image/gradient/gradientsobel.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/image/convert/roxrgba_split.h>

#include <inout/image/pgm/pgmfile.h>
#include <inout/system/errors_print.h>
#include <inout/numeric/array2d_print.h>

int cmp_edgels(const Rox_Edgel_Struct * one, const Rox_Edgel_Struct * two)
{
   if (one->score > two->score) return -1;
   return 1;
}

Rox_ErrorCode rox_edgedraw_new(Rox_EdgeDraw *obj, Rox_Sint width, Rox_Sint height, Rox_Uint min_gradient, Rox_Uint anchorthresh)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgeDraw ret = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *obj = NULL;

   ret = (Rox_EdgeDraw)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->width = width;
   ret->height = height;
   ret->anchorthresh = anchorthresh;
   ret->min_gradient = min_gradient;

   ret->gnorm = NULL;
   ret->gori = NULL;
   ret->resultmask = NULL;
   ret->anchors = NULL;
   ret->stack = NULL;
   ret->resultsegments = NULL;

   error = rox_array2d_sshort_new(&ret->gnorm, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->gori, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_new(&ret->resultmask, height, width);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_edgel_new(&ret->anchors, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_edgeturn_new(&ret->stack, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_objset_dynvec_edgel_new(&ret->resultsegments, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_sshort_fillval(ret->gnorm, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_edgedraw_del(&ret);

   return error;
}

Rox_ErrorCode rox_edgedraw_del(Rox_EdgeDraw *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgeDraw todel = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_array2d_sshort_del(&todel->gnorm);
   rox_array2d_uchar_del(&todel->gori);
   rox_array2d_uchar_del(&todel->resultmask);
   rox_dynvec_edgel_del(&todel->anchors);
   rox_dynvec_edgeturn_del(&todel->stack);
   rox_objset_dynvec_edgel_del(&todel->resultsegments);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgedraw_gradientinfo(Rox_EdgeDraw obj, Rox_Array2D_Point2D_Sshort gradients)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if(!obj || !gradients) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   Rox_Point2D_Sshort_Struct ** dg = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer( &dg, gradients);

   Rox_Sshort ** dn = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer( &dn, obj->gnorm);
   
   Rox_Uchar ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&ds, obj->gori);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint i = 1; i < obj->height - 1; i++)
   {
      for ( Rox_Sint j = 1; j < obj->width - 1; j++)
      {
         Rox_Sshort gx = dg[i][j].u;
         Rox_Sshort gy = dg[i][j].v;

         Rox_Sshort norm = (Rox_Sshort)(sqrt((double)gx*gx+gy*gy));
         if (norm < obj->min_gradient) norm = 0;

         dn[i][j] = norm;

         if (abs(gx) > abs(gy)) ds[i][j] = 1;
         else ds[i][j] = 0;
      }
   }

   rox_dynvec_edgel_reset(obj->anchors);

   for ( Rox_Sint i = 2; i < obj->height - 2; i++)
   {
      for ( Rox_Sint j = 2; j < obj->width - 2; j++)
      {
         Rox_Sshort c = dn[i][j] ;
         Rox_Edgel_Struct pt;

         if (ds[i][j] == 0) // horizontal
         {
            if (c - dn[i - 1][j] >= obj->anchorthresh && c - dn[i + 1][j] >= obj->anchorthresh)
            {
               pt.u = j;
               pt.v = i;
               pt.score = dn[i][j];
               rox_dynvec_edgel_append(obj->anchors, &pt);
            }
         }
         else // vertical
         {
            if (c - dn[i][j - 1] >= obj->anchorthresh && c - dn[i][j + 1] >= obj->anchorthresh)
            {
               pt.u = j;
               pt.v = i;
               pt.score = dn[i][j];
               rox_dynvec_edgel_append(obj->anchors, &pt);
            }
         }
      }
   }

   // Sort anchor so that the anchor with higher gradient norm are processed first 
   qsort(obj->anchors->data, obj->anchors->used, sizeof(Rox_Edgel_Struct), (int (*)(const void *, const void *)) cmp_edgels);

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgedraw_buildedges(Rox_EdgeDraw obj, Rox_Uint min_segment_size, Rox_Bool straight_edge_only)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idanchor, countsub;
   Rox_Uchar ori;
   Rox_Edgel_Struct curpt, addedpt;
   Rox_EdgeTurn_Struct token;
   Rox_Sshort val1, val2, val3;
   Rox_DynVec_Edgel cursegment = NULL;

   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_fillval(obj->resultmask, 0);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar  ** dr = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&dr, obj->resultmask);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uchar  ** ds = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&ds, obj->gori);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sshort ** dn = NULL;
   error = rox_array2d_sshort_get_data_pointer_to_pointer ( &dn, obj->gnorm);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (idanchor = 0; idanchor < obj->anchors->used; idanchor++)
   {
      curpt = obj->anchors->data[idanchor];
      ori = ds[curpt.v][curpt.u];

      // If this anchor is already an edgel continue 
      if (dr[curpt.v][curpt.u])
      {
         continue;
      }

      rox_dynvec_edgeturn_reset(obj->stack);

      error = rox_dynvec_edgel_new(&cursegment, 10);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Add seeds to the stack depending on the peak direction 
      token.u = curpt.u;
      token.v = curpt.v;
      if (ori == 0)
      {
         // Add left seed
         token.direction = 0;
         rox_dynvec_edgeturn_append(obj->stack, &token);
         // Add right seed
         token.direction = 1;
         rox_dynvec_edgeturn_append(obj->stack, &token);
      }
      else
      {
         // Add top seed
         token.direction = 2;
         rox_dynvec_edgeturn_append(obj->stack, &token);
         // Add bottom seed
         token.direction = 3;
         rox_dynvec_edgeturn_append(obj->stack, &token);
      }

      // Build path using stack
      while (obj->stack->used > 0)
      {
         token = obj->stack->data[obj->stack->used - 1];
         obj->stack->used--;

         curpt.u = token.u;
         curpt.v = token.v;

         // *Add point to segment if not already added (anchor added once per direction)
         if (dr[curpt.v][curpt.u] == 0)
         {
            addedpt.u = curpt.u;
            addedpt.v = curpt.v;
            addedpt.score = dn[curpt.v][curpt.u];
            rox_dynvec_edgel_append(cursegment, &addedpt);
         }

         dr[curpt.v][curpt.u] = 255;
         ori = ds[curpt.v][curpt.u];

         // Continue on the same direction
         countsub = 0;
         while (1)
         {
            if (token.direction == 0) //left
            {
               val1 = dn[curpt.v - 1][curpt.u - 1];
               val2 = dn[curpt.v][curpt.u - 1];
               val3 = dn[curpt.v + 1][curpt.u - 1];

               if (val1 > val2 && val1 > val3)
               {
                  curpt.u = curpt.u - 1;
                  curpt.v = curpt.v - 1;
               }
               else if (val2 > val1 && val2 > val3)
               {
                  curpt.u = curpt.u - 1;
                  curpt.v = curpt.v;
               }
               else
               {
                  curpt.u = curpt.u - 1;
                  curpt.v = curpt.v + 1;
               }
            }
            else if (token.direction == 1) //right
            {
               val1 = dn[curpt.v - 1][curpt.u + 1];
               val2 = dn[curpt.v][curpt.u + 1];
               val3 = dn[curpt.v + 1][curpt.u + 1];

               if (val1 > val2 && val1 > val3)
               {
                  curpt.u = curpt.u + 1;
                  curpt.v = curpt.v - 1;
               }
               else if (val2 > val1 && val2 > val3)
               {
                  curpt.u = curpt.u + 1;
                  curpt.v = curpt.v;
               }
               else
               {
                  curpt.u = curpt.u + 1;
                  curpt.v = curpt.v + 1;
               }
            }
            else if (token.direction == 2) //top
            {
               val1 = dn[curpt.v - 1][curpt.u - 1];
               val2 = dn[curpt.v - 1][curpt.u];
               val3 = dn[curpt.v - 1][curpt.u + 1];

               if (val1 > val2 && val1 > val3)
               {
                  curpt.u = curpt.u - 1;
                  curpt.v = curpt.v - 1;
               }
               else if (val2 > val1 && val2 > val3)
               {
                  curpt.u = curpt.u;
                  curpt.v = curpt.v - 1;
               }
               else
               {
                  curpt.u = curpt.u + 1;
                  curpt.v = curpt.v - 1;
               }
            }
            else //bottom
            {
               val1 = dn[curpt.v + 1][curpt.u - 1];
               val2 = dn[curpt.v + 1][curpt.u];
               val3 = dn[curpt.v + 1][curpt.u + 1];

               if (val1 > val2 && val1 > val3)
               {
                  curpt.u = curpt.u - 1;
                  curpt.v = curpt.v + 1;
               }
               else if (val2 > val1 && val2 > val3)
               {
                  curpt.u = curpt.u;
                  curpt.v = curpt.v + 1;
               }
               else
               {
                  curpt.u = curpt.u + 1;
                  curpt.v = curpt.v + 1;
               }
            }

            if (dr[curpt.v][curpt.u] > 0) 
               break; //is this pixel already processed ?
            if (dn[curpt.v][curpt.u] == 0) 
               break; //is this pixel flat ?
            if (ds[curpt.v][curpt.u] != ori) 
               break; //Is there a change in major orientation ?

            dr[curpt.v][curpt.u] = 255;

            addedpt.u = curpt.u;
            addedpt.v = curpt.v;
            addedpt.score = dn[curpt.v][curpt.u];
            rox_dynvec_edgel_append(cursegment, &addedpt);

            countsub++;
         }

         // If after a walk, the next point change direction of peak and is not already an edgel stored ... 
         if (!straight_edge_only && (countsub > 0 && dn[curpt.v][curpt.u] > 0 && dr[curpt.v][curpt.u] == 0))
         {
            ori = ds[curpt.v][curpt.u];

            // Add seeds
            token.u = curpt.u;
            token.v = curpt.v;
            if (ori == 0)
            {
               token.direction = 0;
               rox_dynvec_edgeturn_append(obj->stack, &token);
               token.direction = 1;
               rox_dynvec_edgeturn_append(obj->stack, &token);
            }
            else
            {
               token.direction = 2;
               rox_dynvec_edgeturn_append(obj->stack, &token);
               token.direction = 3;
               rox_dynvec_edgeturn_append(obj->stack, &token);
            }
         }
      }

      // A segment is at least min_segment_size pixel long
      if (cursegment->used > min_segment_size)//1
      {
         rox_objset_dynvec_edgel_append(obj->resultsegments, cursegment);
      }
      else
      {
         rox_dynvec_edgel_del(&cursegment);
      }
      cursegment = NULL;
   }

   rox_dynvec_edgel_del(&cursegment);

function_terminate:
   // if(cursegment) rox_dynvec_edgel_del(&cursegment);
   return error;
}

Rox_ErrorCode rox_edgedraw_process(Rox_EdgeDraw obj, Rox_Array2D_Point2D_Sshort gradients, Rox_Uint min_segment_size, Rox_Bool straight_edge_only)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !gradients) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   rox_objset_dynvec_edgel_reset(obj->resultsegments);

   error = rox_array2d_point2d_sshort_check_size(gradients, obj->height, obj->width);
   ROX_ERROR_CHECK_TERMINATE(error) 

   error = rox_edgedraw_gradientinfo(obj, gradients);
   ROX_ERROR_CHECK_TERMINATE(error) 

   error = rox_edgedraw_buildedges(obj, min_segment_size, straight_edge_only);
   ROX_ERROR_CHECK_TERMINATE(error) 

function_terminate:
   return error;
}
