//==============================================================================
//
//    OPENROX   : File edgepostproc_segment.c
//
//    Contents  : Implementation of edgepostproc_segment module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edgepostproc_segment.h"

#include <baseproc/maths/maths_macros.h>

#include <generated/dynvec_edgel_struct.h>
#include <generated/objset_dynvec_edgel_struct.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>
#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/image/gradient/gradientsobel.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edgepostproc_segment_new(Rox_EdgePostproc_Segment *obj, Rox_Sint width, Rox_Sint height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgePostproc_Segment ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   *obj = NULL;

   ret = (Rox_EdgePostproc_Segment)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->width = width;
   ret->height = height;
   ret->resultsegments = NULL;

   CHECK_ERROR_TERMINATE(rox_dynvec_segment2d_new(&ret->resultsegments, 10));

   ret->minimal_length = (Rox_Uint) ceil(-2.0 * (log((double) width)+log((double) height)) / log(0.125));

   *obj = ret;

function_terminate:
   if (error) rox_edgepostproc_segment_del(&ret);

   return error;
}

Rox_ErrorCode rox_edgepostproc_segment_del(Rox_EdgePostproc_Segment *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgePostproc_Segment todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_dynvec_segment2d_del(&todel->resultsegments);

   rox_memory_delete(todel);

function_terminate:
    return error;
}

Rox_ErrorCode fitline_from_edgels(double * resdx, double *resdy, double *respx, double *respy, Rox_Edgel_Struct * data, Rox_Uint length)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double mX, mY, mXX, mXY, mYY, ci, cj;
   Rox_Double Ex,Ey, Exx, Exy, Eyy,Cxx,Cyy,Cxy,n, theta;

   if (!resdx || !resdy || !respx || !respy)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   mX = 0;
   mY = 0;
   mXX = 0;
   mXY = 0;
   mYY = 0;
   n = 0;

   for (Rox_Uint i = 0; i < length; i++)
   {
      cj = data[i].u;
      ci = data[i].v;

      mX  += (double)cj;
      mY  += (double)ci;
      mXX += (double)(cj*cj);
      mXY += (double)(ci*cj);
      mYY += (double)(ci*ci);
   }

   n = length;

   Ex = mX / n;
   Ey = mY / n;
   Exx = mXX / n;
   Exy = mXY / n;
   Eyy = mYY / n;
   Cxx = Exx - (Ex * Ex);
   Cyy = Eyy - (Ey * Ey);
   Cxy = Exy - (Ex * Ey);

   theta = atan2(2.0 * Cxy, Cxx - Cyy) * 0.5;
   *resdx = cos(theta);
   *resdy = sin(theta);
   *respx = Ex;
   *respy = Ey;

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgepostproc_validate(Rox_EdgePostproc_Segment obj, Rox_DynVec_Edgel segment)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Uint pos_start, pos_end, size;
   double prevx, prevy;
   double dx,dy,px,py,cx,cy;
   double nx, ny, d;
   double dist, max;

   Rox_Segment2D_Struct toadd;

   if (segment->used < obj->minimal_length)
   { error =  ROX_ERROR_NONE; goto function_terminate; }

   pos_start = 0;
   while (pos_start + obj->minimal_length <= segment->used)
   {
      // Fit a line to minimal_length set of points
      error = fitline_from_edgels(&dx, &dy, &px, &py, segment->data + pos_start, obj->minimal_length);
      ROX_ERROR_CHECK_TERMINATE ( error );

      // Evaluate distance
      nx = dy;
      ny = -dx;
      max = 0;

      prevx = segment->data[pos_start].u;
      prevy = segment->data[pos_start].v;
      for (Rox_Uint i = 0; i < obj->minimal_length; i++)
      {
         cx = segment->data[pos_start + i].u;
         cy = segment->data[pos_start + i].v;
         if (fabs(cx - prevx) > 2.0 || fabs(cy - prevy) > 2.0)
         {
            max = 100.0;
         }
         prevx = cx;
         prevy = cy;

         cx -= px;
         cy -= py;
         dist = fabs(nx * cx + ny * cy);
         if (dist > max)
         {
           max = dist;
         }
      }

      // Go to Next seeding pixel if it is not a valid line
      if (max > 1.0)
      {
         pos_start++;
         continue;
      }

      // Try to extend line
      size = obj->minimal_length;
      pos_end = pos_start + size;
      prevx = segment->data[pos_end].u;
      prevy = segment->data[pos_end].v;
      while (pos_end < segment->used)
      {
         cx = segment->data[pos_end].u;
         cy = segment->data[pos_end].v;
         if (fabs(cx - prevx) > 2.0 || fabs(cy - prevy) > 2.0) break;
         prevx = cx;
         prevy = cy;

         cx -= px;
         cy -= py;
         dist = fabs(nx * cx + ny * cy);
         if (dist > 1.0) break;
         size++;
         pos_end++;
      }

      error = fitline_from_edgels(&dx, &dy, &px, &py, segment->data + pos_start, size);
      ROX_ERROR_CHECK_TERMINATE ( error );

      nx = dy;
      ny = -dx;

      cx = segment->data[pos_start].u;
      cy = segment->data[pos_start].v;
      d = (- nx * cx + nx * px - ny * cy + ny * py) / (nx*nx + ny*ny);
      toadd.points[0].u = cx + d * nx;
      toadd.points[0].v = cy + d * ny;

      cx = segment->data[pos_start + size - 1].u;
      cy = segment->data[pos_start + size - 1].v;

      d = (- nx * cx + nx * px - ny * cy + ny * py) / (nx*nx + ny*ny);
      toadd.points[1].u = cx + d * nx;
      toadd.points[1].v = cy + d * ny;

      error = rox_dynvec_segment2d_append(obj->resultsegments, &toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );

      pos_start += size;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgepostproc_compute_robust(Rox_EdgePostproc_Segment obj, Rox_DynVec_Edgel segment)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !segment) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (segment->used < obj->minimal_length) return ROX_ERROR_NONE;

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgepostproc_segment_process(Rox_EdgePostproc_Segment obj, Rox_ObjSet_DynVec_Edgel segments)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_segment2d_reset(obj->resultsegments);

   for (Rox_Uint idsegment = 0; idsegment < segments->used; idsegment++)
   {
      Rox_DynVec_Edgel segment = segments->data[idsegment];

      error = rox_edgepostproc_validate(obj, segment);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgepostproc_segment_process_robust(Rox_EdgePostproc_Segment obj, Rox_ObjSet_DynVec_Edgel segments)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj || !segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_segment2d_reset(obj->resultsegments);

   for (Rox_Uint idsegment = 0; idsegment < segments->used; idsegment++)
   {
      Rox_DynVec_Edgel segment = segments->data[idsegment];
      error = rox_edgepostproc_compute_robust(obj, segment);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}
