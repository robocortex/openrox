//==============================================================================
//
//    OPENROX   : File tlid.c
//
//    Contents  : Implementation of tlid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tlid.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <generated/dynvec_segment2d_struct.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/maths_macros.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_tlid_new(Rox_Tlid *obj, Rox_Sint width, Rox_Sint height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tlid ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_Tlid) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->segments = NULL;
   ret->width = width;
   ret->height = height;

   error = rox_dynvec_tlid_segment_new(&ret->segments, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_tlid_del(&ret);

   return error;
}

Rox_ErrorCode rox_tlid_del(Rox_Tlid *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tlid todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tlid_build_descriptor(Rox_Tlid obj, Rox_Uint idcur)
{
   Rox_Sint knn[30];
   Rox_Double knn_dist[30];
   Rox_Double desc[12*12];
   Rox_TLID_Segment_Struct * cursegment = NULL;
   Rox_TLID_Segment_Struct * refsegment = NULL;
   Rox_Double du, dv, dist, buf, fbuf;
   Rox_Point2D_Double_Struct tij, tji;
   Rox_Double ntij, ntji, alpha, beta, dalpha, dbeta, mdalpha, mdbeta, norm, inorm;
   Rox_Sint ialpha, ibeta, nalpha, nbeta, startcopy;

   cursegment = &obj->segments->data[idcur];

   // Reset Knn
   for (Rox_Sint idnn = 0; idnn < 20; idnn++)
   {
      knn[idnn] = -1;
      knn_dist[idnn] = DBL_MAX;
   }

   // Init descriptors values
   for (Rox_Sint i = 0; i < 12*12*4; i++)
   {
      cursegment->desc[i] = 0;
   }

   for (Rox_Sint i = 0; i < 12*12; i++)
   {
      desc[i] = 0;
   }

   // Find KNN
   for (Rox_Uint idseg = 0; idseg < obj->segments->used; idseg++)
   {
      if (idseg == idcur) continue;

      refsegment = &obj->segments->data[idseg];

      du = cursegment->midpoint.u - refsegment->midpoint.u;
      dv = cursegment->midpoint.v - refsegment->midpoint.v;
      dist = du*du + dv*dv;

      for (Rox_Sint idnn = 0; idnn < 20; idnn++)
      {
         if (dist < knn_dist[idnn])
         {
            for (Rox_Sint iddec = 19; iddec > idnn; iddec--)
            {
               knn_dist[iddec] = knn_dist[iddec - 1];
               knn[iddec] = knn[iddec - 1];
            }

            knn_dist[idnn] = dist;
            knn[idnn] = idseg;
            break;
         }
      }
   }

   // i is cur aand j ref
   for (Rox_Sint idnn = 0; idnn < 20; idnn++)
   {
      Rox_Sint idseg = knn[idnn];
      if (idseg < 0) continue;
      refsegment = &obj->segments->data[idseg];

      // Compute segment between midpoint
      tij.u = refsegment->midpoint.u - cursegment->midpoint.u;
      tij.v = refsegment->midpoint.v - cursegment->midpoint.v;
      tji.u = cursegment->midpoint.u - refsegment->midpoint.u;
      tji.v = cursegment->midpoint.v - refsegment->midpoint.v;

      // Compute norms
      ntij = sqrt(tij.u * tij.u + tij.v * tij.v);
      ntji = sqrt(tji.u * tji.u + tji.v * tji.v);

      // Compute angles between segment and linkinkg (tij/tji) segment
      buf = (cursegment->direction.u * tij.u + cursegment->direction.v * tij.v) / ntij;
      if (buf < -1.0) buf = -1.0;
      if (buf > 1.0) buf = 1.0;
      alpha = acos(buf);

      buf = (refsegment->direction.u * tji.u + refsegment->direction.v * tji.v) / ntji;
      if (buf < -1.0) buf = -1.0;
      if (buf > 1.0) buf = 1.0;
      beta = acos(buf);

      // Disambiguate alpha
      buf = cursegment->direction.u* tij.v - cursegment->direction.v * tij.u;
      fbuf = fabs(buf);
      buf = buf / fabs(buf);
      if (buf < 0.0)
      {
         alpha = 2.0 * ROX_PI - alpha;
      }

      // Disambiguate beta
      buf = refsegment->direction.u * tji.v - refsegment->direction.v * tji.u;
      fbuf = fabs(buf);
      buf = buf / fbuf;
      if (buf < 0.0)
      {
         beta = 2.0 * ROX_PI - beta;
      }

      // Compute bin
      alpha = (alpha / (2.0 * ROX_PI)) * 12.0;
      beta = (beta / (2.0 * ROX_PI)) * 12.0;

      // Bilinear interpolation
      ialpha = (int)alpha;
      ibeta = (int)beta;
      dalpha = alpha - (double)ialpha;
      dbeta = beta - (double)ibeta;
      mdalpha = 1.0 - dalpha;
      mdbeta = 1.0 - dbeta;
      nalpha = ialpha + 1;
      nbeta = ibeta + 1;
      if (nalpha >= 12) nalpha = 0;
      if (nbeta >= 12) nbeta = 0;

      desc[ialpha * 12 + ibeta] += mdalpha * mdbeta;
      desc[ialpha * 12 + ibeta + 1] += mdalpha * dbeta;
      desc[(ialpha + 1) * 12 + ibeta] += dalpha * mdbeta;
      desc[(ialpha + 1) * 12 + ibeta + 1] += dalpha * dbeta;

      if (idnn == 4) startcopy = 0;
      else if (idnn == 9) startcopy = 12*12;
      else if (idnn == 14) startcopy = 12*12*2;
      else if (idnn == 19) startcopy = 12*12*3;
      else startcopy = -1;

      if (startcopy >= 0)
      {
         norm = 0;

         for ( Rox_Sint i = 0; i < 12*12; i++)
         {
            buf = desc[i];
            norm += buf*buf;
         }

         // Normalize and copy local bunch descriptor
         norm = 1.0 / sqrt(norm);
         for ( Rox_Sint i = 0; i < 12*12; i++)
         {
            cursegment->desc[startcopy + i] = desc[i] * norm;
         }
      }
   }

   // Compute L2 norm of histogram
   norm = 0;
   for (Rox_Sint i = 0; i < 12*12*4; i++)
   {
      buf = cursegment->desc[i];
      norm += buf * buf;
   }

   // Normalize histogram
   inorm = 1.0 / sqrt(norm);
   for (Rox_Sint i = 0; i < 12*12*4; i++)
   {
      cursegment->desc[i] *= inorm;
   }

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_tlid_process(Rox_Tlid obj, Rox_DynVec_Segment2D segments, Rox_Array2D_Point2D_Sshort gradients)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_TLID_Segment_Struct toadd;
   Rox_Point2D_Sshort_Struct p1, p2, p3, p4;
   Rox_Point2D_Double_Struct pv1, pv2, dir;
   Rox_Sint u,v;
   Rox_Double dx,dy,mdx,mdy,norm,gu,gv,sign;

   if (!obj || !segments)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Point2D_Sshort_Struct ** dg = NULL;
   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer(&dg, gradients);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for (Rox_Uint idseg = 0; idseg < segments->used; idseg++)
   {
      toadd.points[0] = segments->data[idseg].points[0];
      toadd.points[1] = segments->data[idseg].points[1];

      // Compute direction vector
      dir.u = toadd.points[1].u - toadd.points[0].u;
      dir.v = toadd.points[1].v - toadd.points[0].v;

      // Compute midpoint
      toadd.midpoint.u = toadd.points[0].u + 0.5 * dir.u;
      toadd.midpoint.v = toadd.points[0].v + 0.5 * dir.v;

      // Retrieve gradients around point
      u = (int)toadd.midpoint.u;
      v = (int)toadd.midpoint.v;
      if (u < 0 || v < 0 || u >= obj->width - 1 || v >= obj->height - 1)
      {
         continue;
      }

      // Bilinear interpolation of gradient
      dx = toadd.midpoint.u - (double)u;
      dy = toadd.midpoint.v - (double)v;
      mdx = 1.0 - dx;
      mdy = 1.0 - dy;
      p1 = dg[v][u];
      p2 = dg[v][u + 1];
      p3 = dg[v + 1][u];
      p4 = dg[v + 1][u + 1];
      pv1.u = mdx * p1.u + dx * p2.u;
      pv1.v = mdx * p1.v + dx * p2.v;
      pv2.u = mdx * p3.u + dx * p4.u;
      pv2.v = mdx * p3.v + dx * p4.v;
      gu = mdy * pv1.u + dy * pv2.u;
      gv = mdy * pv1.v + dy * pv2.v;

      // Normalize gradient
      norm = sqrt(gu*gu+gv*gv);
      gu /= norm;
      gv /= norm;

      // Compute disambiguated normalized direction vector for segment
      sign = dir.u * gv - dir.v * gu;
      sign = sign / fabs(sign);
      norm = sqrt(dir.u * dir.u + dir.v * dir.v);
      toadd.direction.u = sign * dir.u / norm;
      toadd.direction.v = sign * dir.v / norm;

      error = rox_dynvec_tlid_segment_append(obj->segments, &toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   for (Rox_Uint idseg = 0; idseg < segments->used; idseg++)
   {
      error = rox_tlid_build_descriptor(obj, idseg);
      if (error) continue;
   }

function_terminate:
   return error;
}

