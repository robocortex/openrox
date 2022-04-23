//==============================================================================
//
//    OPENROX   : File segment2d.c
//
//    Contents  : Implementation Structure of segment2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "segment2d.h"

#include <math.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/line/line2d_struct.h>
#include <baseproc/geometry/segment/segment2d_struct.h>
#include <inout/system/errors_print.h>
#include <inout/system/print.h>
#include <system/errors/errors.h>

Rox_ErrorCode rox_segment2d_new(Rox_Segment2D * segment2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Segment2D ret = NULL;

   if (!segment2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *segment2d = NULL;

   ret = (Rox_Segment2D) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->points[0].u = 1.0;
   ret->points[0].v = 2.0;
   ret->points[1].u = 3.0;
   ret->points[1].v = 4.0;

   *segment2d = ret;

function_terminate:
   if(error) rox_segment2d_del(&ret);
   return error;
}

Rox_ErrorCode rox_segment2d_del(Rox_Segment2D * segment2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Segment2D todel = NULL;

   if (!segment2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *segment2d;
   *segment2d = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment2d_set(Rox_Segment2D segment2d, Rox_Double u1, Rox_Double v1, Rox_Double u2, Rox_Double v2)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!segment2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   segment2d->points[0].u = u1;
   segment2d->points[0].v = v1;

   segment2d->points[1].u = u2;
   segment2d->points[1].v = v2;

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment2d_get_line2d_normal(Rox_Line2D_Normal line2d, Rox_Segment2D segment2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Double u1 = segment2d->points[0].u;
   Rox_Double v1 = segment2d->points[0].v;

   Rox_Double u2 = segment2d->points[1].u;
   Rox_Double v2 = segment2d->points[1].v;

   if (!segment2d || !line2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double d = sqrt((u2-u1)*(u2-u1)+(v2-v1)*(v2-v1));
   // cos(theta)
   Rox_Double ct = (u2-u1)/d;
   // sin(theta)
   Rox_Double st = (v2-v1)/d;

   // tangent vector = [ct; +st; 0.0]
   // normal  vector = [st; -ct; 0.0]

   // line2d_homogeneous = st*(u-u1)-ct*(v-v1) = 0 
   // line2d_homogeneous = st*u-ct*v-(st*u1-ct*v1) = 0
   line2d->theta = atan2(-ct,st);
   line2d->rho = (st*u1-ct*v1);

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment2d_print(Rox_Segment2D segment2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!segment2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_log("segment2d : \n");
   rox_log("point1 = (%f, %f) \n", segment2d->points[0].u, segment2d->points[0].v);
   rox_log("point2 = (%f, %f) \n", segment2d->points[1].u, segment2d->points[1].v);

function_terminate:
   return error;
}

Rox_ErrorCode rox_segment2d_sample (
   Rox_DynVec_Point2D_Double dynvec_point2d, 
   Rox_Segment2D segment2d, 
   Rox_Double sampling_step
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct point2d;

   if (!dynvec_point2d || !segment2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_double_reset(dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Compute line length in pixels
   Rox_Double dx = segment2d->points[1].u - segment2d->points[0].u;
   Rox_Double dy = segment2d->points[1].v - segment2d->points[0].v;
   Rox_Double length = sqrt(dx*dx + dy*dy);

   // 1 should be segment_min_size ???
   if (length < 1) return ROX_ERROR_NONE;

   // How many samples are expected
   Rox_Double expected_density = (double) length / sampling_step;

   // floor
   Rox_Sint nbsamples = (int) expected_density;

   Rox_Double residual = length - nbsamples * sampling_step;
   Rox_Double sx = dx / length;
   Rox_Double sy = dy / length;

   // Create samples    
   if (length < sampling_step)
   {
      if(0)
      {
      // In this case take the two extremal points
      point2d.u = segment2d->points[0].u;
      point2d.v = segment2d->points[0].v;

      // Add the point to the dynvec
      error = rox_dynvec_point2d_double_append(dynvec_point2d, &point2d);
      ROX_ERROR_CHECK_TERMINATE ( error );

      point2d.u = segment2d->points[1].u;
      point2d.v = segment2d->points[1].v;

      // Add the point to the dynvec
      error = rox_dynvec_point2d_double_append(dynvec_point2d, &point2d);
      ROX_ERROR_CHECK_TERMINATE ( error );
      }
      else
      {
         // In this case take the middle point
         point2d.u = 0.5*(segment2d->points[0].u+segment2d->points[1].u);
         point2d.v = 0.5*(segment2d->points[0].v+segment2d->points[1].v);

         // Add the point to the dynvec
         error = rox_dynvec_point2d_double_append(dynvec_point2d, &point2d);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }
   else
   {
      // Initial point
      Rox_Double pi_u = segment2d->points[0].u;
      Rox_Double pi_v = segment2d->points[0].v;

      for (Rox_Sint i = 0; i <= nbsamples; i++)
      {
         point2d.u = pi_u + (0.5*residual + i * sampling_step) * sx;
         point2d.v = pi_v + (0.5*residual + i * sampling_step) * sy;

         // Add the point to the dynvec
         error = rox_dynvec_point2d_double_append(dynvec_point2d, &point2d);
         ROX_ERROR_CHECK_TERMINATE ( error );
      }
   }

function_terminate:
   return error;
}
