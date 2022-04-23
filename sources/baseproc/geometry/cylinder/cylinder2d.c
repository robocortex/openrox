//==============================================================================
//
//    OPENROX   : File cylinder2d.c
//
//    Contents  : Implementation Structures of cylinder2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "cylinder2d.h"

#include <math.h>

#include <generated/dynvec_point2d_double_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/geometry/cylinder/cylinder2d_struct.h>
#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/segment/segment2d.h>

#include <inout/system/errors_print.h>
#include <inout/system/print.h>

Rox_ErrorCode rox_cylinder2d_new(Rox_Cylinder2D * cylinder2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Cylinder2D ret = NULL;

   if (!cylinder2d) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *cylinder2d = NULL;

   ret = (Rox_Cylinder2D) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_segment2d_new(&ret->s1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_segment2d_new(&ret->s2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ellipse2d_new(&ret->e1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ellipse2d_new(&ret->e2);
   ROX_ERROR_CHECK_TERMINATE ( error );

   *cylinder2d = ret;

function_terminate:
   if(error) rox_cylinder2d_del(&ret);
   return error;
}

Rox_ErrorCode rox_cylinder2d_del(Rox_Cylinder2D * cylinder2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Cylinder2D todel = NULL;

   if (!cylinder2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *cylinder2d;
   *cylinder2d = NULL;

   if (!todel)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_segment2d_del(&todel->s1);
   rox_segment2d_del(&todel->s2);
   rox_ellipse2d_del(&todel->e1);
   rox_ellipse2d_del(&todel->e2);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder2d_print(Rox_Cylinder2D cylinder2d)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_segment2d_print(cylinder2d->s1);
   rox_segment2d_print(cylinder2d->s2);
   rox_ellipse2d_print(cylinder2d->e1);
   rox_ellipse2d_print(cylinder2d->e2);

   // TODO

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder2d_get_normal_angle(Rox_Double * angle, Rox_Cylinder2D cylinder2d, const Rox_Double xe, const Rox_Double ye)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder2d || !angle)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   // TO BE IMPLEMENTED 

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder2d_get_tangent_angle(Rox_Double * angle, Rox_Cylinder2D cylinder2d, const Rox_Double xe, const Rox_Double ye)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!cylinder2d || !angle)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // TO BE IMPLEMENTED 

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder2d_sample(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Cylinder2D cylinder2d, Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dynvec_point2d || !cylinder2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_double_reset(dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sample points on segments
   Rox_DynVec_Point2D_Double dynvec_point2d_segment1 = NULL;
   error = rox_dynvec_point2d_double_new(&dynvec_point2d_segment1, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_segment2d_sample(dynvec_point2d_segment1, cylinder2d->s1, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_DynVec_Point2D_Double dynvec_point2d_segment2 = NULL;
   error = rox_dynvec_point2d_double_new(&dynvec_point2d_segment2, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_segment2d_sample(dynvec_point2d_segment2, cylinder2d->s2, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sample points on ellipses
   // TODO

   // Concatenate points

   for (Rox_Uint k = 0; k < dynvec_point2d_segment1->used; k++)
   {
      Rox_Point2D_Double_Struct point2d = dynvec_point2d_segment1->data[k];
      rox_dynvec_point2d_double_append(dynvec_point2d, &point2d);
   }
   for (Rox_Uint k = 0; k < dynvec_point2d_segment2->used; k++)
   {
      Rox_Point2D_Double_Struct point2d = dynvec_point2d_segment2->data[k];
      rox_dynvec_point2d_double_append(dynvec_point2d, &point2d);
   }

function_terminate:
   rox_dynvec_point2d_double_del(&dynvec_point2d_segment1);
   rox_dynvec_point2d_double_del(&dynvec_point2d_segment2);
   return error;
}

Rox_ErrorCode rox_cylinder2d_sample_segment2d_1(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Cylinder2D cylinder2d, Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dynvec_point2d || !cylinder2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_double_reset(dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sample points on segment 1
   error = rox_segment2d_sample(dynvec_point2d, cylinder2d->s1, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder2d_sample_segment2d_2(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Cylinder2D cylinder2d, Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dynvec_point2d || !cylinder2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_double_reset(dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sample points on segment 1
   error = rox_segment2d_sample(dynvec_point2d, cylinder2d->s2, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder2d_sample_ellipse2d_1(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Cylinder2D cylinder2d, Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dynvec_point2d || !cylinder2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_double_reset(dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sample points on segment 1
   error = rox_ellipse2d_sample(dynvec_point2d, cylinder2d->e1, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_cylinder2d_sample_ellipse2d_2(Rox_DynVec_Point2D_Double dynvec_point2d, Rox_Cylinder2D cylinder2d, Rox_Double sampling_step)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!dynvec_point2d || !cylinder2d)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_point2d_double_reset(dynvec_point2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Sample points on segment 1
   error = rox_ellipse2d_sample(dynvec_point2d, cylinder2d->e2, sampling_step);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
