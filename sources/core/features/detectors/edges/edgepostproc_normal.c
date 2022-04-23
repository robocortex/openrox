//==============================================================================
//
//    OPENROX   : File edgepostproc_normal.h
//
//    Contents  : Implementation of edgepostproc_normal module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edgepostproc_normal.h"

#include <baseproc/maths/maths_macros.h>

#include <generated/dynvec_edgel_struct.h> 
#include <generated/objset_dynvec_edgel_struct.h> 

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/image/gradient/gradientsobel.h>
#include <baseproc/maths/maths_macros.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edgepostproc_normal_new(Rox_EdgePostproc_Normal *obj, Rox_Sint width, Rox_Sint height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgePostproc_Normal ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   *obj = NULL;

   ret = (Rox_EdgePostproc_Normal)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->width = width;
   ret->height = height;
   ret->resultsegments = NULL;

   CHECK_ERROR_TERMINATE(rox_objset_dynvec_edgel_new(&ret->resultsegments, 10));

   *obj = ret;

function_terminate:
   if (error) rox_edgepostproc_normal_del(&ret);

   return error;
}

Rox_ErrorCode rox_edgepostproc_normal_del(Rox_EdgePostproc_Normal *obj)
{
    Rox_ErrorCode error = ROX_ERROR_NONE;
  Rox_EdgePostproc_Normal todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_objset_dynvec_edgel_del(&todel->resultsegments);

   rox_memory_delete(todel);

function_terminate:
    return error;
}

Rox_ErrorCode rox_edgepostproc_normal_process(Rox_EdgePostproc_Normal obj, Rox_ObjSet_DynVec_Edgel segments, Rox_Array2D_Point2D_Sshort gradients)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Sshort_Struct ** dg ;
   Rox_Uint idsegment, idpt;
   Rox_DynVec_Edgel iseg = NULL, oseg = NULL;
   Rox_Uint u,v;
   Rox_Point2D_Sshort_Struct g;
   double gx,gy, angle;
   int first;
   Rox_Double refangle, dist;

   if (!obj || !segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_point2d_sshort_get_data_pointer_to_pointer( &dg, gradients);
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_objset_dynvec_edgel_reset(obj->resultsegments);

   for (idsegment = 0; idsegment < segments->used; idsegment++)
   {
      iseg = segments->data[idsegment];

      first = 1;
      oseg = NULL;
      refangle = 0;

      for (idpt = 0; idpt < iseg->used; idpt++)
      {
         u = iseg->data[idpt].u;
         v = iseg->data[idpt].v;
         g = dg[v][u];

         gx = (double)g.u;
         gy = (double)g.v;

         angle = atan2(gy, gx);
         dist = fabs(atan2(sin(refangle-angle), cos(refangle-angle)));

         if (first || dist > ROX_PI_4)
         {
            error = rox_dynvec_edgel_new(&oseg, 10);                           ROX_ERROR_CHECK_TERMINATE(error)
            error = rox_objset_dynvec_edgel_append(obj->resultsegments, oseg); ROX_ERROR_CHECK_TERMINATE(error)
            
            first = 0;

            refangle = angle;
         }

         rox_dynvec_edgel_append(oseg, &iseg->data[idpt]);
      }

      oseg = NULL;
   }
   
function_terminate:
   if ( NULL != oseg ) rox_dynvec_edgel_del(&oseg);

   return error;
}
