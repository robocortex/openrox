//==============================================================================
//
//    OPENROX   : File edgepostproc_ac.c
//
//    Contents  : Implementation of edgepostproc_ac module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "edgepostproc_ac.h"

#include <baseproc/maths/maths_macros.h>

#include <generated/dynvec_edgel_struct.h>
#include <generated/objset_dynvec_edgel_struct.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/kernels/gaussian2d.h>
#include <baseproc/image/gradient/gradientsobel.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_edgepostproc_ac_new(Rox_EdgePostproc_Ac *obj, Rox_Sint width, Rox_Sint height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgePostproc_Ac ret = NULL;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   *obj = NULL;

   ret = (Rox_EdgePostproc_Ac)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->width = width;
   ret->height = height;
   ret->validation_stack = NULL;
   ret->resultsegments = NULL;

   CHECK_ERROR_TERMINATE(rox_dynvec_segment_part_new(&ret->validation_stack, 10));
   CHECK_ERROR_TERMINATE(rox_objset_dynvec_edgel_new(&ret->resultsegments, 10));

   *obj = ret;

function_terminate:
   if (error) rox_edgepostproc_ac_del(&ret);

   return error;
}

Rox_ErrorCode rox_edgepostproc_ac_del(Rox_EdgePostproc_Ac *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_EdgePostproc_Ac todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_segment_part_del(&todel->validation_stack);
   rox_objset_dynvec_edgel_del(&todel->resultsegments);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_validate_segment(Rox_EdgePostproc_Ac obj, Rox_DynVec_Edgel segment, Rox_Double Np, Rox_Double *H, Rox_Double NFA_min)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   int last;
   Rox_Segment_Part_Struct toadd;
   Rox_Segment_Part_Struct curpart;
   Rox_Uint curedgel;
   Rox_Uint minscore, pos;
   Rox_Double Hc, NFA;
   Rox_Uint p1,p2,p3,p4,l1,l2;

   rox_dynvec_segment_part_reset(obj->validation_stack);

   toadd.data = segment->data;
   toadd.length = segment->used;
   rox_dynvec_segment_part_append(obj->validation_stack, &toadd);

   while (obj->validation_stack->used > 0)
   {
      last = obj->validation_stack->used - 1;
      curpart = obj->validation_stack->data[last];
      obj->validation_stack->used--;

      pos = 0;

      // Find the minimal value
      minscore = MAX_GRADIENT;
      for (curedgel = 0; curedgel <curpart.length; curedgel++)
      {
         if (curpart.data[curedgel].score < minscore)
         {
            minscore = curpart.data[curedgel].score;
            pos = curedgel;
         }
      }

      // Check NFA
      Hc = H[minscore];
      NFA = Np * pow((double)Hc, (int)curpart.length);
      if (NFA < NFA_min)
      {
         Rox_DynVec_Edgel segment_to_add = NULL;

         error = rox_dynvec_edgel_new(&segment_to_add, 5);
         ROX_ERROR_CHECK_TERMINATE ( error );

         for (curedgel = 0; curedgel <curpart.length; curedgel++)
         {
            error = rox_dynvec_edgel_append(segment_to_add, &curpart.data[curedgel]);

            if (error)
            {
               if(segment_to_add) rox_dynvec_edgel_del(&segment_to_add);
               goto function_terminate;
            }
         }

         rox_objset_dynvec_edgel_append(obj->resultsegments, segment_to_add);
      }
      else
      {
         p1 = 0;
         p2 = pos - 1;
         p3 = pos + 1;
         p4 = curpart.length - 1;
         l1 = p2 - p1 + 1;
         l2 = p4 - p3 + 1;

         if (l1 > 2)
         {
            toadd.data = &curpart.data[p1];
            toadd.length = l1;
            rox_dynvec_segment_part_append(obj->validation_stack, &toadd);
         }

         if (l2 > 2)
         {
            toadd.data = &curpart.data[p3];
            toadd.length = l2;
            rox_dynvec_segment_part_append(obj->validation_stack, &toadd);
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_edgepostproc_ac_process(Rox_EdgePostproc_Ac obj, Rox_ObjSet_DynVec_Edgel segments, Rox_Double min_NFA)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_DynVec_Edgel curseg = NULL;
   Rox_Uint counter[MAX_GRADIENT], val = 0, total = 0;
   Rox_Double H[MAX_GRADIENT];
   Rox_Double Np = 0.0;


   if (!obj || !segments) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_objset_dynvec_edgel_reset(obj->resultsegments);

   // Reset counter per gradient values

   for (Rox_Sint idgradient = 0; idgradient < MAX_GRADIENT; idgradient++)
   {
      counter[idgradient] = 0;
   }

   // Loop over found segments
   for (Rox_Uint idsegment = 0; idsegment < segments->used; idsegment++)
   {
      curseg = segments->data[idsegment];

      //  compute card of connected pieces

      Np = Np + (0.5 * (curseg->used * (curseg->used - 1)));

      for (Rox_Uint idedgel = 0; idedgel < curseg->used; idedgel++)
      {
         val = curseg->data[idedgel].score;
         counter[val]++;
      }
   }

   // Compute histogram sum
   total = 0;
   for (Rox_Sint idgradient = 0; idgradient < MAX_GRADIENT; idgradient++)
   {
      total += counter[idgradient];
   }

   // Compute cumulative histogram
   for (Rox_Sint idgradient = MAX_GRADIENT - 2; idgradient >= 0; idgradient--)
   {
      counter[idgradient] = counter[idgradient] + counter[idgradient + 1];
   }

   // Normalize histogram
   for (Rox_Sint idgradient = 0; idgradient < MAX_GRADIENT; idgradient++)
   {
      H[idgradient] = (double)counter[idgradient] / (double)total;
   }

   for (Rox_Uint idsegment = 0; idsegment < segments->used; idsegment++)
   {
      curseg = segments->data[idsegment];

      rox_validate_segment(obj, curseg, Np, H, min_NFA);
   }

function_terminate:
   return error;
}
