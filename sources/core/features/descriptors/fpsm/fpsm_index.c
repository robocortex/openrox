//==============================================================================
//
//    OPENROX   : File fpsm_index.c
//
//    Contents  : Implementation of fpsm_index module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fpsm_index.h"

#include <stdio.h>
#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/maths_macros.h>
#include <generated/dynvec_fpsm_feature_struct.h>
#include <generated/dynvec_sint_struct.h>
#include <generated/dynvec_fpsm_template_struct.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

//#define FPSM_INDEX_DEBUG 1

Rox_ErrorCode rox_fpsm_index_new(Rox_Fpsm_Index *obj, Rox_Uint nd, Rox_Uint ntheta, Rox_Uint m, Rox_Uint min_votes, Rox_Sint width, Rox_Sint height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Fpsm_Index ret = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   *obj = NULL;

   ret = (Rox_Fpsm_Index)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->nd = nd;
   ret->ntheta = ntheta;
   ret->m = m;
   ret->width = width;
   ret->height = height;
   ret->index = NULL;
   ret->counters = NULL;
   ret->results = NULL;
   ret->min_votes = min_votes;

   CHECK_ERROR_TERMINATE(rox_objset_dynvec_fpsm_template_new(&ret->index, 10));
   CHECK_ERROR_TERMINATE(rox_dynvec_fpsm_template_new(&ret->results, 10));
   CHECK_ERROR_TERMINATE(rox_objset_dynvec_sint_new(&ret->counters, 10));

   *obj = ret;

function_terminate:
   if (error) rox_fpsm_index_del(&ret);

   return error;
}

Rox_ErrorCode rox_fpsm_index_del(Rox_Fpsm_Index *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Fpsm_Index todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_objset_dynvec_fpsm_template_del(&todel->index);
   rox_objset_dynvec_sint_del(&todel->counters);
   rox_dynvec_fpsm_template_del(&todel->results);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_fpsm_index_init(Rox_Fpsm_Index obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Fpsm_Template toadd = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   obj->maxdist = 0;
   rox_objset_dynvec_fpsm_template_reset(obj->index);
   rox_objset_dynvec_sint_reset(obj->counters);

   for (Rox_Uint i = 0; i < obj->nd; i++)
   {
      for (Rox_Uint j = 0; j < obj->ntheta; j++)
      {
         for (Rox_Uint k = 0; k < obj->m; k++)
         {
            CHECK_ERROR_TERMINATE(rox_dynvec_fpsm_template_new(&toadd, 10));
            CHECK_ERROR_TERMINATE(rox_objset_dynvec_fpsm_template_append(obj->index, toadd));
            toadd = NULL;
         }
      }
   }

function_terminate:
   rox_dynvec_fpsm_template_del(&toadd);

   return error;
}

Rox_Sint compute_pos(Rox_Sint idcell, Rox_Double angle, Rox_Double dist, Rox_Sint nbr_dist, Rox_Sint nbr_angles, Rox_Double max_dist)
{
   Rox_Sint iangle, idist, pos;

   angle = (angle + ROX_PI) / (2 * ROX_PI); //beween 0 and 1
   angle = angle * nbr_angles;
   iangle = (Rox_Sint)angle;
   if (iangle >= nbr_angles) iangle = 0;

   dist = dist / max_dist; //between 0 and 1;
   dist = dist * nbr_dist;
   idist = (Rox_Sint)dist;
   if (idist >= nbr_dist) idist = nbr_dist - 1;

   pos = idcell * nbr_dist * nbr_angles + idist * nbr_angles + iangle;

   return pos;
}

Rox_ErrorCode rox_fpsm_index_append_object(Rox_Fpsm_Index obj, Rox_DynVec_Fpsm_Feature features, Rox_Uint object_id)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint idfeat, idcell;
   Rox_Fpsm_Feature_Struct * feat;
   Rox_Double dist;
   Rox_Sint pos;
   Rox_Fpsm_Template_Struct templ;
   Rox_DynVec_Sint lcounters;

   if (!obj || !features) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   lcounters = NULL;
   CHECK_ERROR_TERMINATE(rox_dynvec_sint_new(&lcounters, 10));

   for (idfeat = 0; idfeat < (Rox_Sint)features->used; idfeat++)
   {
      feat = &features->data[idfeat];

      for (idcell = 0; idcell < (Rox_Sint)obj->m; idcell++)
      {
         dist = (Rox_Double)feat->distances[idcell];
         if (dist > obj->maxdist)
         {
            obj->maxdist = dist;
         }
      }
   }

   templ.object_id = object_id;

   for (idfeat = 0; idfeat < (Rox_Sint)features->used; idfeat++)
   {
      feat = &features->data[idfeat];

      templ.view_id = idfeat;

      for (idcell = 0; idcell < (Rox_Sint)obj->m; idcell++)
      {
         templ.angle = feat->angles[idcell]/10000.0;//x1000 to avoid storing float
         templ.dist = (Rox_Double)feat->distances[idcell];
         pos = compute_pos(idcell, templ.angle, templ.dist, obj->nd, obj->ntheta, obj->maxdist);

         CHECK_ERROR_TERMINATE(rox_dynvec_fpsm_template_append(obj->index->data[pos], &templ));
      }

      CHECK_ERROR_TERMINATE(rox_dynvec_sint_append(lcounters, &idfeat));
   }

   CHECK_ERROR_TERMINATE(rox_objset_dynvec_sint_append(obj->counters, lcounters));

   lcounters = NULL;

function_terminate:
   rox_dynvec_sint_del(&lcounters);

   return error;
}

Rox_ErrorCode rox_fpsm_index_search(Rox_Fpsm_Index obj, Rox_Fpsm_Feature_Struct * feature)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Fpsm_Template templates;
   Rox_Fpsm_Template_Struct toadd;

   if (!obj || !feature) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_dynvec_fpsm_template_reset(obj->results);

   for (Rox_Uint idobject = 0; idobject < obj->counters->used; idobject++)
   {
      Rox_DynVec_Sint lcounters = obj->counters->data[idobject];

      for (Rox_Uint idview = 0; idview < lcounters->used; idview++)
      {
         lcounters->data[idview] = 0;
      }
   }

   for (Rox_Uint idcell = 0; idcell < obj->m; idcell++)
   {
      Rox_Sint pos = compute_pos(idcell,  feature->angles[idcell], (Rox_Double)feature->distances[idcell], obj->nd, obj->ntheta, obj->maxdist);
      templates = obj->index->data[pos];

      for (Rox_Uint idtmp = 0; idtmp < templates->used; idtmp++)
      {
         Rox_Uint idobject = templates->data[idtmp].object_id;
         Rox_Uint idview = templates->data[idtmp].view_id;

         obj->counters->data[idobject]->data[idview]++;
      }
   }

   for (Rox_Uint idobject = 0; idobject < obj->counters->used; idobject++)
   {
      Rox_DynVec_Sint lcounters = obj->counters->data[idobject];
      Rox_Sint maxview = 0;
      Rox_Sint maxcount = 0;

      for (Rox_Uint idview = 0; idview < lcounters->used; idview++)
      {
         Rox_Sint count = lcounters->data[idview];

         if (count >= maxcount)
         {
            maxcount = count;
            maxview = idview;
         }
      }

      if (maxcount >= (Rox_Sint)obj->min_votes)
      {
         toadd.object_id = idobject;
         toadd.view_id = maxview;
         CHECK_ERROR_RETURN(rox_dynvec_fpsm_template_append(obj->results, &toadd));
      }

#ifdef FPSM_INDEX_DEBUG
      rox_log("Obj %d candidates : %d, votes : %d\n", idobject, obj->results->used, maxcount);
#endif
   }

function_terminate:
   return error;
}
