//==============================================================================
//
//    OPENROX   : File tlid_matcher.c
//
//    Contents  : Implementation of tlid_matcher module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "tlid_matcher.h"

#include <baseproc/maths/maths_macros.h>
#include <float.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/maths_macros.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_tlid_matcher_new(Rox_Tlid_Matcher *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tlid_Matcher ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   *obj = NULL;

   ret = (Rox_Tlid_Matcher)rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->reference_matched = NULL;
   ret->current_matched = NULL;
   ret->reference_matched_unfiltered = NULL;
   ret->current_matched_unfiltered = NULL;

   for ( Rox_Sint idbin = 0; idbin < NB_ROTATION_BINS; idbin++)
   {
      ret->rotation_bins[idbin] = NULL;
   }

   error = rox_dynvec_point2d_double_new(&ret->reference_matched, 10);

   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_dynvec_point2d_double_new(&ret->current_matched, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );
  
   error = rox_dynvec_point2d_double_new(&ret->reference_matched_unfiltered, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_point2d_double_new(&ret->current_matched_unfiltered, 10);
   ROX_ERROR_CHECK_TERMINATE ( error );

   for ( Rox_Sint idbin = 0; idbin < NB_ROTATION_BINS; idbin++)
   {
      error = rox_dynvec_uint_new(&ret->rotation_bins[idbin], 10);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *obj = ret;

function_terminate:
   if (error) rox_tlid_matcher_del(&ret);
   return error;
}

Rox_ErrorCode rox_tlid_matcher_del(Rox_Tlid_Matcher *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Tlid_Matcher todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_point2d_double_del(&todel->reference_matched);
   rox_dynvec_point2d_double_del(&todel->current_matched);
   rox_dynvec_point2d_double_del(&todel->reference_matched_unfiltered);
   rox_dynvec_point2d_double_del(&todel->current_matched_unfiltered);

   for ( Rox_Sint idbin = 0; idbin < NB_ROTATION_BINS; idbin++)
   {
      rox_dynvec_uint_del(&todel->rotation_bins[idbin]);
   }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_tlid_matcher_match(Rox_Tlid_Matcher obj, Rox_Tlid curset, Rox_Tlid refset)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idref, idcur, idcurconfirm, idcell, idmatch, index;
   Rox_Double diff, sum, min, maxscore;
   Rox_Sint minid, minid_confirm, iangle, cbestbin,bestbin;
   Rox_TLID_Segment_Struct * cur, * ref, *curconfirm;
   Rox_Point2D_Double_Struct toadd;
   Rox_Double angle, sindif, cosdif, da, mda;
   Rox_Double hist[NB_ROTATION_BINS];
   Rox_DynVec_Uint currentbin;


   if (!obj || !curset || !refset) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_point2d_double_reset(obj->reference_matched);
   rox_dynvec_point2d_double_reset(obj->current_matched);
   rox_dynvec_point2d_double_reset(obj->reference_matched_unfiltered);
   rox_dynvec_point2d_double_reset(obj->current_matched_unfiltered);

   for (iangle = 0; iangle < NB_ROTATION_BINS; iangle++)
   {
      rox_dynvec_uint_reset(obj->rotation_bins[iangle]);
      hist[iangle] = 0;
   }

   for (idcur = 0; idcur < curset->segments->used; idcur++)
   {
      cur = &curset->segments->data[idcur];
      minid = -1;
      min = DBL_MAX;
      for (idref = 0; idref < refset->segments->used; idref++)
      {

         ref = &refset->segments->data[idref];

         sum = 0;
         for (idcell = 0; idcell < 12*12*4; idcell++)
         {
            diff = ref->desc[idcell] - cur->desc[idcell];
            sum += diff * diff;
         }

         if (sum < min)
         {
            minid = idref;
            min = sum;
         }
      }

      ref = &refset->segments->data[minid];

      min = DBL_MAX;
      minid_confirm = -1;
      for (idcurconfirm = 0; idcurconfirm < curset->segments->used; idcurconfirm++)
      {
         curconfirm = &curset->segments->data[idcurconfirm];

         sum = 0;
         for (idcell = 0; idcell < 12*12*4; idcell++)
         {
            diff = ref->desc[idcell] - curconfirm->desc[idcell];
            sum += diff * diff;
         }

         if (sum < min)
         {
            minid_confirm = idcurconfirm;
            min = sum;
         }

      }

      if (minid_confirm != idcur) continue;

      cosdif = cur->direction.u * ref->direction.u + cur->direction.v * ref->direction.v;
      sindif = cur->direction.u * ref->direction.v - cur->direction.v * ref->direction.u;
      angle = ROX_PI + atan2(sindif, cosdif);
      angle = (angle / ROX_PI) * (NB_ROTATION_BINS/2);
      iangle = (Rox_Sint)angle;
      da = angle - (double)iangle;
      mda = 1.0 - da;

      hist[iangle] += mda;
      if (iangle >= NB_ROTATION_BINS-1)
      {
         hist[0] += da;
      }
      else
      {
         hist[iangle + 1] += da;
      }

      error = rox_dynvec_uint_append(obj->rotation_bins[iangle], &obj->current_matched_unfiltered->used);
      ROX_ERROR_CHECK_TERMINATE ( error );

      toadd.u = cur->midpoint.u;
      toadd.v = cur->midpoint.v;
      error = rox_dynvec_point2d_double_append(obj->current_matched_unfiltered, &toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );

      toadd.u = ref->midpoint.u;
      toadd.v = ref->midpoint.v;
      error = rox_dynvec_point2d_double_append(obj->reference_matched_unfiltered, &toadd);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   bestbin = 0;
   maxscore = 0;
   for (iangle = 0; iangle < NB_ROTATION_BINS; iangle++)
   {
      if (maxscore < hist[iangle])
      {
         maxscore = hist[iangle];
         bestbin = iangle;
      }
   }

   currentbin = obj->rotation_bins[bestbin];
   for (idmatch = 0; idmatch < currentbin->used; idmatch++)
   {
      index = currentbin->data[idmatch];

      error = rox_dynvec_point2d_double_append(obj->current_matched, &obj->current_matched_unfiltered->data[index]);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point2d_double_append(obj->reference_matched, &obj->reference_matched_unfiltered->data[index]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   cbestbin = bestbin - 1;
   if (cbestbin < 0) cbestbin = NB_ROTATION_BINS - 1;
   currentbin = obj->rotation_bins[cbestbin];
   for (idmatch = 0; idmatch < currentbin->used; idmatch++)
   {
      index = currentbin->data[idmatch];

      error = rox_dynvec_point2d_double_append(obj->current_matched, &obj->current_matched_unfiltered->data[index]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point2d_double_append(obj->reference_matched, &obj->reference_matched_unfiltered->data[index]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   cbestbin = bestbin + 1;
   if (cbestbin >= NB_ROTATION_BINS) cbestbin = 0;
   currentbin = obj->rotation_bins[cbestbin];
   for (idmatch = 0; idmatch < currentbin->used; idmatch++)
   {
      index = currentbin->data[idmatch];

      error = rox_dynvec_point2d_double_append(obj->current_matched, &obj->current_matched_unfiltered->data[index]);

      ROX_ERROR_CHECK_TERMINATE ( error );

      error = rox_dynvec_point2d_double_append(obj->reference_matched, &obj->reference_matched_unfiltered->data[index]);
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   return error;
}

