//==============================================================================
//
//    OPENROX   : File ehid_matcher.c
//
//    Contents  : Implementation of ehid_matcher module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ehid_matcher.h"
#include "ehid_target.h"

#include "ehid_database_struct.h"
#include "ehid_matcher_struct.h"
#include "ehid_match_struct.h"
#include "ehid_point.h"

#include <float.h>
#include <stdio.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/random/random.h>

#include <baseproc/maths/base/basemaths.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/array/multiply/mulmatmat.h>
#include <baseproc/array/inverse/svdinverse.h>
// #include <system/time/timer.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

#define MIN_SCORE_MATCHING 4

Rox_ErrorCode rox_ehid_matcher_new(Rox_Ehid_Matcher * ehid_matcher, Rox_Uint max_templates_detected)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Matcher ret = NULL;


   if (!ehid_matcher) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *ehid_matcher = NULL;

   ret = (Rox_Ehid_Matcher)rox_memory_allocate(sizeof(struct Rox_Ehid_Matcher_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dynvec_ehid_match_new(&ret->results, 10);
   if (error)
   {
      rox_ehid_matcher_del(&ret);
      return error;
   }

   ret->max_templates_per_query = max_templates_detected;

   *ehid_matcher = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_matcher_del(Rox_Ehid_Matcher * ehid_matcher)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Matcher todel = NULL;

   if (!ehid_matcher)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ehid_matcher;
   *ehid_matcher = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_ehid_match_del(&todel->results);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_matcher_estimate_poses (
   Rox_Ehid_Matcher ehid_matcher,
   Rox_Ehid_Database db,
   Rox_DynVec_Ehid_Point detectedfeats,
   Rox_MatUT3 camera_calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idtarget;
   Rox_Uint besttarget, besttargetcard;
   Rox_Uint countfound;


   if (!ehid_matcher || !detectedfeats) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Start pose estimation, looping through possible viewpoints
   while (1)
   {
      besttarget = 0;
      besttargetcard = 0;
      countfound = 0;

      //Are all the target already found ?
      for (idtarget = 0; idtarget < db->_targets->used; idtarget++)
      {
         countfound += db->_targets->data[idtarget]->posefound;
      }
      if (countfound == db->_targets->used || countfound >= ehid_matcher->max_templates_per_query) break;

      //Compute viewpoints statistics per target
      for (idtarget = 0; idtarget < db->_targets->used; idtarget++)
      {
         rox_ehid_target_compute_stats(db->_targets->data[idtarget]);

         // Find best target
         if (besttargetcard < db->_targets->data[idtarget]->bestvpcard)
         {
            besttarget = idtarget;
            besttargetcard = db->_targets->data[idtarget]->bestvpcard;
         }
      }

      // If the best viewpoint primary cardinality is less than 6, it's time to exit
      if (besttargetcard < 6) break;

      rox_ehid_target_estimate_pose(db->_targets->data[besttarget], detectedfeats, db->_fulllist, camera_calib);

      if (!db->_targets->data[besttarget]->posefound)
      {
         // Pose not found, erase viewpoint and neighboors
         rox_ehid_target_ignorebestvp(db->_targets->data[besttarget]);
      }
      else
      {
         // Pose found, all viewpoints of the target are to be ignored
         rox_ehid_target_ignoreallvp(db->_targets->data[besttarget]);
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_matcher_estimate_homographies (
   Rox_Ehid_Matcher ehid_matcher,
   Rox_Ehid_Database db,
   Rox_DynVec_Ehid_Point detectedfeats
   )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint besttarget = 0;
   Rox_Uint besttargetcard = 0;
   Rox_Uint countfound = 0;


   if (!ehid_matcher || !detectedfeats) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Start pose estimation, looping through possible viewpoints

   while (1)
   {
      besttarget = 0;
      besttargetcard = 0;
      countfound = 0;

      // Are all the target already found ?
      for (Rox_Uint idtarget = 0; idtarget < db->_targets->used; idtarget++)
      {
         countfound += db->_targets->data[idtarget]->posefound;
      }

      if (countfound == db->_targets->used) break;

      // Compute viewpoints statistics per target
      for (Rox_Uint idtarget = 0; idtarget < db->_targets->used; idtarget++)
      {
         rox_ehid_target_compute_stats(db->_targets->data[idtarget]);

         //Find best target
         if (besttargetcard < db->_targets->data[idtarget]->bestvpcard)
         {
            besttarget = idtarget;
            besttargetcard = db->_targets->data[idtarget]->bestvpcard;
         }
      }

      // If the best viewpoint primary cardinality is less than 6, it's time to exit
      if (besttargetcard < 6) break;

      rox_ehid_target_estimate_homography(db->_targets->data[besttarget], detectedfeats, db->_fulllist);

      if (!db->_targets->data[besttarget]->posefound)
      {
         // Pose not found, erase viewpoint and neighboors
         rox_ehid_target_ignorebestvp(db->_targets->data[besttarget]);
      }
      else
      {
         // Pose found, all viewpoints of the target are to be ignored
         rox_ehid_target_ignoreallvp(db->_targets->data[besttarget]);
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_matcher_match_se3 (
   Rox_Ehid_Matcher ehid_matcher,
   Rox_Ehid_Database db,
   Rox_DynVec_Ehid_Point detectedfeats,
   Rox_MatUT3 camera_calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idcur, idref, idtarget, idmatch;
   Rox_Ehid_Point  cur = NULL, ref = NULL;
   Rox_Uint score;
   Rox_Ehid_Match_Struct match;
   Rox_Double difangle, cosdif, sindif;
   Rox_Uint binangle;


   if (!ehid_matcher || !detectedfeats) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Reset targets
   for (idtarget = 0; idtarget < db->_targets->used; idtarget++)
   {
      rox_ehid_target_reset(db->_targets->data[idtarget]);
   }

   // Reset reference counter
   for (idref = 0; idref < db->_fulllist->used; idref++)
   {
      db->_fulllist->data[idref].refcount = 0;
   }

   // Per point global matching
   for (idcur = 0; idcur < detectedfeats->used; idcur++)
   {
      cur = &(detectedfeats->data[idcur]);

      error = rox_ehid_searchtree_lookup (ehid_matcher->results, db->_trees[cur->index], cur->Description);
      if (error) continue;

      for (idmatch = 0; idmatch < ehid_matcher->results->used; idmatch++)
      {
         idref = ehid_matcher->results->data[idmatch].dbid;
         score = ehid_matcher->results->data[idmatch].score;

         ref = &db->_fulllist->data[idref];
         if (ref->dbid >= db->_targets->used) continue;

         //If score is good enough, at least secondary match
         if (score <= MIN_SCORE_MATCHING)
         {
            match.curid = idcur;
            match.dbid = idref;
            match.score = score;

            //Compute offset for angles
            cosdif = cur->dir.u * ref->dir.u + cur->dir.v * ref->dir.v;
            sindif = cur->dir.u * ref->dir.v - cur->dir.v * ref->dir.u;
            match.roterr = (Rox_Double) fast_atan2f2((Rox_Float) sindif, (Rox_Float) cosdif);
            difangle = ROX_PI + match.roterr;
            difangle = 18.0 * (difangle) / ROX_2PI;
            binangle = (Rox_Uint)difangle;
            if (binangle == 18) binangle = 0;

            //Store primary match
            if (score <= 2)
            {
               rox_dynvec_ehid_match_append(db->_targets->data[ref->dbid]->primarymatches[binangle], &match);
            }
            else //Store secondary match
            {
               rox_dynvec_ehid_match_append(db->_targets->data[ref->dbid]->secondarymatches[binangle], &match);
            }

            //Increase db reference count
            ref->refcount++;
         }
      }
   }

   // Clean Up initial match list
   for (idtarget = 0; idtarget < db->_targets->used; idtarget++)
   {
      rox_ehid_target_cleanup(db->_targets->data[idtarget], detectedfeats, db->_fulllist);
   }

   error = rox_ehid_matcher_estimate_poses(ehid_matcher, db, detectedfeats, camera_calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_matcher_match_sl3(Rox_Ehid_Matcher ehid_matcher, Rox_Ehid_Database db, Rox_DynVec_Ehid_Point detectedfeats)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idcur, idref, idtarget, idmatch;
   Rox_Ehid_Point  cur = NULL, ref = NULL;
   Rox_Uint score;
   Rox_Ehid_Match_Struct match;
   Rox_Double difangle, cosdif, sindif;
   Rox_Uint binangle;


   if (!ehid_matcher || !detectedfeats || !db) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Reset targets
   for (idtarget = 0; idtarget < db->_targets->used; idtarget++)
   {
      rox_ehid_target_reset(db->_targets->data[idtarget]);
   }

   // Reset reference counter
   for (idref = 0; idref < db->_fulllist->used; idref++)
   {
      db->_fulllist->data[idref].refcount = 0;
   }

   // Per point global matching
   for (idcur = 0; idcur < detectedfeats->used; idcur++)
   {
      cur = &(detectedfeats->data[idcur]);

      error = rox_ehid_searchtree_lookup(ehid_matcher->results, db->_trees[cur->index], cur->Description);
      if (error) continue;

      for (idmatch = 0; idmatch < ehid_matcher->results->used; idmatch++)
      {
         idref = ehid_matcher->results->data[idmatch].dbid;
         score = ehid_matcher->results->data[idmatch].score;

         ref = &db->_fulllist->data[idref];
         if (ref->dbid >= db->_targets->used) continue;

         // If score is good enough, at least secondary match
         if (score <= MIN_SCORE_MATCHING)
         {
            match.curid = idcur;
            match.dbid = idref;
            match.score = score;

            //Compute offset for angles
            cosdif = cur->dir.u * ref->dir.u + cur->dir.v * ref->dir.v;
            sindif = cur->dir.u * ref->dir.v - cur->dir.v * ref->dir.u;
            match.roterr = (Rox_Double) fast_atan2f2((Rox_Float) sindif, (Rox_Float) cosdif);
            difangle = ROX_PI + match.roterr;
            difangle = 18.0 * (difangle) / ROX_2PI;
            binangle = (Rox_Uint) difangle;
            if (binangle == 18) binangle = 0;

            // Store primary match
            if (score <= 2)
            {
               rox_dynvec_ehid_match_append(db->_targets->data[ref->dbid]->primarymatches[binangle], &match);
            }
            else // Store secondary match
            {
               rox_dynvec_ehid_match_append(db->_targets->data[ref->dbid]->secondarymatches[binangle], &match);
            }

            // Increase db reference count
            ref->refcount++;
         }
      }
   }

   // Clean Up initial match list
   for (idtarget = 0; idtarget < db->_targets->used; idtarget++)
   {
      rox_ehid_target_cleanup(db->_targets->data[idtarget], detectedfeats, db->_fulllist);
   }

   error = rox_ehid_matcher_estimate_homographies(ehid_matcher, db, detectedfeats);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
