//==============================================================================
//
//    OPENROX   : File ehid_target.c
//
//    Contents  : Implementation of ehid_target module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ehid_target.h"
#include "ehid_target_struct.h"

#include <generated/dynvec_ehid_point_struct.h>
#include <generated/dynvec_ehid_match_struct.h>
#include <generated/dynvec_uint_struct.h>

#include <system/errors/errors.h>
#include <system/memory/memory.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/random/random.h>
#include <baseproc/array/robust/tukey.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>

#include <core/indirect/euclidean/p3points.h>
#include <core/indirect/euclidean/vvspointsse3.h>
#include <core/indirect/homography/vvspointssl3.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ehid_target_new(Rox_Ehid_Target * ehid_target)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Target ret = NULL;


   if (!ehid_target) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *ehid_target = NULL;

   ret = (Rox_Ehid_Target)rox_memory_allocate(sizeof(struct Rox_Ehid_Target_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->vpdifferences = 0;
   ret->vprefs = 0;
   ret->vprefs2d = 0;
   ret->vpcurs = 0;
   ret->allrefs = 0;
   ret->allrefs2d = 0;
   ret->allcurs = 0;

   ret->width_meters = 1;
   ret->width_pixels = 1;
   ret->height_meters = 1;
   ret->height_pixels = 1;

   ret->bestvp = 0;
   ret->bestvpcard = 0;
   ret->posefound = 0;
   ret->calib_input = 0;
   ret->coarse_poses = 0;
   ret->pose = 0;
   ret->homography = 0;
   ret->best_homography = 0;
   ret->best_score_minimization = 0;
   ret->best_score_p3p = 0;
   ret->used_cur = 0;

   for ( Rox_Sint idvp = 0; idvp < 18; idvp++)
   {
      ret->primarymatches[idvp] = 0;
      ret->secondarymatches[idvp] = 0;
   }

   for ( Rox_Sint idvp = 0; idvp < 18; idvp++)
   {

      error = rox_dynvec_ehid_match_new(&ret->primarymatches[idvp], 50);  
      ROX_ERROR_CHECK_TERMINATE ( error );
      error = rox_dynvec_ehid_match_new(&ret->secondarymatches[idvp], 50);  
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   error = rox_dynvec_point3d_float_new(&ret->vprefs, 50); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_float_new(&ret->vprefs2d, 50); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_float_new(&ret->vpcurs, 50); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point3d_float_new(&ret->allrefs, 50); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_float_new(&ret->allcurs, 50); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_point2d_float_new(&ret->allrefs2d, 50); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_dynvec_uint_new(&ret->used_cur, 50); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->calib_input, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->pose, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_new(&ret->best_homography, 3, 3); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_collection_new(&ret->coarse_poses, 4, 4, 4); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   *ehid_target = ret;

function_terminate:
   if (error) rox_ehid_target_del(&ret);
   return error;
}

Rox_ErrorCode rox_ehid_target_del(Rox_Ehid_Target * ehid_target)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Ehid_Target todel = NULL;


   if (!ehid_target) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *ehid_target;
   *ehid_target = NULL;

   rox_array2d_double_del(&todel->vpdifferences);
   rox_dynvec_point3d_float_del(&todel->vprefs);
   rox_dynvec_point2d_float_del(&todel->vprefs2d);
   rox_dynvec_point2d_float_del(&todel->vpcurs);
   rox_dynvec_point3d_float_del(&todel->allrefs);
   rox_dynvec_point2d_float_del(&todel->allrefs2d);
   rox_dynvec_point2d_float_del(&todel->allcurs);
   rox_array2d_double_collection_del(&todel->coarse_poses);
   rox_array2d_double_del(&todel->calib_input);
   rox_array2d_double_del(&todel->pose);
   rox_array2d_double_del(&todel->homography);
   rox_array2d_double_del(&todel->best_homography);
   rox_dynvec_uint_del(&todel->used_cur);

   for ( Rox_Sint idvp = 0; idvp < 18; idvp++)
   {
      rox_dynvec_ehid_match_del(&todel->primarymatches[idvp]);
      rox_dynvec_ehid_match_del(&todel->secondarymatches[idvp]);
   }

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_reset(Rox_Ehid_Target ehid_target)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ehid_target)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ehid_target->posefound = 0;

   for ( Rox_Sint idvp = 0; idvp < 18; idvp++)
   {
      rox_dynvec_ehid_match_reset(ehid_target->primarymatches[idvp]);
      rox_dynvec_ehid_match_reset(ehid_target->secondarymatches[idvp]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_compute_stats(Rox_Ehid_Target ehid_target)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ehid_target) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ehid_target->bestvp = 0;
   ehid_target->bestvpcard = 0;

   for ( Rox_Sint idvp = 0; idvp < 18; idvp++)
   {
      Rox_Sint cur = idvp;
      Rox_Sint prev = cur - 1; if (prev < 0) prev = 17;
      Rox_Sint next = cur + 1; if (next >= 18) next = 0;

      Rox_Uint card = ehid_target->primarymatches[cur]->used;
      if (card == 0) continue;

      card += ehid_target->primarymatches[prev]->used;
      card += ehid_target->primarymatches[next]->used;

      if (card > ehid_target->bestvpcard)
      {
         ehid_target->bestvpcard = card;
         ehid_target->bestvp = idvp;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_cleanup(Rox_Ehid_Target ehid_target, Rox_DynVec_Ehid_Point detectedfeats, Rox_DynVec_Ehid_Point globaldb)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_DynVec_Ehid_Match vec;
   Rox_Uint idmatch, idvp, idref;
   Rox_Uint cv;
   double x1, y1, x2, y2, dist;

   if (!ehid_target)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (idvp = 0; idvp < 18; idvp++)
   {
      vec = ehid_target->primarymatches[idvp];
      idmatch = 0;
      cv = 0;
      while (idmatch < vec->used)
      {
         idref = vec->data[idmatch].dbid;

         x1 = detectedfeats->data[vec->data[idmatch].curid].pos.u;
         y1 = detectedfeats->data[vec->data[idmatch].curid].pos.v;
         x2 = globaldb->data[vec->data[idmatch].dbid].pos.u;
         y2 = globaldb->data[vec->data[idmatch].dbid].pos.v;
         dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));

         if (dist < 1.5) cv++;

         // Is this database feature too much repeated ?
         if (globaldb->data[idref].refcount >= 3)
         {
            // Put this match in secondary list
            rox_dynvec_ehid_match_append(ehid_target->secondarymatches[idvp], &vec->data[idmatch]);

            // Remove this match from primary list
            vec->data[idmatch] = vec->data[vec->used - 1];
            vec->used--;
         }
         else
         {
            idmatch++;
         }
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_ignorebestvp(Rox_Ehid_Target ehid_target)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ehid_target) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (ehid_target->bestvp >= 18) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint cur = ehid_target->bestvp;
   Rox_Sint prev = cur - 1; if (prev < 0) prev = 17;
   Rox_Sint next = cur + 1; if (next >= 18) next = 0;

   rox_dynvec_ehid_match_reset(ehid_target->primarymatches[cur]);
   rox_dynvec_ehid_match_reset(ehid_target->primarymatches[prev]);
   rox_dynvec_ehid_match_reset(ehid_target->primarymatches[next]);
   rox_dynvec_ehid_match_reset(ehid_target->secondarymatches[cur]);
   rox_dynvec_ehid_match_reset(ehid_target->secondarymatches[prev]);
   rox_dynvec_ehid_match_reset(ehid_target->secondarymatches[next]);

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_ignoreallvp(Rox_Ehid_Target ehid_target)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!ehid_target) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   for (Rox_Sint cur = 0; cur < 18; cur++)
   {
      rox_dynvec_ehid_match_reset(ehid_target->primarymatches[cur]);
      rox_dynvec_ehid_match_reset(ehid_target->secondarymatches[cur]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_robust_score_se3(Rox_Uint * count_close, Rox_Ehid_Target ehid_target, Rox_DynVec_Ehid_Point detectedfeats, Rox_DynVec_Ehid_Point globaldb, Rox_Array2D_Double camera_calib, Rox_Array2D_Double pose)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint pos;
   Rox_Uint idmatch, idref, idcur;
   Rox_DynVec_Ehid_Match primary, secondary;
   Rox_Point2D_Double_Struct cur, ref;
   Rox_Double X,Y,Z, iZ, u, v, du, dv, dist;
   Rox_Double ** dt, **dk, **dd;
   Rox_Uint count, idused, found;


   if (!ehid_target || !detectedfeats || !globaldb || !camera_calib || !pose) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer(&dt, pose); ROX_ERROR_CHECK_TERMINATE ( error ); 
   error = rox_array2d_double_get_data_pointer_to_pointer(&dk, camera_calib); ROX_ERROR_CHECK_TERMINATE ( error ); 
   error = rox_array2d_double_get_data_pointer_to_pointer(&dd, ehid_target->vpdifferences); ROX_ERROR_CHECK_TERMINATE ( error ); 

   primary = ehid_target->primarymatches[ehid_target->bestvp];
   secondary = ehid_target->secondarymatches[ehid_target->bestvp];
   rox_dynvec_uint_reset(ehid_target->used_cur);

   count = 0;
   pos = 0;

   // Add primary matches
   for (idmatch = 0; idmatch < primary->used; idmatch++)
   {
      // Retrieve coordinates
      idref = primary->data[idmatch].dbid;
      idcur = primary->data[idmatch].curid;
      ref = globaldb->data[idref].pos_meters;
      cur = detectedfeats->data[idcur].pos;

      // Compute transformed reference
      X = dt[0][0] * ref.u + dt[0][1] * ref.v + dt[0][2] * 1.0 + dt[0][3];
      Y = dt[1][0] * ref.u + dt[1][1] * ref.v + dt[1][2] * 1.0 + dt[1][3];
      Z = dt[2][0] * ref.u + dt[2][1] * ref.v + dt[2][2] * 1.0 + dt[2][3];
      if (fabs(Z) < DBL_EPSILON) continue;

      iZ = 1.0 / Z;
      X = X * iZ;
      Y = Y * iZ;
      u = dk[0][0] * X + dk[0][2];
      v = dk[1][1] * Y + dk[1][2];

      du = u - cur.u;
      dv = v - cur.v;
      dist = (du*du + dv*dv);
      dd[pos][0] = dist;

      if (dist < 2.0)
      {
         found = 0;
         for (idused = 0; idused < ehid_target->used_cur->used && !found; idused++)
         {
            if (ehid_target->used_cur->data[idused] == idcur)
            {
               found = 1;
            }
         }

         if (!found)
         {
            rox_dynvec_uint_append(ehid_target->used_cur, &idcur);
            count++;
         }
      }

      pos++;
   }

   // Add secondary matches
   for (idmatch = 0; idmatch < secondary->used; idmatch++)
   {
      // Retrieve coordinates
      idref = secondary->data[idmatch].dbid;
      idcur = secondary->data[idmatch].curid;
      ref = globaldb->data[idref].pos_meters;
      cur = detectedfeats->data[idcur].pos;

      // Compute transformed reference
      X = dt[0][0] * ref.u + dt[0][1] * ref.v + dt[0][2] * 1.0 + dt[0][3];
      Y = dt[1][0] * ref.u + dt[1][1] * ref.v + dt[1][2] * 1.0 + dt[1][3];
      Z = dt[2][0] * ref.u + dt[2][1] * ref.v + dt[2][2] * 1.0 + dt[2][3];
      if (fabs(Z) < DBL_EPSILON) continue;

      iZ = 1.0 / Z;
      X = X * iZ;
      Y = Y * iZ;
      u = dk[0][0] * X + dk[0][2];
      v = dk[1][1] * Y + dk[1][2];

      du = u - cur.u;
      dv = v - cur.v;
      dist = (du*du + dv*dv);
      dd[pos][0] = dist;

      if (dist < 2.0)
      {
         found = 0;
         for (idused = 0; idused < ehid_target->used_cur->used && !found; idused++)
         {
            if (ehid_target->used_cur->data[idused] == idcur)
            {
               found = 1;
            }
         }

         if (!found)
         {
            rox_dynvec_uint_append(ehid_target->used_cur, &idcur);
            count++;
         }
      }

      pos++;
   }

   *count_close = count;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_robust_score_sl3(Rox_Double *score, Rox_Uint * count_close, Rox_Ehid_Target ehid_target, Rox_DynVec_Ehid_Point detectedfeats, Rox_DynVec_Ehid_Point globaldb, Rox_Array2D_Double homography)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint pos;
   Rox_Uint idmatch, idref, idcur, idw;
   Rox_DynVec_Ehid_Match primary, secondary;
   Rox_Point2D_Double_Struct cur, ref;
   Rox_Double X,Y,W, iW, u, v, du, dv, dist, sumsqwdist, wdist;
   Rox_Double ** dh, **dd, **dw;
   Rox_Uint count, found, idused;
   Rox_Array2D_Double weights, work1, work2, subdist;


   if (!ehid_target || !detectedfeats || !globaldb || !score || !homography) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   weights = NULL;
   work1 = NULL;
   work2 = NULL;
   subdist = NULL;

   error = rox_array2d_double_get_data_pointer_to_pointer(&dh, homography); ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_double_get_data_pointer_to_pointer(&dd, ehid_target->vpdifferences); ROX_ERROR_CHECK_TERMINATE ( error );

   primary = ehid_target->primarymatches[ehid_target->bestvp];
   secondary = ehid_target->secondarymatches[ehid_target->bestvp];
   rox_dynvec_uint_reset(ehid_target->used_cur);

   count = 0;
   pos = 0;

   // Add primary matches
   for (idmatch = 0; idmatch < primary->used; idmatch++)
   {
      // Retrieve coordinates
      idref = primary->data[idmatch].dbid;
      idcur = primary->data[idmatch].curid;
      ref = globaldb->data[idref].pos;
      cur = detectedfeats->data[idcur].pos;

      // Compute transformed reference
      X = dh[0][0] * ref.u + dh[0][1] * ref.v + dh[0][2];
      Y = dh[1][0] * ref.u + dh[1][1] * ref.v + dh[1][2];
      W = dh[2][0] * ref.u + dh[2][1] * ref.v + dh[2][2];
      if (fabs(W) < DBL_EPSILON) continue;

      iW = 1.0 / W;
      u = X * iW;
      v = Y * iW;

      du = u - cur.u;
      dv = v - cur.v;
      dist = sqrt(du*du + dv*dv);
      dd[pos][0] = dist;

      if (dist <= 2.0)
      {
         found = 0;
         for (idused = 0; idused < ehid_target->used_cur->used && !found; idused++)
         {
            if (ehid_target->used_cur->data[idused] == idcur)
            {
               found = 1;
            }
         }

         if (!found)
         {
            rox_dynvec_uint_append(ehid_target->used_cur, &idcur);
            count++;
         }
      }

      if (dist <= 15.0) pos++;
   }

   // Add secondary matches
   for (idmatch = 0; idmatch < secondary->used; idmatch++)
   {
      // Retrieve coordinates
      idref = secondary->data[idmatch].dbid;
      idcur = secondary->data[idmatch].curid;
      ref = globaldb->data[idref].pos;
      cur = detectedfeats->data[idcur].pos;

      // Compute transformed reference
      X = dh[0][0] * ref.u + dh[0][1] * ref.v + dh[0][2];
      Y = dh[1][0] * ref.u + dh[1][1] * ref.v + dh[1][2];
      W = dh[2][0] * ref.u + dh[2][1] * ref.v + dh[2][2];
      if (fabs(W) < DBL_EPSILON) continue;

      iW = 1.0 / W;
      u = X * iW;
      v = Y * iW;

      du = u - cur.u;
      dv = v - cur.v;
      dist = sqrt(du*du + dv*dv);
      dd[pos][0] = dist;

      if (dist <= 2.0)
      {
         found = 0;
         for (idused = 0; idused < ehid_target->used_cur->used && !found; idused++)
         {
            if (ehid_target->used_cur->data[idused] == idcur)
            {
               found = 1;
            }
         }

         if (!found)
         {
            rox_dynvec_uint_append(ehid_target->used_cur, &idcur);
            count++;
         }
      }

      if (dist <= 15.0) pos++;
   }


   error = rox_array2d_double_new(&weights, pos, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&work1, pos, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_double_new(&work2, pos, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_new_subarray2d(&subdist, ehid_target->vpdifferences, 0, 0, pos, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dw, weights);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_tukey(weights, work1, work2, subdist);
   ROX_ERROR_CHECK_TERMINATE ( error );

   sumsqwdist = 0;
   for (idw = 0; idw < pos; idw++)
   {
      wdist = dw[idw][0] * dd[idw][0];
      sumsqwdist += wdist * wdist;
   }

   *count_close = count;
   *score = sumsqwdist / pos;

function_terminate:
   rox_array2d_double_del(&weights);
   rox_array2d_double_del(&work1);
   rox_array2d_double_del(&work2);
   rox_array2d_double_del(&subdist);

   return error;
}

Rox_ErrorCode rox_ehid_target_upgrade_pose(Rox_Ehid_Target ehid_target, Rox_DynVec_Ehid_Point detectedfeats, Rox_DynVec_Ehid_Point globaldb, Rox_Array2D_Double camera_calib, Rox_Uint match1, Rox_Uint match2, Rox_Uint match3)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint valid_coarse_poses_count;
   Rox_Point3D_Double_Struct prefs[3];
   Rox_Point2D_Double_Struct pcurs[3];
   Rox_Uint idref, idcur, idpose;
   Rox_Array2D_Double curpose;
   Rox_Array2D_Double bestpose;
   Rox_Double minscore;
   Rox_Uint count_close;
   Rox_Double **dt;
   Rox_Double nX, nY, nZ, norm;


   if (!ehid_target || !detectedfeats || !globaldb || !camera_calib) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Double ** dk = NULL;
   error = rox_array2d_double_get_data_pointer_to_pointer( &dk, camera_calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Load first match coordinates
   idref = ehid_target->primarymatches[ehid_target->bestvp]->data[match1].dbid;
   idcur = ehid_target->primarymatches[ehid_target->bestvp]->data[match1].curid;
   prefs[0].X = globaldb->data[idref].pos_meters.u;
   prefs[0].Y = globaldb->data[idref].pos_meters.v;
   prefs[0].Z = 1;
   pcurs[0].u = detectedfeats->data[idcur].pos.u;
   pcurs[0].v = detectedfeats->data[idcur].pos.v;

   // Load second match coordinates
   idref = ehid_target->primarymatches[ehid_target->bestvp]->data[match2].dbid;
   idcur = ehid_target->primarymatches[ehid_target->bestvp]->data[match2].curid;
   prefs[1].X = globaldb->data[idref].pos_meters.u;
   prefs[1].Y = globaldb->data[idref].pos_meters.v;
   prefs[1].Z = 1;
   pcurs[1].u = detectedfeats->data[idcur].pos.u;
   pcurs[1].v = detectedfeats->data[idcur].pos.v;

   // Load third match coordinates
   idref = ehid_target->primarymatches[ehid_target->bestvp]->data[match3].dbid;
   idcur = ehid_target->primarymatches[ehid_target->bestvp]->data[match3].curid;
   prefs[2].X = globaldb->data[idref].pos_meters.u;
   prefs[2].Y = globaldb->data[idref].pos_meters.v;
   prefs[2].Z = 1;
   pcurs[2].u = detectedfeats->data[idcur].pos.u;
   pcurs[2].v = detectedfeats->data[idcur].pos.v;

   // Coarse pose estimation using 3 seed points
   error = rox_pose_from_3_points(ehid_target->coarse_poses, &valid_coarse_poses_count, prefs, pcurs, dk[0][0], dk[1][1], dk[0][2], dk[1][2]);

   ROX_ERROR_CHECK_TERMINATE ( error );
   
   if (valid_coarse_poses_count == 0) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Loop through possible poses
   minscore = 0;
   bestpose = NULL;
   for (idpose = 0; idpose < valid_coarse_poses_count; idpose++)
   {
      curpose = rox_array2d_double_collection_get(ehid_target->coarse_poses, idpose);

      error = rox_ehid_target_robust_score_se3(&count_close, ehid_target, detectedfeats, globaldb, camera_calib, curpose);
      if (error) continue;

      if (count_close >= minscore)
      {
         minscore = count_close;
         bestpose = curpose;
      }
   }


   //  Is the best pose worst than previously found ? 
   if (minscore < ehid_target->best_score_p3p) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //  Store best score
   ehid_target->best_score_p3p = minscore;

   //  Optimize using LS (15 iterations)
   error = rox_points_float_refine_pose_vvs(bestpose, camera_calib, ehid_target->vpcurs, ehid_target->vprefs, 15);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check score of optimized pose
   error = rox_ehid_target_robust_score_se3(&count_close, ehid_target, detectedfeats, globaldb, camera_calib, bestpose);
   ROX_ERROR_CHECK_TERMINATE ( error );


   if (count_close <= 4) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Check normal
   error = rox_array2d_double_get_data_pointer_to_pointer(&dt, bestpose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   nX = dt[0][2];
   nY = dt[1][2];
   nZ = dt[2][2];
   norm = sqrt(nX*nX+nY*nY+nZ*nZ);

   if (nZ/norm < 0.5) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //if (dt[2][3] < 0.0)
   //{
   //  rox_array2d_double_print(bestpose);
   //  {error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE(error)}
   //}

   rox_array2d_double_copy(ehid_target->pose, bestpose);

   ehid_target->best_score_minimization = count_close;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_upgrade_homography(Rox_Ehid_Target ehid_target, Rox_DynVec_Ehid_Point detectedfeats, Rox_DynVec_Ehid_Point globaldb, Rox_Uint match1, Rox_Uint match2, Rox_Uint match3, Rox_Uint match4)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Double_Struct prefs[4];
   Rox_Point2D_Double_Struct pcurs[4];
   Rox_Uint idref, idcur;
   Rox_Double score;
   Rox_Uint count_close;


   if (!ehid_target || !detectedfeats || !globaldb) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Load first match coordinates
   idref = ehid_target->primarymatches[ehid_target->bestvp]->data[match1].dbid;
   idcur = ehid_target->primarymatches[ehid_target->bestvp]->data[match1].curid;
   prefs[0].u = globaldb->data[idref].pos.u;
   prefs[0].v = globaldb->data[idref].pos.v;
   pcurs[0].u = detectedfeats->data[idcur].pos.u;
   pcurs[0].v = detectedfeats->data[idcur].pos.v;

   // Load second match coordinates
   idref = ehid_target->primarymatches[ehid_target->bestvp]->data[match2].dbid;
   idcur = ehid_target->primarymatches[ehid_target->bestvp]->data[match2].curid;
   prefs[1].u = globaldb->data[idref].pos.u;
   prefs[1].v = globaldb->data[idref].pos.v;
   pcurs[1].u = detectedfeats->data[idcur].pos.u;
   pcurs[1].v = detectedfeats->data[idcur].pos.v;

   // Load third match coordinates
   idref = ehid_target->primarymatches[ehid_target->bestvp]->data[match3].dbid;
   idcur = ehid_target->primarymatches[ehid_target->bestvp]->data[match3].curid;
   prefs[2].u = globaldb->data[idref].pos.u;
   prefs[2].v = globaldb->data[idref].pos.v;
   pcurs[2].u = detectedfeats->data[idcur].pos.u;
   pcurs[2].v = detectedfeats->data[idcur].pos.v;

   // Load fourth match coordinates
   idref = ehid_target->primarymatches[ehid_target->bestvp]->data[match4].dbid;
   idcur = ehid_target->primarymatches[ehid_target->bestvp]->data[match4].curid;
   prefs[3].u = globaldb->data[idref].pos.u;
   prefs[3].v = globaldb->data[idref].pos.v;
   pcurs[3].u = detectedfeats->data[idcur].pos.u;
   pcurs[3].v = detectedfeats->data[idcur].pos.v;

   // Coarse pose estimation using 3 seed points
   error = rox_matsl3_from_4_points_double(ehid_target->homography, prefs, pcurs);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_ehid_target_robust_score_sl3(&score, &count_close, ehid_target, detectedfeats, globaldb, ehid_target->homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Is the best pose worst than previously found ?

   if (count_close <= ehid_target->best_score_p3p) 
   { error = ROX_ERROR_TOO_LARGE_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // { error = ROX_ERROR_TOO_LARGE_VALUE; goto function_terminate; }

   // Store best score
   ehid_target->best_score_p3p = count_close;

   // Optimize using LS
   error = rox_points_float_refine_homography_vvs(ehid_target->homography, ehid_target->vpcurs, ehid_target->vprefs2d);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Check score of optimized pose
   error = rox_ehid_target_robust_score_sl3(&score, &count_close, ehid_target, detectedfeats, globaldb, ehid_target->homography);
   ROX_ERROR_CHECK_TERMINATE ( error );


   if (count_close <= 6) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ehid_target->best_score_minimization = count_close;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_estimate_pose (
   Rox_Ehid_Target ehid_target,
   Rox_DynVec_Ehid_Point detectedfeats,
   Rox_DynVec_Ehid_Point globaldb,
   Rox_MatUT3 camera_calib
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Uint max_trials_level1 = 40;
   const Rox_Uint max_trials_levelx = 10;
   const Rox_Double dist_threshold = 20.0;

   Rox_Uint iter_level1;
   Rox_Uint iter_levelx;
   Rox_Uint ptfound;
   Rox_Uint idmatch1, idmatch2, idmatch3, idmatch, idvp;
   Rox_DynVec_Ehid_Match primary, secondary;
   Rox_DynVec_Ehid_Match pri, sec;
   Rox_Ehid_Point_Struct * refpt1, *curpt1;
   Rox_Ehid_Point_Struct * refpt2, *curpt2;
   Rox_Ehid_Point_Struct * refpt3, *curpt3;
   Rox_Double curdx1, curdy1;
   Rox_Double refdx1, refdy1;
   Rox_Double curdx2, curdy2;
   Rox_Double refdx2, refdy2;
   Rox_Double rotx, roty;
   Rox_Double curdist, refdist;
   Rox_Double icurdist, irefdist;
   Rox_Double scale, rot, crot, srot;
   Rox_Double rotdif, cos123;
   Rox_Uint totalvpsize;
   Rox_Point2D_Double_Struct ref, cur;
   Rox_Point3D_Float_Struct fref;
   Rox_Point2D_Float_Struct fcur;


   if (!ehid_target || !detectedfeats || !globaldb || !camera_calib) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   primary = ehid_target->primarymatches[ehid_target->bestvp];
   secondary = ehid_target->secondarymatches[ehid_target->bestvp];

   // Create vectors for estimation
   totalvpsize = primary->used + secondary->used;
   rox_array2d_double_del(&ehid_target->vpdifferences);

   error = rox_array2d_double_new(&ehid_target->vpdifferences, totalvpsize, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Load points primary + secondary for gaussian minimization

   // Reinit dynvecs
   rox_dynvec_point3d_float_reset(ehid_target->vprefs);
   rox_dynvec_point2d_float_reset(ehid_target->vpcurs);
   rox_dynvec_point3d_float_reset(ehid_target->allrefs);
   rox_dynvec_point2d_float_reset(ehid_target->allcurs);

   for (idvp = 0; idvp < 18; idvp++)
   {
      pri = ehid_target->primarymatches[idvp];
      sec = ehid_target->secondarymatches[idvp];

      for (idmatch = 0; idmatch < pri->used; idmatch++)
      {
         cur = detectedfeats->data[pri->data[idmatch].curid].pos;
         ref = globaldb->data[pri->data[idmatch].dbid].pos_meters;

         // Add point to optimization vectors
         fref.X = (Rox_Float) ref.u;
         fref.Y = (Rox_Float) ref.v;
         fref.Z = 1.0;
         fcur.u = (Rox_Float) cur.u;
         fcur.v = (Rox_Float) cur.v;

         rox_dynvec_point3d_float_append(ehid_target->allrefs, &fref);
         rox_dynvec_point2d_float_append(ehid_target->allcurs, &fcur);

         if (idvp == ehid_target->bestvp)
         {
            rox_dynvec_point3d_float_append(ehid_target->vprefs, &fref);
            rox_dynvec_point2d_float_append(ehid_target->vpcurs, &fcur);
         }
      }

      for (idmatch = 0; idmatch < sec->used; idmatch++)
      {
         cur = detectedfeats->data[sec->data[idmatch].curid].pos;
         ref = globaldb->data[sec->data[idmatch].dbid].pos_meters;

         // Add point to optimization vectors
         fref.X = (Rox_Float) ref.u;
         fref.Y = (Rox_Float) ref.v;
         fref.Z = 1.0;
         fcur.u = (Rox_Float) cur.u;
         fcur.v = (Rox_Float) cur.v;

         rox_dynvec_point3d_float_append(ehid_target->allrefs, &fref);
         rox_dynvec_point2d_float_append(ehid_target->allcurs, &fcur);
         if (idvp == ehid_target->bestvp)
         {
            rox_dynvec_point3d_float_append(ehid_target->vprefs, &fref);
            rox_dynvec_point2d_float_append(ehid_target->vpcurs, &fcur);
         }
      }
   }

   // Initialize scores
   ehid_target->best_score_minimization = DBL_MAX;
   ehid_target->best_score_p3p = 0;
   ehid_target->posefound = 0;

   // Loop ransac like
   for (iter_level1 = 0; iter_level1 < max_trials_level1; iter_level1++)
   {
      // Choose randomly a seed point among primary matches
      // No constraint on the first point as it is alone
      idmatch1 = rox_rand() % primary->used;
      curpt1 = &detectedfeats->data[primary->data[idmatch1].curid];
      refpt1 = &globaldb->data[primary->data[idmatch1].dbid];

      scale = 1.0 / refpt1->scale;
      rot = primary->data[idmatch1].roterr;
      srot = sin(rot);
      crot = cos(rot);

      // Try to find a second compatible point
      ptfound = 0;
      for (iter_levelx = 0; iter_levelx < max_trials_levelx; iter_levelx++)
      {
         // Choose randomly a second point among primary matches
         idmatch2 = rox_rand() % primary->used;
         if (idmatch2 == idmatch1) continue;
         curpt2 = &detectedfeats->data[primary->data[idmatch2].curid];
         refpt2 = &globaldb->data[primary->data[idmatch2].dbid];

         // Compute vectors
         refdx1 = refpt2->pos.u - refpt1->pos.u;
         refdy1 = refpt2->pos.v - refpt1->pos.v;
         curdx1 = curpt2->pos.u - curpt1->pos.u;
         curdy1 = curpt2->pos.v - curpt1->pos.v;

         // Is it far enough from point 1 ?
         curdist = sqrt(curdx1*curdx1 + curdy1*curdy1);
         if (curdist < dist_threshold) continue;

         // Is the estimated distance respected ?
         refdist = sqrt(refdx1*refdx1 + refdy1*refdy1);
         if (refdist < dist_threshold) continue;
         if (curdist < refdist * scale * 0.4) continue;
         if (curdist > refdist * scale * 1.3) continue;

         // Normalize directions
         icurdist = 1.0 / curdist;
         irefdist = 1.0 / refdist;
         curdx1 *= icurdist;
         curdy1 *= icurdist;
         refdx1 *= irefdist;
         refdy1 *= irefdist;

         // Is the estimated rotation respected ?
         rotx = crot * refdx1 + srot * refdy1;
         roty = - srot * refdx1 + crot * refdy1;
         rotdif = acos(rotx * curdx1 + roty * curdy1);
         if (rotdif > 0.5236) continue;

         ptfound = 1;
         break;
      }

      // If no secondary point found, try another seed point
      if (!ptfound) continue;

      // Try to find a third compatible point
      ptfound = 0;
      for (iter_levelx = 0; iter_levelx < max_trials_levelx; iter_levelx++)
      {
         // Choose randomly a second point among primary matches
         idmatch3 = rox_rand() % primary->used;
         if (idmatch3 == idmatch1 || idmatch3 == idmatch2) continue;
         curpt3 = &detectedfeats->data[primary->data[idmatch3].curid];
         refpt3 = &globaldb->data[primary->data[idmatch3].dbid];

         // Compute vectors
         refdx2 = refpt3->pos.u - refpt1->pos.u;
         refdy2 = refpt3->pos.v - refpt1->pos.v;
         curdx2 = curpt3->pos.u - curpt1->pos.u;
         curdy2 = curpt3->pos.v - curpt1->pos.v;

         // Is it far enough from point 1 ?
         curdist = sqrt(curdx2*curdx2 + curdy2*curdy2);
         if (curdist < dist_threshold) continue;

         // Is the estimated distance respected ?
         refdist = sqrt(refdx2*refdx2 + refdy2*refdy2);
         if (refdist < dist_threshold) continue;
         if (curdist < refdist * scale * 0.4) continue;
         if (curdist > refdist * scale * 1.3) continue;

         // Normalize directions
         icurdist = 1.0 / curdist;
         irefdist = 1.0 / refdist;
         curdx2 *= icurdist;
         curdy2 *= icurdist;
         refdx2 *= irefdist;
         refdy2 *= irefdist;

         // Is the estimated rotation respected ?
         rotx = crot * refdx2 + srot * refdy2;
         roty = - srot * refdx2 + crot * refdy2;
         rotdif = acos(rotx * curdx2 + roty * curdy2);
         if (rotdif > 0.5236) continue;

         // Finally, check that the 3 points are not colinears
         cos123 = curdx2*curdx1 + curdy1*curdy2;
         if (fabs(cos123) > 0.94) continue;

         ptfound = 1;
         break;
      }

      if (!ptfound) continue;

      // A consistent triplet was found
      error = rox_ehid_target_upgrade_pose(ehid_target, detectedfeats, globaldb, camera_calib, idmatch1, idmatch2, idmatch3);
      if (error) continue;

      ehid_target->posefound = 1;
   }


   if (!ehid_target->posefound) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Optimize with all points of the target
   error = rox_points_float_refine_pose_vvs ( ehid_target->pose, camera_calib, ehid_target->allcurs, ehid_target->allrefs, 15);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_transformtools_updateZref(ehid_target->pose, ehid_target->pose, 1);
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_ehid_target_estimate_homography(Rox_Ehid_Target ehid_target, Rox_DynVec_Ehid_Point detectedfeats, Rox_DynVec_Ehid_Point globaldb)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Uint max_trials_level1 = 40;
   const Rox_Uint max_trials_levelx = 10;
   const Rox_Double dist_threshold = 20.0;

   Rox_Uint iter_level1;
   Rox_Uint iter_levelx;
   Rox_Uint ptfound;
   Rox_Uint idmatch1, idmatch2, idmatch3, idmatch4, idmatch, idvp;
   Rox_DynVec_Ehid_Match primary, secondary;
   Rox_DynVec_Ehid_Match pri, sec;
   Rox_Ehid_Point_Struct * refpt1, *curpt1;
   Rox_Ehid_Point_Struct * refpt2, *curpt2;
   Rox_Ehid_Point_Struct * refpt3, *curpt3;
   Rox_Ehid_Point_Struct * refpt4, *curpt4;
   Rox_Double curdx1, curdy1;
   Rox_Double refdx1, refdy1;
   Rox_Double curdx2, curdy2;
   Rox_Double refdx2, refdy2;
   Rox_Double curdx3, curdy3;
   Rox_Double refdx3, refdy3;
   Rox_Double rotx, roty;
   Rox_Double curdist, refdist;
   Rox_Double icurdist, irefdist;
   Rox_Double scale, rot, crot, srot;
   Rox_Double rotdif, cos123;
   Rox_Uint totalvpsize;
   Rox_Point2D_Double_Struct ref, cur;
   Rox_Point2D_Float_Struct fref;
   Rox_Point2D_Float_Struct fcur;


   if (!ehid_target || !detectedfeats || !globaldb) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   primary = ehid_target->primarymatches[ehid_target->bestvp];
   secondary = ehid_target->secondarymatches[ehid_target->bestvp];

   // Create vectors for estimation
   totalvpsize = primary->used + secondary->used;
   rox_array2d_double_del(&ehid_target->vpdifferences);

   
   error = rox_array2d_double_new(&ehid_target->vpdifferences, totalvpsize, 1); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Load points primary + secondary for gaussian minimization

   // Reinit dynvecs
   rox_dynvec_point2d_float_reset(ehid_target->vprefs2d);
   rox_dynvec_point2d_float_reset(ehid_target->vpcurs);
   rox_dynvec_point2d_float_reset(ehid_target->allrefs2d);
   rox_dynvec_point2d_float_reset(ehid_target->allcurs);

   for (idvp = 0; idvp < 18; idvp++)
   {
      pri = ehid_target->primarymatches[idvp];
      sec = ehid_target->secondarymatches[idvp];

      for (idmatch = 0; idmatch < pri->used; idmatch++)
      {
         cur = detectedfeats->data[pri->data[idmatch].curid].pos;
         ref = globaldb->data[pri->data[idmatch].dbid].pos;

         // Add point to optimization vectors
         fref.u = (Rox_Float) ref.u;
         fref.v = (Rox_Float) ref.v;
         fcur.u = (Rox_Float) cur.u;
         fcur.v = (Rox_Float) cur.v;

         rox_dynvec_point2d_float_append(ehid_target->allrefs2d, &fref);
         rox_dynvec_point2d_float_append(ehid_target->allcurs, &fcur);

         if (idvp == ehid_target->bestvp)
         {
            rox_dynvec_point2d_float_append(ehid_target->vprefs2d, &fref);
            rox_dynvec_point2d_float_append(ehid_target->vpcurs, &fcur);
         }
      }

      for (idmatch = 0; idmatch < sec->used; idmatch++)
      {
         cur = detectedfeats->data[sec->data[idmatch].curid].pos;
         ref = globaldb->data[sec->data[idmatch].dbid].pos;

         // Add point to optimization vectors
         fref.u = (Rox_Float) ref.u;
         fref.v = (Rox_Float) ref.v;
         fcur.u = (Rox_Float) cur.u;
         fcur.v = (Rox_Float) cur.v;

         rox_dynvec_point2d_float_append(ehid_target->allrefs2d, &fref);
         rox_dynvec_point2d_float_append(ehid_target->allcurs, &fcur);

         if (idvp == ehid_target->bestvp)
         {
            rox_dynvec_point2d_float_append(ehid_target->vprefs2d, &fref);
            rox_dynvec_point2d_float_append(ehid_target->vpcurs, &fcur);
         }
      }
   }

   // Initialize scores
   ehid_target->best_score_minimization = DBL_MAX;
   ehid_target->best_score_p3p = 0;
   ehid_target->posefound = 0;

   // Loop ransac like
   for (iter_level1 = 0; iter_level1 < max_trials_level1; iter_level1++)
   {
      // Choose randomly a seed point among primary matches
      // No constraint on the first point as it is alone
      idmatch1 = rox_rand() % primary->used;
      curpt1 = &detectedfeats->data[primary->data[idmatch1].curid];
      refpt1 = &globaldb->data[primary->data[idmatch1].dbid];

      scale = 1.0 / refpt1->scale;
      rot = primary->data[idmatch1].roterr;
      srot = sin(rot);
      crot = cos(rot);

      // Try to find a second compatible point
      ptfound = 0;
      for (iter_levelx = 0; iter_levelx < max_trials_levelx; iter_levelx++)
      {
         // Choose randomly a second point among primary matches
         idmatch2 = rox_rand() % primary->used;
         if (idmatch2 == idmatch1) continue;
         curpt2 = &detectedfeats->data[primary->data[idmatch2].curid];
         refpt2 = &globaldb->data[primary->data[idmatch2].dbid];

         // Compute vectors
         refdx1 = refpt2->pos.u - refpt1->pos.u;
         refdy1 = refpt2->pos.v - refpt1->pos.v;
         curdx1 = curpt2->pos.u - curpt1->pos.u;
         curdy1 = curpt2->pos.v - curpt1->pos.v;

         // Is it far enough from point 1 ?
         curdist = sqrt(curdx1*curdx1 + curdy1*curdy1);
         if (curdist < dist_threshold) continue;

         // Is the estimated distance respected ?
         refdist = sqrt(refdx1*refdx1 + refdy1*refdy1);
         if (refdist < dist_threshold) continue;
         if (curdist < refdist * scale * 0.4) continue;
         if (curdist > refdist * scale * 1.3) continue;

         // Normalize directions
         icurdist = 1.0 / curdist;
         irefdist = 1.0 / refdist;
         curdx1 *= icurdist;
         curdy1 *= icurdist;
         refdx1 *= irefdist;
         refdy1 *= irefdist;

         // Is the estimated rotation respected ?
         rotx = crot * refdx1 + srot * refdy1;
         roty = - srot * refdx1 + crot * refdy1;
         rotdif = acos(rotx * curdx1 + roty * curdy1);
         if (rotdif > 0.5236) continue;

         ptfound = 1;
         break;
      }

      // If no secondary point found, try another seed point
      if (!ptfound) continue;

      // Try to find a third compatible point
      ptfound = 0;
      for (iter_levelx = 0; iter_levelx < max_trials_levelx; iter_levelx++)
      {
         // Choose randomly a second point among primary matches
         idmatch3 = rox_rand() % primary->used;
         if (idmatch3 == idmatch1 || idmatch3 == idmatch2) continue;
         curpt3 = &detectedfeats->data[primary->data[idmatch3].curid];
         refpt3 = &globaldb->data[primary->data[idmatch3].dbid];

         // Compute vectors
         refdx2 = refpt3->pos.u - refpt1->pos.u;
         refdy2 = refpt3->pos.v - refpt1->pos.v;
         curdx2 = curpt3->pos.u - curpt1->pos.u;
         curdy2 = curpt3->pos.v - curpt1->pos.v;

         // Is it far enough from point 1 ?
         curdist = sqrt(curdx2*curdx2 + curdy2*curdy2);
         if (curdist < dist_threshold) continue;

         // Is the estimated distance respected ?
         refdist = sqrt(refdx2*refdx2 + refdy2*refdy2);
         if (refdist < dist_threshold) continue;
         if (curdist < refdist * scale * 0.4) continue;
         if (curdist > refdist * scale * 1.3) continue;

         // Normalize directions
         icurdist = 1.0 / curdist;
         irefdist = 1.0 / refdist;
         curdx2 *= icurdist;
         curdy2 *= icurdist;
         refdx2 *= irefdist;
         refdy2 *= irefdist;

         // Is the estimated rotation respected ?
         rotx = crot * refdx2 + srot * refdy2;
         roty = - srot * refdx2 + crot * refdy2;
         rotdif = acos(rotx * curdx2 + roty * curdy2);
         if (rotdif > 0.5236) continue;

         // Finally, check that the 3 points are not colinears
         cos123 = curdx2*curdx1 + curdy1*curdy2;
         if (fabs(cos123) > 0.94) continue;

         ptfound = 1;
         break;
      }

      if (!ptfound) continue;

      // Try to find a fourth compatible point
      ptfound = 0;
      for (iter_levelx = 0; iter_levelx < max_trials_levelx; iter_levelx++)
      {
         // Choose randomly a second point among primary matches
         idmatch4 = rox_rand() % primary->used;
         if (idmatch4 == idmatch1 || idmatch4 == idmatch2 || idmatch4 == idmatch3) continue;
         curpt4 = &detectedfeats->data[primary->data[idmatch4].curid];
         refpt4 = &globaldb->data[primary->data[idmatch4].dbid];

         // Compute vectors
         refdx3 = refpt4->pos.u - refpt1->pos.u;
         refdy3 = refpt4->pos.v - refpt1->pos.v;
         curdx3 = curpt4->pos.u - curpt1->pos.u;
         curdy3 = curpt4->pos.v - curpt1->pos.v;

         // Is it far enough from point 1 ?
         curdist = sqrt(curdx3*curdx3 + curdy3*curdy3);
         if (curdist < dist_threshold) continue;

         // Is the estimated distance respected ?
         refdist = sqrt(refdx3*refdx3 + refdy3*refdy3);
         if (refdist < dist_threshold) continue;
         if (curdist < refdist * scale * 0.4) continue;
         if (curdist > refdist * scale * 1.3) continue;

         // Normalize directions
         icurdist = 1.0 / curdist;
         irefdist = 1.0 / refdist;
         curdx3 *= icurdist;
         curdy3 *= icurdist;
         refdx3 *= irefdist;
         refdy3 *= irefdist;

         // Is the estimated rotation respected ?
         rotx = crot * refdx3 + srot * refdy3;
         roty = - srot * refdx3 + crot * refdy3;
         rotdif = acos(rotx * curdx3 + roty * curdy3);
         if (rotdif > 0.5236) continue;

         // Finally, check that the 3 points are not colinears
         cos123 = curdx3*curdx1 + curdy1*curdy3;
         if (fabs(cos123) > 0.94) continue;
         cos123 = curdx3*curdx2 + curdy2*curdy3;
         if (fabs(cos123) > 0.94) continue;

         ptfound = 1;
         break;
      }

      if (!ptfound) continue;

      // A consistent group of four was found
      error = rox_ehid_target_upgrade_homography(ehid_target, detectedfeats, globaldb, idmatch1, idmatch2, idmatch3, idmatch4);
      if (error) continue;

      error = rox_array2d_double_copy(ehid_target->best_homography, ehid_target->homography);
      if (error) continue;

      ehid_target->posefound = 1;
   }

   if (!ehid_target->posefound) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Optimize with all points of the target
   error = rox_points_float_refine_homography_vvs(ehid_target->best_homography, ehid_target->allcurs, ehid_target->allrefs2d);

   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}
