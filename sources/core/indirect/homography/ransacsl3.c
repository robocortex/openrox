//==============================================================================
//
//    OPENROX   : File ransacsl3.c
//
//    Contents  : API of ransacsl3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ransacsl3.h"

#include <float.h>
#include <string.h>

#include <generated/dynvec_point2d_float_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/random/random.h>
#include <baseproc/geometry/point/point2d_matsl3_transform.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/geometry/transforms/matsl3/sl3from4points.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_match_float_select_random_4points (
   Rox_Uint * idxs,
   Rox_DynVec_Point2D_Float ref,
   Rox_DynVec_Point2D_Float cur,
   Rox_Uint *pool
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Parameters
   const Rox_Sint maxiters = 200;
   Rox_Uint card;
   Rox_Sint poolsize;
   Rox_Sint found = 0;
   Rox_Sint curpool, idpool;
   Rox_Sint nbfound;
   Rox_Sint iters;
   Rox_Float v1x, v2x, v3x, v4x;
   Rox_Float v1y, v2y, v3y, v4y;
   Rox_Float v1xp, v2xp, v3xp, v4xp;
   Rox_Float v1yp, v2yp, v3yp, v4yp;
   Rox_Float det1, det2, det3, det4;
   Rox_Float det1p, det2p, det3p, det4p;
   Rox_Sint s1, s2, s3, s4, s1p, s2p, s3p, s4p;

   card = ref->used;
   poolsize = ref->used;
   iters = 0;
   nbfound = 0;

   while (!found)
   {
      iters++;
      if (iters == maxiters)
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

      // Throw dice for one point
      curpool = rox_rand() % poolsize;

      // Update possible points
      idpool = pool[curpool];
      pool[curpool] = pool[poolsize - 1];
      pool[poolsize - 1] = idpool;
      poolsize--;

      // store one inde
      idxs[nbfound] = idpool;
      nbfound++;

      // If four points found, ready for pose estimation if ...
      if (nbfound == 4)
      {
         nbfound = 0;

         // see An Effective Rigidity Constraint for Improving RANSAC in Homography Estimation

         // Eventually reset poolsize in case we continue
         poolsize = card;

         v1x = ref->data[idxs[1]].u - ref->data[idxs[0]].u;
         v1y = ref->data[idxs[1]].v - ref->data[idxs[0]].v;
         v2x = ref->data[idxs[2]].u - ref->data[idxs[1]].u;
         v2y = ref->data[idxs[2]].v - ref->data[idxs[1]].v;
         v3x = ref->data[idxs[3]].u - ref->data[idxs[2]].u;
         v3y = ref->data[idxs[3]].v - ref->data[idxs[2]].v;
         v4x = ref->data[idxs[0]].u - ref->data[idxs[3]].u;
         v4y = ref->data[idxs[0]].v - ref->data[idxs[3]].v;

         v1xp = cur->data[idxs[1]].u - cur->data[idxs[0]].u;
         v1yp = cur->data[idxs[1]].v - cur->data[idxs[0]].v;
         v2xp = cur->data[idxs[2]].u - cur->data[idxs[1]].u;
         v2yp = cur->data[idxs[2]].v - cur->data[idxs[1]].v;
         v3xp = cur->data[idxs[3]].u - cur->data[idxs[2]].u;
         v3yp = cur->data[idxs[3]].v - cur->data[idxs[2]].v;
         v4xp = cur->data[idxs[0]].u - cur->data[idxs[3]].u;
         v4yp = cur->data[idxs[0]].v - cur->data[idxs[3]].v;

         det1 = v1x * v2y - v1y * v2x;
         det2 = v2x * v3y - v2y * v3x;
         det3 = v3x * v4y - v3y * v4x;
         det4 = v4x * v1y - v4y * v1x;

         det1p = v1xp * v2yp - v1yp * v2xp;
         det2p = v2xp * v3yp - v2yp * v3xp;
         det3p = v3xp * v4yp - v3yp * v4xp;
         det4p = v4xp * v1yp - v4yp * v1xp;

         if (fabsf(det1) < 1e-6f) continue;
         if (fabsf(det2) < 1e-6f) continue;
         if (fabsf(det3) < 1e-6f) continue;
         if (fabsf(det4) < 1e-6f) continue;
         if (fabsf(det1p) < 1e-6f) continue;
         if (fabsf(det2p) < 1e-6f) continue;
         if (fabsf(det3p) < 1e-6f) continue;
         if (fabsf(det4p) < 1e-6f) continue;

         s1 = (det1 < 0.0f) - (0.0f < det1);
         s2 = (det2 < 0.0f) - (0.0f < det2);
         s3 = (det3 < 0.0f) - (0.0f < det3);
         s4 = (det4 < 0.0f) - (0.0f < det4);
         s1p = (det1p < 0.0f) - (0.0f < det1p);
         s2p = (det2p < 0.0f) - (0.0f < det2p);
         s3p = (det3p < 0.0f) - (0.0f < det3p);
         s4p = (det4p < 0.0f) - (0.0f < det4p);

         if (s1 != s1p) continue;
         if (s2 != s2p) continue;
         if (s3 != s3p) continue;
         if (s4 != s4p) continue;

         found = 1;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_homography_check_consensus (
   Rox_Uint * cardconsensus,
   Rox_Uint *inliers,
   Rox_MatSL3 homography,
   Rox_DynVec_Point2D_Float ref2D,
   Rox_DynVec_Point2D_Float cur2D
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point2D_Float cref2D = NULL, ccur2D = NULL;
   Rox_Double diffu, diffv, dist;
   Rox_Uint countconsensus;
   Rox_Uint count_pairs;
   Rox_MatSL3 Hi;
   Rox_Double **dh, **dhi;

   if (!cardconsensus || !homography || !ref2D || !cur2D || !inliers)
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}


   error = rox_matsl3_check_size ( homography ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   count_pairs = ref2D->used;
   if (count_pairs == 0) {error = ROX_ERROR_BAD_SIZE;
      ROX_ERROR_CHECK_TERMINATE(error)}

   error = rox_matsl3_new ( &Hi );

   if (error) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cref2D = (Rox_Point2D_Float ) rox_memory_allocate(sizeof(Rox_Point2D_Float_Struct), count_pairs);
   ccur2D = (Rox_Point2D_Float ) rox_memory_allocate(sizeof(Rox_Point2D_Float_Struct), count_pairs);

   if (!cref2D || !ccur2D)
   {
      rox_matsl3_del(&Hi);
      rox_memory_delete(cref2D);
      rox_memory_delete(ccur2D);
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   }

   error = rox_array2d_double_get_data_pointer_to_pointer(&dh, homography);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dhi, Hi);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Let suppose H is normalized for fast inversion ...
   dhi[0][0] = dh[1][1] * dh[2][2] - dh[1][2] * dh[2][1];
   dhi[0][1] = -dh[0][1] * dh[2][2] + dh[0][2] * dh[2][1];
   dhi[0][2] = dh[0][1] * dh[1][2] - dh[0][2] * dh[1][1];
   dhi[1][0] = dh[2][0] * dh[1][2] - dh[1][0] * dh[2][2];
   dhi[1][1] = -dh[2][0] * dh[0][2] + dh[0][0] * dh[2][2];
   dhi[1][2] = dh[1][0] * dh[0][2] - dh[0][0] * dh[1][2];
   dhi[2][0] = -dh[2][0] * dh[1][1] + dh[1][0] * dh[2][1];
   dhi[2][1] = dh[2][0] * dh[0][1] - dh[0][0] * dh[2][1];
   dhi[2][2] = -dh[1][0] * dh[0][1] + dh[0][0] * dh[1][1];

   error = rox_point2d_float_homography(cref2D, ref2D->data, homography, count_pairs); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_point2d_float_homography(ccur2D, cur2D->data, Hi, count_pairs); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   countconsensus = 0;
   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      diffu = cref2D[i].u - cur2D->data[i].u;
      diffv = cref2D[i].v - cur2D->data[i].v;
      dist = diffu * diffu + diffv * diffv;

      diffu = ccur2D[i].u - ref2D->data[i].u;
      diffv = ccur2D[i].v - ref2D->data[i].v;
      dist += diffu * diffu + diffv * diffv;

      inliers[i] = 0;
      if (dist < 2.0)
      {
         inliers[i] = 1;
         countconsensus++;
      }
   }

   *cardconsensus = countconsensus;

function_terminate:
   rox_matsl3_del(&Hi);
   rox_memory_delete(cref2D);
   rox_memory_delete(ccur2D);

   return error;
}

Rox_ErrorCode rox_points_build_homography_inliers_subset(Rox_DynVec_Point2D_Float refinliers, Rox_DynVec_Point2D_Float curinliers, Rox_Uint *inliers, Rox_DynVec_Point2D_Float cur2D, Rox_DynVec_Point2D_Float ref2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!refinliers || !curinliers || !inliers || !cur2D || !ref2D) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint count = 0;
   Rox_Uint count_pairs = cur2D->used;
   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      if (inliers[i]) count++;
   }


   if (count == 0) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   refinliers->used = 0;
   curinliers->used = 0;

   Rox_Uint pos = 0;
   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      if (!inliers[i]) continue;

      refinliers->data[pos] = ref2D->data[i];
      curinliers->data[pos] = cur2D->data[i];
      pos++;

      rox_dynvec_point2d_float_usecells(refinliers, 1);
      rox_dynvec_point2d_float_usecells(curinliers, 1);

      if (pos == count) break;
   }

function_terminate:
   return error;
}

// See RANSAC for Dummies by Marco Zuliani for equations and proofs of generic ransac
Rox_ErrorCode rox_ransac_homography(Rox_MatSL3 homography, Rox_DynVec_Point2D_Float inlierscur, Rox_DynVec_Point2D_Float inliersref, Rox_DynVec_Point2D_Float cur, Rox_DynVec_Point2D_Float ref)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   // Parameters
   const Rox_Uint minsize_mss = 4;
   const Rox_Uint max_iterations = 5000;
   const Rox_Double log_probability_of_never_selecting_good_subset = log(1e-2);

   Rox_Uint idxs[4];
   Rox_Uint iter = 0, i;
   Rox_Point2D_Float_Struct localref[4];
   Rox_Point2D_Float_Struct localcur[4];
   Rox_Uint card_consensus = 0;
   Rox_Uint max_consensus = 4;
   Rox_Uint count_pairs = 0;
   Rox_Uint ransac_max_iterations = max_iterations;
   Rox_Double q, p_q;
   Rox_MatSL3 current_hom = NULL;
   Rox_Uint * cur_inliers = NULL;
   Rox_Uint * best_inliers = NULL;
   Rox_Uint * pool = NULL;

   // Input check

   if (!homography || !cur || !ref || !inlierscur || !inliersref) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   count_pairs = cur->used;
   if (count_pairs != ref->used) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (count_pairs < minsize_mss) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matsl3_check_size ( homography );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Create buffers
   cur_inliers = (Rox_Uint *) rox_memory_allocate(sizeof(Rox_Uint), count_pairs);
   best_inliers = (Rox_Uint *) rox_memory_allocate(sizeof(Rox_Uint), count_pairs);
   pool = (Rox_Uint *) rox_memory_allocate(sizeof(Rox_Uint), count_pairs);

   error = rox_matsl3_new ( &current_hom );
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (!cur_inliers || !best_inliers || !pool || error)
   {
      rox_memory_delete(pool);
      rox_memory_delete(cur_inliers);
      rox_memory_delete(best_inliers);
      rox_matsl3_del ( &current_hom );
      {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}
   }

   // Initialize random pool
   for (i = 0; i < count_pairs; i++) pool[i] = i;

   // Trials loop
   while (iter < max_iterations && iter < ransac_max_iterations)
   {
      // Update iter first, for "continue" sake
      iter++;

      // Select a subset
      error = rox_match_float_select_random_4points(idxs, ref, cur, pool);
      if (error) continue;

      // Buffer with subset
      localref[0] = ref->data[idxs[0]];
      localref[1] = ref->data[idxs[1]];
      localref[2] = ref->data[idxs[2]];
      localref[3] = ref->data[idxs[3]];
      localcur[0] = cur->data[idxs[0]];
      localcur[1] = cur->data[idxs[1]];
      localcur[2] = cur->data[idxs[2]];
      localcur[3] = cur->data[idxs[3]];

      // Compute coarse pose given subset
      error = rox_matsl3_from_4_points_float(current_hom, localref, localcur);
      if (error) continue;

      // Check poses

      error = rox_homography_check_consensus(&card_consensus, cur_inliers, current_hom, ref, cur);
      if (error) continue;

      // If score for current pose is better than ever, keep it
      if (card_consensus > max_consensus)
      {
         // Update pose
         max_consensus = card_consensus;
         rox_matsl3_copy ( homography, current_hom );
         memcpy(best_inliers, cur_inliers, sizeof(Rox_Uint) * count_pairs);

         // Ransac update
         q = pow(((double) max_consensus) / ((double) count_pairs), (int)minsize_mss);
         p_q = 1.0 - q;

         if (p_q < DBL_EPSILON) ransac_max_iterations = 0;
         else
         {
            ransac_max_iterations = (int)(0.5 + (log_probability_of_never_selecting_good_subset / log(p_q)));
         }
      }
   }

   error = rox_points_build_homography_inliers_subset(inliersref, inlierscur, best_inliers, cur, ref);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Minimal consensus check for validity 
   if (max_consensus <= 6) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   function_terminate:

   rox_memory_delete(cur_inliers);
   rox_memory_delete(best_inliers);
   rox_memory_delete(pool);
   rox_matsl3_del ( &current_hom );

   return error;
}
