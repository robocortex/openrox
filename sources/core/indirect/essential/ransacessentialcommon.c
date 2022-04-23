//==============================================================================
//
//    OPENROX   : File ransacessentialcommon.c
//
//    Contents  : API of ransacessentialcommon module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ransacessentialcommon.h"

#include <float.h>

#include <generated/dynvec_point2d_float_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/random/random.h>

#include <inout/system/errors_print.h>

Rox_ErrorCode rox_match_float_select_random_5points(Rox_Uint *idxs, Rox_DynVec_Point2D_Float ref, Rox_DynVec_Point2D_Float cur, Rox_Uint * pool, Rox_Double px, Rox_Double py, Rox_Double u0, Rox_Double v0)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Sint maxiters = 200;
   const Rox_Sint nbp = 5;

   Rox_Uint card;
   Rox_Sint poolsize;
   Rox_Sint found = 0;
   Rox_Sint curpool, idpool;
   Rox_Sint nbfound;
   Rox_Sint iters, i, j, k, idi, idj, idk;
   Rox_Double mindist, dist, dx, dy, dx2, dy2, dline, mindline;

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
      pool[curpool] = pool[poolsize-1];
      pool[poolsize - 1] = idpool;
      poolsize--;

      // store one inde
      idxs[nbfound] = idpool;
      nbfound++;

      // If four points found, ready for pose estimation if ...
      if (nbfound == nbp)
      {
         nbfound = 0;
         // Eventually reset poolsize in case we continue
         poolsize = card;

         mindist = DBL_MAX;
         mindline = DBL_MAX;

         for (i = 0; i < nbp; i++)
         {
            idi = idxs[i];
            for (j = 0; j < nbp; j++)
            {
               if (i == j) continue;
               idj = idxs[j];

               // Check distances between points
               dx = (px * ref->data[idj].u + u0) - (px * ref->data[idi].u + u0);
               dy = (py * ref->data[idj].v + v0) - (py * ref->data[idi].v + v0);
               dist = sqrt(dx*dx + dy*dy);
               if (dist < mindist) mindist = dist;
               if (dist < DBL_EPSILON) continue;

               // Check area of other points to this line
               for (k = 0; k < nbp; k++)
               {
                  if (k == i || k == j) continue;
                  idk = idxs[k];

                  dx2 = (px * ref->data[idi].u + u0) - (px * ref->data[idk].u + u0);
                  dy2 = (py * ref->data[idi].v + v0) - (py * ref->data[idk].v + v0);

                  dline = (dx2 * dy - dx * dy2) / dist;
                  if (dline < mindline) mindline = dline;
               }

               // Check distances between points
               dx = (px * cur->data[idj].u + u0) - (px * cur->data[idi].u + u0);
               dy = (py * cur->data[idj].v + v0) - (py * cur->data[idi].v + v0);
               dist = sqrt(dx*dx + dy*dy);
               if (dist < mindist) mindist = dist;
               if (dist < DBL_EPSILON) continue;

               // Check area of other points to this line
               for (k = 0; k < nbp; k++)
               {
                  if (k == i || k == j) continue;
                  idk = idxs[k];

                  dx2 = (px * cur->data[idi].u + u0) - (px * cur->data[idk].u + u0);
                  dy2 = (py * cur->data[idi].v + v0) - (py * cur->data[idk].v + v0);

                  dline = (dx2 * dy - dx * dy2) / dist;
                  if (dline < mindline) mindline = dline;
               }
            }
         }

         // At least a distance of 20 pixels between each points
         if (mindist < 20.0) continue;
         if (fabs(mindline) < 2.0) continue;

         found = 1;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_points_build_essential_inliers_subset(Rox_DynVec_Point2D_Float refinliers, Rox_DynVec_Point2D_Float curinliers, Rox_Uint * inliers, Rox_DynVec_Point2D_Float cur2D, Rox_DynVec_Point2D_Float ref2D)
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
