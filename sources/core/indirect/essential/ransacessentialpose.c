//==============================================================================
//
//    OPENROX   : File ransacessentialpose.c
//
//    Contents  : API of ransacessentialpose module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ransacessentialpose.h"
#include "ransacessentialcommon.h"

#include <float.h>
#include <string.h>

#include <generated/dynvec_point2d_float_struct.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/array/fill/fillval.h>
#include <baseproc/array/fill/fillunit.h>

#include <core/indirect/euclidean/triangulate.h>
#include <core/indirect/essential/e5points.h>
#include <core/indirect/essential/essentialposes.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_essential_check_3Dconsensus(Rox_Uint * cardconsensus, Rox_Uint * inliers, Rox_MatSE3 pose, Rox_MatUT3 calib, Rox_DynVec_Point2D_Float ref2D, Rox_DynVec_Point2D_Float cur2D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint countconsensus, reversed;
   Rox_Uint count_pairs;
   Rox_Double ** dt, **dk;
   Rox_Point2D_Double_Struct ref;
   Rox_Point2D_Double_Struct cur, diff;
   Rox_Point3D_Double_Struct triangulated;
   Rox_Point3D_Double_Struct transformed;
   Rox_Double dist;
   Rox_Double px, py, u0, v0;

   if (!cardconsensus || !pose || !ref2D || !cur2D || !inliers) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_matse3_check_size ( pose ); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_matut3_check_size ( calib ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   count_pairs = ref2D->used;
   if (count_pairs == 0) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_get_data_pointer_to_pointer(&dt, pose);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dk, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   px = dk[0][0];
   py = dk[1][1];
   u0 = dk[0][2];
   v0 = dk[1][2];

   countconsensus = 0;

   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      inliers[i] = 0;
      reversed = 0;

      ref.u = ref2D->data[i].u;
      ref.v = ref2D->data[i].v;
      cur.u = cur2D->data[i].u;
      cur.v = cur2D->data[i].v;

      error = rox_pose_triangulate_oneway(&triangulated, &ref, &cur, pose);
      if (error) continue;

      if (triangulated.Z < DBL_EPSILON) reversed = 1;

      transformed.X = dt[0][0] * triangulated.X + dt[0][1] * triangulated.Y + dt[0][2] * triangulated.Z + dt[0][3];
      transformed.Y = dt[1][0] * triangulated.X + dt[1][1] * triangulated.Y + dt[1][2] * triangulated.Z + dt[1][3];
      transformed.Z = dt[2][0] * triangulated.X + dt[2][1] * triangulated.Y + dt[2][2] * triangulated.Z + dt[2][3];

      if (transformed.Z < DBL_EPSILON) reversed = 1;

      diff.u = (px * (transformed.X / transformed.Z) + u0) - (px*cur.u+u0);
      diff.v = (py * (transformed.Y / transformed.Z) + v0) - (py*cur.v+v0);
      dist = sqrt(diff.u * diff.u + diff.v * diff.v);

      if (dist < 1.0)
      {
         if (reversed)
         {
            *cardconsensus = 0;
            error = ROX_ERROR_NONE;
            goto function_terminate;
         }

         inliers[i] = 1;
         countconsensus++;
      }
   }

   *cardconsensus = countconsensus;

function_terminate:
   return error;
}

Rox_ErrorCode rox_essential_check_poses(Rox_Uint * idpose, Rox_Array2D_Double * poses, Rox_Point2D_Double  refs, Rox_Point2D_Double  curs)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint found;
   Rox_Point3D_Double_Struct triangulated;
   Rox_Point3D_Double_Struct triangulatedtrans;

   if (!idpose || !poses) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *idpose = 0;

   found = -1;
   for ( Rox_Sint i = 0; i < 4; i++)
   {
      Rox_Double ** dt = NULL;
      error = rox_array2d_double_get_data_pointer_to_pointer( &dt, poses[i]);
      ROX_ERROR_CHECK_TERMINATE ( error );

      if (dt[2][2] < 0.0) continue;

      // Check the first point cheirality
      error = rox_pose_triangulate_simple(&triangulated, &refs[0], &curs[0], poses[i]);
      if (error) continue;
      if (triangulated.Z < DBL_EPSILON) continue;
      triangulatedtrans.Z = dt[2][0] * triangulated.X + dt[2][1] * triangulated.Y + dt[2][2] * triangulated.Z + dt[2][3];
      if (triangulatedtrans.Z < DBL_EPSILON) continue;

      found = i;

      // Verify that the other points have the same cheirality
      for ( Rox_Sint j = 1; j < 5; j++)
      {
         error = rox_pose_triangulate_simple(&triangulated, &refs[j], &curs[j], poses[i]);
         if (error) continue;
         if (triangulated.Z < DBL_EPSILON)
         {
            found = -1;
            break;
         }

         triangulatedtrans.Z = dt[2][0] * triangulated.X + dt[2][1] * triangulated.Y + dt[2][2] * triangulated.Z + dt[2][3];
         if (triangulatedtrans.Z < DBL_EPSILON)
         {
            found = -1;
            break;
         }
      }
   }

   if (found < 0) 
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error); } 

   *idpose = found;

function_terminate:
   return error;
}

Rox_ErrorCode rox_ransac_essential_pose(Rox_Array2D_Double pose, Rox_Array2D_Double calib, Rox_DynVec_Point2D_Float inlierscur, Rox_DynVec_Point2D_Float inliersref, Rox_DynVec_Point2D_Float cur, Rox_DynVec_Point2D_Float ref)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint idxs[5];
   Rox_Uint iter, i, idpose;
   Rox_Point2D_Double_Struct localref[5];
   Rox_Point2D_Double_Struct localcur[5];
   Rox_Array2D_Double essentials[10];
   Rox_Uint card_consensus;
   Rox_Uint max_consensus;
   Rox_Uint ransac_max_iterations;
   Rox_Double q, p_q;
   Rox_Uint * cur_inliers = NULL;
   Rox_Uint * best_inliers = NULL;
   Rox_Uint * pool = NULL;
   Rox_Uint count_pairs;
   Rox_Uint count_essentials;
   Rox_Array2D_Double poses[4];
   Rox_Double px, py, u0, v0;
   Rox_Double ** dk = NULL;

   // See RANSAC for Dummies by Marco Zuliani for equations and proofs of generic ransac

   // Constants
   const Rox_Uint minsize_mss = 5;
   const Rox_Uint max_iterations = 30000;
   const Rox_Double log_probability_of_never_selecting_good_subset = log(1e-1);
   max_consensus = 5;
   iter = 0;
   ransac_max_iterations = max_iterations;

   // Input check
   if (!pose || !cur || !ref || !inlierscur || !inliersref)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
  }

   error = rox_matse3_check_size ( pose );
   ROX_ERROR_CHECK_TERMINATE(error)

   error = rox_matut3_check_size ( calib );
   ROX_ERROR_CHECK_TERMINATE(error)

   count_pairs = cur->used;
   
   if (count_pairs != ref->used) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (count_pairs < minsize_mss)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   error = rox_matse3_check_size ( pose );  
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer(&dk, calib);
   ROX_ERROR_CHECK_TERMINATE ( error );

   px = dk[0][0];
   py = dk[1][1];
   u0 = dk[0][2];
   v0 = dk[1][2];

   // Create buffers
   for (i = 0; i < 10; i++) essentials[i] = 0;
   for (i = 0; i < 4; i++) poses[i] = 0;

   cur_inliers = (Rox_Uint*)rox_memory_allocate(sizeof(Rox_Uint), count_pairs);
   best_inliers = (Rox_Uint*)rox_memory_allocate(sizeof(Rox_Uint), count_pairs);
   pool = (Rox_Uint*)rox_memory_allocate(sizeof(Rox_Uint), count_pairs);

   for (i = 0; i < 10; i++)
   {
      error = rox_matrix_new ( &essentials[i], 3, 3 );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }
   for (i = 0; i < 4; i++)
   {
      error = rox_matse3_new ( &poses[i] );
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   if (!cur_inliers || !best_inliers || !pool || error)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Initialize random pool
   for (i = 0; i < count_pairs; i++) pool[i] = i;

   // Trials loop
   while (iter < max_iterations && iter < ransac_max_iterations)
   {
      // Update iter first, for "continue" sake
      iter++;

      // Select a subset
      error = rox_match_float_select_random_5points(idxs, ref, cur, pool, px, py, u0, v0);
      if (error) continue;

      // Buffer with subset
      for (i = 0; i < 5; i++)
      {
         localref[i].u = ref->data[idxs[i]].u;
         localref[i].v = ref->data[idxs[i]].v;
         localcur[i].u = cur->data[idxs[i]].u;
         localcur[i].v = cur->data[idxs[i]].v;
      }

      // Compute coarse pose given subset
      error = rox_essential_from_5_points_nister(essentials, &count_essentials, localref, localcur);
      if (error) continue;

      // Check poses
      for (i = 0; i < count_essentials; i++)
      {
         error = rox_essential_possible_poses(poses, essentials[i]);
         if (error) continue;

         error = rox_essential_check_poses(&idpose, poses, localref, localcur);
         if (error) continue;

         error = rox_essential_check_3Dconsensus(&card_consensus, cur_inliers, poses[idpose], calib, ref, cur);
         if (error) continue;

         // If score for current pose is better than ever, keep it
         if (card_consensus > max_consensus)
         {
            rox_matse3_copy(pose, poses[idpose]);

            // Update pose
            max_consensus = card_consensus;
            memcpy(best_inliers, cur_inliers, sizeof(Rox_Uint) * count_pairs);

            // Ransac update
            q = pow(((double)max_consensus) / ((double)count_pairs), (int)minsize_mss);
            p_q = 1.0 - q;

            if (p_q < DBL_EPSILON) ransac_max_iterations = 0;
            else
            {
               ransac_max_iterations = (int)(0.5 + (log_probability_of_never_selecting_good_subset / log(p_q)));
            }
         }
      }
   }

   error = rox_essential_check_3Dconsensus(&card_consensus, cur_inliers, pose, calib, ref, cur); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_points_build_essential_inliers_subset(inliersref, inlierscur, best_inliers, cur, ref); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Minimal consensus check for validity
   if (max_consensus <= 6) {error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE(error)}

function_terminate:

   rox_memory_delete(cur_inliers);
   rox_memory_delete(best_inliers);
   rox_memory_delete(pool);
   for (i = 0; i < 10; i++) rox_matrix_del(&essentials[i]);
   for (i = 0; i < 4; i++) rox_matse3_del(&poses[i]);

   return error;
}
