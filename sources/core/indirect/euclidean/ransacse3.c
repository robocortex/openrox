//==============================================================================
//
//    OPENROX   : File ransacse3.c
//
//    Contents  : Implementation of ransacse3 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ransacse3.h"

#include <float.h>
#include <string.h>

#include <generated/dynvec_point3d_float_struct.h>
#include <generated/dynvec_point2d_float_struct.h>
#include <generated/dynvec_point3d_double_struct.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <generated/dynvec_sint_struct.h>

#include <baseproc/geometry/point/point2d_projection_from_point3d.h>
#include <baseproc/geometry/point/point3d_matse3_transform.h>
#include <baseproc/geometry/transforms/transform_tools.h>
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/random/random.h>

#include <core/indirect/euclidean/p3points.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>


#define MAGIC_MIN_PIXELS_DIST     25
#define MAGIC_MIN_METERS_DIST     1e-5
#define MIN_PIXELS_DIST_CONSENSUS 4.0  // From "Multiple View Geometry" the value should be 5.99


//------------------------------------------------------------------------------
//--- Float stuff
//------------------------------------------------------------------------------
Rox_ErrorCode
rox_match_double_select_random_3points(
  Rox_Uint                        *idxs,
  Rox_DynVec_Point3D_Float  ref,
  Rox_DynVec_Point2D_Float  cur,
  Rox_Uint                        *pool )
{
   Rox_ErrorCode error=ROX_ERROR_NONE;
   Rox_Uint      card;
   int           poolsize;
   int           found=0;
   int           curpool,idpool;
   int           nbfound;
   int           iters;
   int           valid;
   const int     maxiters=200;
   Rox_Double      diff1, diff2, diff3;
   Rox_Double      dist;

   card     = ref->used;
   poolsize = ref->used;
   iters    = 0;
   nbfound  = 0;

   while (!found)
   {
      iters++;
      if (iters == maxiters)
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );}

      // Throw dice for one point
      curpool = rox_rand() % poolsize;

      // Update possible points
      idpool             = pool[curpool];
      pool[curpool]      = pool[poolsize - 1];
      pool[poolsize - 1] = idpool;
      poolsize--;

      //  store one index
      idxs[nbfound] = idpool;
      nbfound++;

      // If three points found, ready for pose estimation if ...
      if (nbfound == 3)
      {
         valid   = 1;
         nbfound = 0;

         // Eventually reset poolsize in case we continue
         poolsize = card;

         diff1 = cur->data[idxs[1]].u - cur->data[idxs[0]].u;
         diff2 = cur->data[idxs[1]].v - cur->data[idxs[0]].v;
         dist  = diff1*diff1 + diff2*diff2;
         if ( dist < MAGIC_MIN_PIXELS_DIST ) continue;

         diff1 = cur->data[idxs[2]].u - cur->data[idxs[0]].u;
         diff2 = cur->data[idxs[2]].v - cur->data[idxs[0]].v;
         dist  = diff1*diff1 + diff2*diff2;
         if ( dist < MAGIC_MIN_PIXELS_DIST ) continue;

         diff1 = cur->data[idxs[2]].u - cur->data[idxs[1]].u;
         diff2 = cur->data[idxs[2]].v - cur->data[idxs[1]].v;
         dist  = diff1*diff1 + diff2*diff2;
         if ( dist < MAGIC_MIN_PIXELS_DIST ) continue;

         diff1 = ref->data[idxs[1]].X - ref->data[idxs[0]].X;
         diff2 = ref->data[idxs[1]].Y - ref->data[idxs[0]].Y;
         diff3 = ref->data[idxs[1]].Z - ref->data[idxs[0]].Z;
         dist  = diff1*diff1 + diff2*diff2 + diff3*diff3;
         if ( dist < MAGIC_MIN_METERS_DIST ) continue;

         diff1 = ref->data[idxs[2]].X - ref->data[idxs[0]].X;
         diff2 = ref->data[idxs[2]].Y - ref->data[idxs[0]].Y;
         diff3 = ref->data[idxs[2]].Z - ref->data[idxs[0]].Z;
         dist  = diff1*diff1 + diff2*diff2 + diff3*diff3;
         if ( dist < MAGIC_MIN_METERS_DIST ) continue;

         diff1 = ref->data[idxs[2]].X - ref->data[idxs[1]].X;
         diff2 = ref->data[idxs[2]].Y - ref->data[idxs[1]].Y;
         diff3 = ref->data[idxs[2]].Z - ref->data[idxs[1]].Z;
         dist  = diff1*diff1 + diff2*diff2 + diff3*diff3;
         if ( dist < MAGIC_MIN_METERS_DIST ) continue;

         if (valid) found = 1;
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode
rox_pose_check_consensus(
  Rox_Uint                        *cardconsensus,
  Rox_Uint                        *inliers,
  Rox_Array2D_Double               pose,
  Rox_Array2D_Double               calib,
  Rox_DynVec_Point3D_Float  ref3D,
  Rox_DynVec_Point2D_Float  cur2D )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Point3D_Float  cref3D = NULL;
   Rox_Point2D_Float  cref2D = NULL;
   Rox_Uint countconsensus;
   Rox_Uint count_pairs;

   if (!cardconsensus || !pose || !calib || !ref3D || !cur2D || !inliers)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   count_pairs = ref3D->used;
   if (count_pairs == 0)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cref3D = (Rox_Point3D_Float ) rox_memory_allocate(sizeof(Rox_Point3D_Float_Struct), count_pairs);
   cref2D = (Rox_Point2D_Float ) rox_memory_allocate(sizeof(Rox_Point2D_Float_Struct), count_pairs);

   error = rox_point3d_float_transform(cref3D, pose, ref3D->data, count_pairs);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point2d_float_project(cref2D, cref3D, calib, count_pairs);
   ROX_ERROR_CHECK_TERMINATE ( error );

   countconsensus = 0;

   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      Rox_Double diffu = cref2D[i].u - cur2D->data[i].u;
      Rox_Double diffv = cref2D[i].v - cur2D->data[i].v;
      Rox_Double dist  = diffu*diffu + diffv*diffv;

      inliers[i] = 0;

      if ( dist < MIN_PIXELS_DIST_CONSENSUS )
      {
         inliers[i] = 1;
         countconsensus++;
      }
   }

   *cardconsensus = countconsensus;

function_terminate:
   rox_memory_delete(cref3D);
   rox_memory_delete(cref2D);

   return error;
}


Rox_ErrorCode
rox_points_build_inliers_subset(
  Rox_DynVec_Point3D_Float  refinliers,
  Rox_DynVec_Point2D_Float  curinliers,
  Rox_Uint                        *inliers,
  Rox_DynVec_Point2D_Float  cur2D,
  Rox_DynVec_Point3D_Float  ref3D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!refinliers || !curinliers || !inliers || !cur2D || !ref3D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint count = 0;
   Rox_Uint count_pairs = cur2D->used;
   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      if (inliers[i])
         count++;
   }

   if (count == 0)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   refinliers->used = 0;
   curinliers->used = 0;

   Rox_Uint pos = 0;
   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      if (!inliers[i]) continue;

      refinliers->data[pos] = ref3D->data[i];
      curinliers->data[pos] = cur2D->data[i];
      pos++;

      rox_dynvec_point3d_float_usecells(refinliers, 1);
      rox_dynvec_point2d_float_usecells(curinliers, 1);

      if (pos == count) break;
   }

function_terminate:
   return error;
}


// See RANSAC for Dummies by Marco Zuliani for equations and proofs of generic ransac
Rox_ErrorCode
rox_ransac_p3p(
  Rox_Array2D_Double              pose,
  Rox_DynVec_Point2D_Float inliers2D,
  Rox_DynVec_Point3D_Float inliers3D,
  Rox_DynVec_Point2D_Float cur2D,
  Rox_DynVec_Point3D_Float ref3D,
  Rox_Array2D_Double              calib )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //  Constants
   const Rox_Uint minsize_mss = 5;
   const Rox_Uint max_iterations = 1000;
   const Rox_Double log_probability_of_never_selecting_good_subset = log( 1e-2 );

   Rox_Uint                        idxs[3];
   Rox_Uint                        iter=0;
   Rox_Point3D_Double_Struct       localref[3];
   Rox_Point2D_Double_Struct       localcur[3];
   Rox_Array2D_Double              current_pose;
   Rox_Uint                        possible_count;
   Rox_Double                      **dx=NULL;
   Rox_Uint                        card_consensus;
   Rox_Uint                        max_consensus=3;
   Rox_Uint                        ransac_max_iterations=max_iterations;
   Rox_Double                        q, p_q;
   Rox_Uint                       *cur_inliers=NULL;
   Rox_Uint                       *best_inliers=NULL;
   Rox_Uint                       *pool=NULL;
   Rox_Array2D_Double_Collection   possible_poses=NULL;

   // Input check
   if ( !pose || !cur2D || !ref3D || !calib )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint count_pairs = cur2D->used;

   if ( count_pairs != ref3D->used ) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if ( count_pairs < minsize_mss )  { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_point2d_float_reset( inliers2D );
   rox_dynvec_point3d_float_reset( inliers3D );

   error = rox_array2d_double_check_size( pose, 4, 4 );   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( calib, 3, 3 );  ROX_ERROR_CHECK_TERMINATE(error)

   // Create buffers
   cur_inliers = ( Rox_Uint * ) rox_memory_allocate( sizeof( Rox_Uint ), count_pairs );
   if ( cur_inliers == NULL ) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   best_inliers = ( Rox_Uint * ) rox_memory_allocate( sizeof( Rox_Uint ), count_pairs );
   if ( best_inliers == NULL ) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   pool = ( Rox_Uint * ) rox_memory_allocate( sizeof( Rox_Uint ), count_pairs );
   if ( pool == NULL ) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_array2d_double_collection_new( &possible_poses, 4, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dx, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Initialize random pool
   for (Rox_Uint i = 0; i < count_pairs; i++ )
   {
      best_inliers[i] = 0;
      pool[i] = i;
   }

   //  Trials loop
   while ( iter < max_iterations && iter < ransac_max_iterations )
   {
      // Update iter first, for "continue" sake
      iter++;

      // Select a subset
      error = rox_match_double_select_random_3points( idxs, ref3D, cur2D, pool );
      if (error) continue;

      //  Buffer with subset
      POINT3D_FLOAT_TO_DOUBLE( localref[0], ref3D->data[idxs[0]] );
      POINT3D_FLOAT_TO_DOUBLE( localref[1], ref3D->data[idxs[1]] );
      POINT3D_FLOAT_TO_DOUBLE( localref[2], ref3D->data[idxs[2]] );
      POINT2D_FLOAT_TO_DOUBLE( localcur[0], cur2D->data[idxs[0]] );
      POINT2D_FLOAT_TO_DOUBLE( localcur[1], cur2D->data[idxs[1]] );
      POINT2D_FLOAT_TO_DOUBLE( localcur[2], cur2D->data[idxs[2]] );

      //  Compute coarse pose given subset
      error = rox_pose_from_3_points( possible_poses,
                                     &possible_count,
                                      localref,
                                      localcur,
                                      dx[0][0],
                                      dx[1][1],
                                      dx[0][2],
                                      dx[1][2] );
      if (error) continue;

      //  Check poses
      for (Rox_Uint id_pose = 0; id_pose < possible_count; id_pose++ )
      {
         current_pose = rox_array2d_double_collection_get( possible_poses, id_pose );
         error = rox_pose_check_consensus( &card_consensus, cur_inliers, current_pose, calib, ref3D, cur2D );
         if (error) continue;

         // If score for current pose is better than ever, keep it
         if ( card_consensus > max_consensus )
         {
            // Update pose
            max_consensus = card_consensus;
            rox_array2d_double_copy( pose, current_pose );
            memcpy( best_inliers, cur_inliers, sizeof( Rox_Uint ) * count_pairs );

            // Ransac update
            q = pow( ( ( double ) max_consensus ) / ( ( double ) count_pairs ), ( int )minsize_mss );
            p_q = 1.0 - q;

            if ( p_q < DBL_EPSILON )
            {
               ransac_max_iterations = 0;
            }
            else
            {
               ransac_max_iterations = ( int )( 0.5 + ( log_probability_of_never_selecting_good_subset / log( p_q ) ) );
            }
         }
      }
   }

   error = rox_points_build_inliers_subset( inliers3D, inliers2D, best_inliers, cur2D, ref3D );
   ROX_ERROR_CHECK_TERMINATE(error)

   //  Minimal consensus check for validity
   if ( max_consensus <= 4 )
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_collection_del( &possible_poses );
   rox_memory_delete( cur_inliers );
   rox_memory_delete( best_inliers );
   rox_memory_delete( pool );

   return error;
}


//------------------------------------------------------------------------------
//--- Double stuff
//------------------------------------------------------------------------------
Rox_ErrorCode
rox_match_double_select_random_3points_double(
  Rox_Uint                         *idxs,
  Rox_DynVec_Point3D_Double  ref,
  Rox_DynVec_Point2D_Double  cur,
  Rox_Uint                         *pool )
{
   Rox_ErrorCode error=ROX_ERROR_NONE;
   Rox_Uint      card;
   int           poolsize;
   int           found=0;
   int           curpool,idpool;
   int           nbfound;
   int           iters;
   int           valid;
   const int     maxiters=200;
   Rox_Double      diff1, diff2, diff3;
   Rox_Double      dist;

   card     = ref->used;
   poolsize = ref->used;
   iters    = 0;
   nbfound  = 0;

   while (!found)
   {
      iters++;
      if (iters == maxiters)
      { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error );}

      // Throw dice for one point
      curpool = rox_rand() % poolsize;

      // Update possible points
      idpool             = pool[curpool];
      pool[curpool]      = pool[poolsize - 1];
      pool[poolsize - 1] = idpool;
      poolsize--;

      //  store one index
      idxs[nbfound] = idpool;
      nbfound++;

      // If three points found, ready for pose estimation if ...
      if (nbfound == 3)
      {
         valid   = 1;
         nbfound = 0;

         // Eventually reset poolsize in case we continue
         poolsize = card;

         diff1 = cur->data[idxs[1]].u - cur->data[idxs[0]].u;
         diff2 = cur->data[idxs[1]].v - cur->data[idxs[0]].v;
         dist  = diff1*diff1 + diff2*diff2;
         if ( dist < MAGIC_MIN_PIXELS_DIST ) continue;

         diff1 = cur->data[idxs[2]].u - cur->data[idxs[0]].u;
         diff2 = cur->data[idxs[2]].v - cur->data[idxs[0]].v;
         dist  = diff1*diff1 + diff2*diff2;
         if ( dist < MAGIC_MIN_PIXELS_DIST ) continue;

         diff1 = cur->data[idxs[2]].u - cur->data[idxs[1]].u;
         diff2 = cur->data[idxs[2]].v - cur->data[idxs[1]].v;
         dist  = diff1*diff1 + diff2*diff2;
         if ( dist < MAGIC_MIN_PIXELS_DIST ) continue;

         diff1 = ref->data[idxs[1]].X - ref->data[idxs[0]].X;
         diff2 = ref->data[idxs[1]].Y - ref->data[idxs[0]].Y;
         diff3 = ref->data[idxs[1]].Z - ref->data[idxs[0]].Z;
         dist  = diff1*diff1 + diff2*diff2 + diff3*diff3;
         if ( dist < MAGIC_MIN_METERS_DIST ) continue;

         diff1 = ref->data[idxs[2]].X - ref->data[idxs[0]].X;
         diff2 = ref->data[idxs[2]].Y - ref->data[idxs[0]].Y;
         diff3 = ref->data[idxs[2]].Z - ref->data[idxs[0]].Z;
         dist  = diff1*diff1 + diff2*diff2 + diff3*diff3;
         if ( dist < MAGIC_MIN_METERS_DIST ) continue;

         diff1 = ref->data[idxs[2]].X - ref->data[idxs[1]].X;
         diff2 = ref->data[idxs[2]].Y - ref->data[idxs[1]].Y;
         diff3 = ref->data[idxs[2]].Z - ref->data[idxs[1]].Z;
         dist  = diff1*diff1 + diff2*diff2 + diff3*diff3;
         if ( dist < MAGIC_MIN_METERS_DIST ) continue;

         if (valid) found = 1;
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode
rox_pose_check_consensus_double(
  Rox_Uint                         *cardconsensus,
  Rox_Uint                         *inliers,
  Rox_Array2D_Double                pose,
  Rox_Array2D_Double                calib,
  Rox_DynVec_Point3D_Double  ref3D,
  Rox_DynVec_Point2D_Double  cur2D )
{
   Rox_ErrorCode              error=ROX_ERROR_NONE;
   Rox_Point3D_Double cref3D=NULL;
   Rox_Point2D_Double cref2D=NULL;
   Rox_Uint                   countconsensus;
   Rox_Uint                   count_pairs;

   if (!cardconsensus || !pose || !calib || !ref3D || !cur2D || !inliers)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_double_check_size(pose, 4, 4);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_check_size(calib, 3, 3);
   ROX_ERROR_CHECK_TERMINATE ( error );

   count_pairs = ref3D->used;
   if (count_pairs == 0)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   cref3D = (Rox_Point3D_Double ) rox_memory_allocate(sizeof(Rox_Point3D_Double_Struct), count_pairs);
   cref2D = (Rox_Point2D_Double ) rox_memory_allocate(sizeof(Rox_Point2D_Double_Struct), count_pairs);

   error = rox_point3d_double_transform(cref3D, pose, ref3D->data, count_pairs);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_point2d_double_project(cref2D, cref3D, calib, count_pairs);
   ROX_ERROR_CHECK_TERMINATE ( error );

   countconsensus = 0;

   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      Rox_Double diffu = cref2D[i].u - cur2D->data[i].u;
      Rox_Double diffv = cref2D[i].v - cur2D->data[i].v;
      Rox_Double dist  = diffu*diffu + diffv*diffv;

      inliers[i] = 0;

      if ( dist < MIN_PIXELS_DIST_CONSENSUS )
      {
         inliers[i] = 1;
         countconsensus++;
      }
   }

   *cardconsensus = countconsensus;

function_terminate:
   rox_memory_delete(cref3D);
   rox_memory_delete(cref2D);

   return error;
}


Rox_ErrorCode
rox_points_build_inliers_subset_double(
  Rox_DynVec_Point3D_Double  refinliers,
  Rox_DynVec_Point2D_Double  curinliers,
  Rox_Uint                         *inliers,
  Rox_DynVec_Point2D_Double  cur2D,
  Rox_DynVec_Point3D_Double  ref3D)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!refinliers || !curinliers || !inliers || !cur2D || !ref3D)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint count = 0;
   Rox_Uint count_pairs = cur2D->used;
   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      if (inliers[i])
         count++;
   }

   if (count == 0)
   { error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   refinliers->used = 0;
   curinliers->used = 0;

   Rox_Uint pos = 0;
   for (Rox_Uint i = 0; i < count_pairs; i++)
   {
      if (!inliers[i]) continue;

      refinliers->data[pos] = ref3D->data[i];
      curinliers->data[pos] = cur2D->data[i];
      pos++;

      rox_dynvec_point3d_double_usecells(refinliers, 1);
      rox_dynvec_point2d_double_usecells(curinliers, 1);

      if (pos == count) break;
   }

function_terminate:
   return error;
}


// See RANSAC for Dummies by Marco Zuliani for equations and proofs of generic ransac
Rox_ErrorCode
rox_ransac_p3p_double(
  Rox_Array2D_Double               pose,
  Rox_DynVec_Point2D_Double inliers2D,
  Rox_DynVec_Point3D_Double inliers3D,
  Rox_DynVec_Point2D_Double cur2D,
  Rox_DynVec_Point3D_Double ref3D,
  Rox_Array2D_Double               calib )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   //  Constants
   const Rox_Uint minsize_mss = 5;
   const Rox_Uint max_iterations = 1000;
   const Rox_Double log_probability_of_never_selecting_good_subset = log( 1e-2 );

   Rox_Uint                        idxs[3];
   Rox_Uint                        iter=0;
   Rox_Point3D_Double_Struct       localref[3];
   Rox_Point2D_Double_Struct       localcur[3];
   Rox_Array2D_Double              current_pose;
   Rox_Uint                        possible_count;
   Rox_Double                      **dx=NULL;
   Rox_Uint                        card_consensus;
   Rox_Uint                        max_consensus=3;
   Rox_Uint                        ransac_max_iterations=max_iterations;
   Rox_Double                        q, p_q;
   Rox_Uint                       *cur_inliers=NULL;
   Rox_Uint                       *best_inliers=NULL;
   Rox_Uint                       *pool=NULL;
   Rox_Array2D_Double_Collection   possible_poses=NULL;

   // Input check
   if ( !pose || !cur2D || !ref3D || !calib )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint count_pairs = cur2D->used;

   if ( count_pairs != ref3D->used ) { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   if ( count_pairs < minsize_mss )  { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //rox_log( "=============\n" );
   //rox_log( "ransac_inputs\n" );
   //rox_array2d_double_print( pose );
   //rox_array2d_double_print( calib );
   //for ( int ii = 0; ii < cur2D->used; ii++ )
   //{
   //   rox_log( "( %f, %f ) <-> ( %f, %f, %f )\n",
   //           cur2D->data[ ii ].u, cur2D->data[ ii ].v,
   //           ref3D->data[ ii ].X, ref3D->data[ ii ].Y, ref3D->data[ ii ].Z );
   //}


   rox_dynvec_point2d_double_reset( inliers2D );
   rox_dynvec_point3d_double_reset( inliers3D );

   error = rox_array2d_double_check_size( pose, 4, 4 );   ROX_ERROR_CHECK_TERMINATE(error)
   error = rox_array2d_double_check_size( calib, 3, 3 );  ROX_ERROR_CHECK_TERMINATE(error)

   // Create buffers
   cur_inliers = ( Rox_Uint * ) rox_memory_allocate( sizeof( Rox_Uint ), count_pairs );
   if ( cur_inliers == NULL ) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   best_inliers = ( Rox_Uint * ) rox_memory_allocate( sizeof( Rox_Uint ), count_pairs );
   if ( best_inliers == NULL ) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   pool = ( Rox_Uint * ) rox_memory_allocate( sizeof( Rox_Uint ), count_pairs );
   if ( pool == NULL ) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error );}

   error = rox_array2d_double_collection_new( &possible_poses, 4, 4, 4 );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_double_get_data_pointer_to_pointer( &dx, calib );
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Initialize random pool
   for (Rox_Uint i = 0; i < count_pairs; i++ )
   {
      best_inliers[i] = 0;
      pool[i] = i;
   }

   // Initialize rand
   rox_srand( 0 );

   while ( iter < max_iterations && iter < ransac_max_iterations )
   {
      // Update iter first, for "continue" sake
      iter++;

      // Select a subset
      error = rox_match_double_select_random_3points_double( idxs, ref3D, cur2D, pool );
      if (error) continue;

      //  Buffer with subset
      localref[0] = ref3D->data[idxs[0]];
      localref[1] = ref3D->data[idxs[1]];
      localref[2] = ref3D->data[idxs[2]];
      localcur[0] = cur2D->data[idxs[0]];
      localcur[1] = cur2D->data[idxs[1]];
      localcur[2] = cur2D->data[idxs[2]];

      //  Compute coarse pose given subset
      error = rox_pose_from_3_points ( possible_poses, &possible_count, localref, localcur, dx[0][0], dx[1][1], dx[0][2], dx[1][2] );
      if (error) continue;

      //  Check poses
      for (Rox_Uint id_pose = 0; id_pose < possible_count; id_pose++ )
      {
         current_pose = rox_array2d_double_collection_get( possible_poses, id_pose );
         error = rox_pose_check_consensus_double( &card_consensus, cur_inliers, current_pose, calib, ref3D, cur2D );
         if (error) continue;

         // If score for current pose is better than ever, keep it
         if ( card_consensus > max_consensus )
         {
            // Update pose
            max_consensus = card_consensus;
            rox_array2d_double_copy( pose, current_pose );
            memcpy( best_inliers, cur_inliers, sizeof( Rox_Uint ) * count_pairs );

            // Ransac update
            q = pow( ( ( double ) max_consensus ) / ( ( double ) count_pairs ), ( int )minsize_mss );
            p_q = 1.0 - q;

            if ( p_q < DBL_EPSILON )
            {
               ransac_max_iterations = 0;
            }
            else
            {
               ransac_max_iterations = ( int )( 0.5 + ( log_probability_of_never_selecting_good_subset / log( p_q ) ) );
            }
         }
      }
   }

   error = rox_points_build_inliers_subset_double( inliers3D, inliers2D, best_inliers, cur2D, ref3D );
   ROX_ERROR_CHECK_TERMINATE(error)

   //  Minimal consensus check for validity
   if ( max_consensus <= 4 )
   {
      error = ROX_ERROR_NUMERICAL_ALGORITHM_FAILURE;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

function_terminate:
   rox_array2d_double_collection_del( &possible_poses );
   rox_memory_delete( cur_inliers );
   rox_memory_delete( best_inliers );
   rox_memory_delete( pool );

   return error;
}
