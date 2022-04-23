//==============================================================================
//
//    OPENROX   : File sraid_matchset.c
//
//    Contents  : Implementation of sraid_matchset module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sraid_matchset.h"

#include <float.h>
#include <limits.h>

#include <generated/array2d_double.h>
#include <generated/dynvec_sint_struct.h>
#include <generated/dynvec_sraiddesc_struct.h>

#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/measures/distance_point_to_line.h>
#include <baseproc/maths/maths_macros.h>

#include <inout/system/print.h>
#include <inout/system/errors_print.h>



#define MAGIC_THRESH_RATIO_SND_BEST_MATCH 1.0



Rox_ErrorCode rox_sraid_matchset(
  Rox_DynVec_Sint          matchassocs,
  Rox_DynVec_SRAID_Feature feat1,
  Rox_DynVec_SRAID_Feature feat2 )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   const Rox_Double thresh = 0.7 * 0.7;

   if ( !matchassocs || !feat1 || !feat2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   matchassocs->used = 0;
   rox_dynvec_sint_usecells( matchassocs, feat1->used );

   for (Rox_Uint i = 0; i < feat1->used; i++ )
   {
      int idmin = -1;
      Rox_Uint prevminval = ~0;
      Rox_Uint minval = ~0;
      Rox_Uint val;

      // Loop over second set for match
      for (Rox_Uint j = 0; j < feat2->used; j++ )
      {
         val = rox_sraid_match( feat1->data[i].descriptor, feat2->data[j].descriptor );

         if ( val < minval )
         {
            prevminval = minval;
            idmin = j;
            minval = val;
         }
         else if ( val < prevminval )
         {
            prevminval = val;
         }
      }

      //Is is far enough from second putative match ?
      if ( ( (Rox_Float) minval ) < ( (Rox_Float) prevminval ) * thresh ) // corresponds to ratio = 0.7 in rox_dev
      {
         matchassocs->data[i] = idmin;
      }
      else
      {
         matchassocs->data[i] = -1;
      }
   }

function_terminate:
   return error;
}


Rox_ErrorCode rox_sraid_matchset_exclusive(
  Rox_DynVec_Sint           matchassocs,
  Rox_Sint                 *matchcount,
  Rox_DynVec_Sint           matchdists,
  Rox_DynVec_SRAID_Feature  feat1,
  Rox_DynVec_SRAID_Feature  feat2 )
{
   Rox_ErrorCode   error=ROX_ERROR_NONE;
   Rox_DynVec_Sint best_feat2=NULL;
   Rox_DynVec_Sint best_feat1=NULL;
   Rox_DynVec_Sint val_feat1=NULL;

   if ( NULL == matchassocs || NULL == matchdists || NULL == feat1 || NULL == feat2 )
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE ( error );
   }

   *matchcount       = 0;
   matchassocs->used = 0;
   matchdists->used  = 0;

   #ifdef HEAVY_DEBUG
      rox_log( "feat1->used : %d \n", feat1->used );
      for ( int ii = 0; ii < 10; ii++ )
      {
         for ( int jj = 0; jj < 128; jj++ )
            rox_log( "%d , ", feat1->data[ ii ].descriptor[ jj ] );

         rox_log( "\n" );
      }

      rox_log( "feat2->used : %d \n", feat2->used );
      for ( int ii = 0; ii < 10; ii++ )
      {
         for ( int jj = 0; jj < 128; jj++ )
            rox_log( "%d , ", feat2->data[ ii ].descriptor[ jj ] );

         rox_log( "\n" );
      }
      rox_log( "%s %d \n", __FUNCTION__, __LINE__ );
   #endif

   error = rox_dynvec_sint_new( &best_feat2, feat1->used + 1 );        
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_dynvec_sint_new( &best_feat1, feat2->used + 1 );        
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_dynvec_sint_new( &val_feat1,  feat2->used + 1 );        
   ROX_ERROR_CHECK_TERMINATE ( error );

   rox_dynvec_sint_usecells( best_feat1, feat2->used );
   rox_dynvec_sint_usecells( val_feat1,  feat2->used );
   for (Rox_Uint ii = 0; ii < feat2->used; ii++ )
   {
      best_feat1->data[ ii ] = -1;
      val_feat1->data[ ii ]  = INT_MAX;
   }

   best_feat2->used = 0;
   rox_dynvec_sint_usecells( best_feat2, feat1->used );

   for (Rox_Uint ii = 0; ii < feat1->used; ii++ )
   {
      int      idmin   = 0;
      Rox_Uint min_val = UINT_MAX;
      Rox_Uint snd_val = UINT_MAX;
      Rox_Uint val;

      // Loop over second set for match
      for (Rox_Uint jj = 0; jj < feat2->used; jj++ )
      {
         val = rox_sraid_match( feat1->data[ii].descriptor, feat2->data[jj].descriptor );

         if ( val < min_val )
         {
            snd_val = min_val;
            min_val = val;
            idmin   = jj;
         }
         else if ( val < snd_val )
         {
            snd_val = val;
         }
      }

      // Features1->Features2
      // Minimum value must be sufficiently smaller than the second minimum
      if ( ( (Rox_Float) min_val ) < ( ((Rox_Float) snd_val) * MAGIC_THRESH_RATIO_SND_BEST_MATCH ) )
      {
         best_feat2->data[ ii ] = idmin;
         matchdists->data[ ii ] = (Rox_Sint) min_val;
      }
      else
      {
         best_feat2->data[ii] = -1;
      }

      // Features2->Features1
      // Current minimum value must be lower than the minimum already found
      if ( ( (Rox_Float) min_val ) < ( (Rox_Float) val_feat1->data[ idmin ] ) ) // <= not exactly symetric since no threshold
      {
         best_feat1->data[ idmin ] = ii;
         val_feat1->data[ idmin ]  = min_val;
      }
   }

   for (Rox_Uint ii = 0; ii < feat1->used; ii++ )
   {
      Rox_Sint candidate = best_feat2->data[ ii ];

      if ( ( -1 != candidate ) && ( ii == best_feat1->data[ candidate ] ) )
      {
         matchassocs->data[ ii ] = candidate;
         // matchdists was already filled
         (*matchcount)++;
      }
      else
      {
         matchassocs->data[ ii ] = -1;
      }
   }

   rox_dynvec_sint_usecells( matchassocs, feat1->used );
   rox_dynvec_sint_usecells( matchdists,  feat1->used );

   #ifdef HEAVY_DEBUG
      rox_log("\n");
      rox_log( "matchassocs->used : %d \n", matchassocs->used );
      for ( int ii = 0; ii < matchassocs->used; ii++ )
      {
         rox_log( "%d , ", matchassocs->data[ ii ] );
      }
      rox_log( "\n" );
      rox_log( "%s %d \n", __FUNCTION__, __LINE__ );
   #endif

function_terminate:
   if ( NULL != val_feat1  ) rox_dynvec_sint_del( &val_feat1  );
   if ( NULL != best_feat1 ) rox_dynvec_sint_del( &best_feat1 );
   if ( NULL != best_feat2 ) rox_dynvec_sint_del( &best_feat2 );

   return error;
}


Rox_ErrorCode rox_sraid_matchset_constrained ( 
   Rox_DynVec_Sint matchassocs, 
   Rox_DynVec_SRAID_Feature feat1, 
   Rox_DynVec_SRAID_Feature feat2 
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   const Rox_Double thresh = 0.7 * 0.7;
   Rox_Double ** F = NULL;
   Rox_Uint i = 0;
   Rox_Uint j = 0;

   // Hack
   Rox_Array2D_Double fundamental = NULL;

   error = rox_array2d_double_new( &fundamental, 3, 3 );
   error = rox_array2d_double_get_data_pointer_to_pointer(&F, fundamental);

   F[0][0] = -0.0000000000418285; F[0][1] =  0.0000000092036494; F[0][2] = -0.0000030349203516;
   F[1][0] = -0.0000000110357279; F[1][1] =  0.0000000007736746; F[1][2] =  0.0000906624917869;
   F[2][0] =  0.0000036997174777; F[2][1] = -0.0000905704056729; F[2][2] =  0.0027682192809068;

   if ( !matchassocs || !feat1 || !feat2 )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   matchassocs->used = 0;
   rox_dynvec_sint_usecells( matchassocs, feat1->used );
   
   for ( i = 0; i < feat1->used; i++ ) // feat 1 -> cur
   {
      int idmin = -1;
      Rox_Uint prevminval = ~0;
      Rox_Uint minval = ~0;
      Rox_Uint val = ~0;

      // Loop over second set for match
      for ( j = 0; j < feat2->used; j++ ) // feat 2 -> ref
      {
         Rox_Float distance = 0.0;
         Rox_Point2D_Float_Struct point_ref;
         Rox_Point2D_Float_Struct point_cur;

         point_cur.u = feat1->data[i].x;
         point_cur.v = feat1->data[i].y;

         point_ref.u = feat2->data[j].x;
         point_ref.v = feat2->data[j].y;

         rox_distance_point2d_to_epipolar_line( &distance, &point_ref, &point_cur, fundamental );

         if( distance > 0.0 ) // 1.0 pixels threshold
         {
            val = rox_sraid_match( feat1->data[i].descriptor, feat2->data[j].descriptor );

            if ( val < minval )
            {
               prevminval = minval;
               idmin = j;
               minval = val;
            }
            else if ( val < prevminval )
            {
               prevminval = val;
            }
         }
      }

      // Is is far enough from second putative match ?
      if ( ( (Rox_Float) minval ) < ( (Rox_Float) prevminval ) * thresh )    // corresponds to ratio = 0.7 in rox_dev
      {
         matchassocs->data[i] = idmin;
      }
      else
      {
         matchassocs->data[i] = -1;
      }
   }

function_terminate:
   return error;
}
