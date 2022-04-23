//==============================================================================
//
//    OPENROX   : File fastst_score_sse.h
//
//    Contents  : API of fastst_score module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fastst_score.h"

#include <baseproc/maths/maths_macros.h>
#include <generated/dynvec_segment_point_struct.h>
#include <inout/system/errors_print.h>

#define CIRCLE_SIZE 16
#define SEGMENT_SIZE 9
#define TAB_SIZE CIRCLE_SIZE + SEGMENT_SIZE
#define Compare(X, Y) ((X)>=(Y))

int rox_ansi_fastst_detector_score_sse ( short * score, Rox_Sshort d[TAB_SIZE] )
{
   int error = 0;
      __m128i q0 = _mm_set1_epi16(-1000);
      __m128i q1 = _mm_set1_epi16(1000);

      for ( Rox_Sint k = 0; k < 16; k += 8 )
      {
         __m128i v0 = _mm_loadu_si128((__m128i*)(d + k + 1));
         __m128i v1 = _mm_loadu_si128((__m128i*)(d + k + 2));

         __m128i a = _mm_min_epi16(v0, v1);
         __m128i b = _mm_max_epi16(v0, v1);

         v0 = _mm_loadu_si128((__m128i*)(d + k + 3));
         a = _mm_min_epi16(a, v0);
         b = _mm_max_epi16(b, v0);

         v0 = _mm_loadu_si128((__m128i*)(d + k + 4));
         a = _mm_min_epi16(a, v0);
         b = _mm_max_epi16(b, v0);

         v0 = _mm_loadu_si128((__m128i*)(d + k + 5));
         a = _mm_min_epi16(a, v0);
         b = _mm_max_epi16(b, v0);

         v0 = _mm_loadu_si128((__m128i*)(d + k + 6));
         a = _mm_min_epi16(a, v0);
         b = _mm_max_epi16(b, v0);

         v0 = _mm_loadu_si128((__m128i*)(d + k + 7));
         a = _mm_min_epi16(a, v0);
         b = _mm_max_epi16(b, v0);

         v0 = _mm_loadu_si128((__m128i*)(d + k + 8));
      a = _mm_min_epi16(a, v0);
      b = _mm_max_epi16(b, v0);

      v0 = _mm_loadu_si128((__m128i*)(d + k));
      q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
      q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));

      v0 = _mm_loadu_si128((__m128i*)(d + k + 9));
      q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
      q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
   }

   q0 = _mm_max_epi16(q0, _mm_sub_epi16(_mm_setzero_si128(), q1));
   q0 = _mm_max_epi16(q0, _mm_unpackhi_epi64(q0, q0));
   q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 4));
   q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 2));

   *score = (short) _mm_cvtsi128_si32(q0);

   return error;
}

Rox_ErrorCode rox_fastst_detector_score (
   Rox_DynVec_Segment_Point points, 
   Rox_Image source, 
   Rox_Sint barrier
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sshort d[TAB_SIZE];

   if (!points || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uchar ** in = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &in, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stride = 0;
   error = rox_array2d_uchar_get_stride(&stride, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // We assume points contains only valid corner coordinates

   int tabs[] = {
            0 + stride * 3, 1 + stride * 3, 2 + stride * 2, 3 + stride * 1,
            3 + stride * 0, 3 + stride * -1, 2 + stride * -2, 1 + stride * -3,
            0 + stride * -3, -1 + stride * -3, -2 + stride * -2, -3 + stride * -1,
            -3 + stride * 0, -3 + stride * 1, -2 + stride * 2, -1 + stride * 3,
            0 + stride * 3, 1 + stride * 3, 2 + stride * 2, 3 + stride * 1,
            3 + stride * 0, 3 + stride * -1, 2 + stride * -2, 1 + stride * -3,
            0 + stride * -3, -1 + stride * -3, -2 + stride * -2, -3 + stride * -1
   };

   for (Rox_Uint idpt = 0; idpt < points->used; idpt++)
   {
      short score = 0;
      Rox_Sint j = points->data[idpt].j;
      Rox_Sint i = points->data[idpt].i;
      Rox_Uchar * ptr = &in[i][j];
      Rox_Sint v = ptr[0];

      // Compute differences on whole circle + segment size
      for ( Rox_Sint k = 0; k < TAB_SIZE; k++ )
      {
         d[k] = v - ptr[tabs[k]];
      }

      error = rox_ansi_fastst_detector_score_sse ( &score, d );
      ROX_ERROR_CHECK_TERMINATE ( error );

      points->data[idpt].score = score - 1;
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_fastst_nonmax_suppression (
   Rox_DynVec_Segment_Point ptr, 
   Rox_DynVec_Segment_Point corners
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint last_row;
   int* row_start;
   Rox_Uint i, j;
   const Rox_Uint sz = corners->used;
   Rox_Uint point_above = 0;
   Rox_Uint point_below = 0;
   int prev_row = -1;

   if (!ptr || !corners) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   
   if (corners->used < 1) return ROX_ERROR_NONE;
   rox_dynvec_segment_point_reset(ptr);

   last_row = corners->data[sz-1].i;
   row_start = (int*)rox_memory_allocate(sizeof(int), last_row + 1);
   if (!row_start) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   // Build Row look up table
   for(i=0; i < last_row+1; i++) row_start[i] = -1;
      
   for(i=0; i < sz; i++)
   {
      if(((int)corners->data[i].i) != prev_row)
      {
         row_start[corners->data[i].i] = i;
         prev_row = corners->data[i].i;
      }
   }

   for(i=0; i < sz; i++)
   {
      Rox_Segment_Point_Struct pos = corners->data[i];
      Rox_Float score = (Rox_Float) pos.score;

      // Check left
      if(i > 0)
      {
         if(corners->data[i-1].j == pos.j-1 && corners->data[i-1].i == pos.i && Compare(corners->data[i-1].score, score)) continue;
      }

      // Check right
      if(i < (sz - 1))
      {
         if(corners->data[i+1].j == pos.j+1 && corners->data[i+1].i == pos.i && Compare(corners->data[i+1].score, score)) continue;
      }

      // Check above (if there is a valid row above)
      if (pos.i != 0 && row_start[pos.i - 1] != -1)
      {
         // Make sure that current point_above is one row above.
         if (corners->data[point_above].i < pos.i - 1) point_above = row_start[pos.i-1];

         // Make point_above point to the first of the pixels above the current point, if it exists.
         for (; corners->data[point_above].i < pos.i && corners->data[point_above].j < pos.j - 1; point_above++)
         {
         }

         // Check if a point above is close
         for(j=point_above; corners->data[j].i < pos.i && corners->data[j].j <= pos.j + 1; j++)
         {
            int x = corners->data[j].j;
            if (((x == (int) pos.j - 1) || (x == (int) pos.j) || (x == (int) pos.j+1)) && Compare(corners->data[j].score, score))
            {
               goto cont;
            }
         }
      }

      // Check below (if there is anything below)
      if(pos.i != last_row && row_start[pos.i + 1] != -1 && point_below < sz) // Nothing below
      {
         if(corners->data[point_below].i < pos.i + 1) point_below = row_start[pos.i+1];

         //  Make point below point to one of the pixels belowthe current point, if it exists.
         for (; point_below < sz && corners->data[point_below].i == pos.i+1 && corners->data[point_below].j < pos.j - 1; point_below++)
         {
         }

         // Check if a point below is close
         for(j = point_below; j < sz && corners->data[j].i == pos.i+1 && corners->data[j].j <= pos.j + 1; j++)
         {
            int x = corners->data[j].j;
            if (((x == (int)pos.j - 1) || (x == (int) pos.j) || (x == (int)pos.j+1)) && Compare(corners->data[j].score,score))
            {
               goto cont;
            }
         }
      }

      rox_dynvec_segment_point_append(ptr, &corners->data[i]);

      cont:
      ;
   }

   rox_memory_delete(row_start);

function_terminate:
   return error;
}

int comparePoints(const Rox_Segment_Point  c1, const Rox_Segment_Point  c2)
{
   if (c1->score > c2->score) return -1;
   if (c1->score < c2->score) return 1;
   return 0;
}

Rox_ErrorCode rox_fastst_detector_sort ( Rox_DynVec_Segment_Point ptr )
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!ptr) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   if (ptr->used == 0) return ROX_ERROR_NONE;

   qsort(ptr->data, ptr->used, sizeof(Rox_Segment_Point_Struct), (int (*)(const void *, const void *)) comparePoints);

function_terminate:
   return error;
}
