//==============================================================================
//
//    OPENROX   : File fastst_sse.h
//
//    Contents  : API of fastst module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "fastst.h"

#include "segmentpoint_tools.h"
#include <core/features/detectors/segment/segmentpoint_struct.h>
#include <inout/system/errors_print.h>

#define CHECKER(RES,LO,HI,CIRCLEID)\
      circle = _mm_loadu_si128((__m128i *)(ptr + tabs[CIRCLEID]));\
      diff = _mm_subs_epu8(LO, circle);\
      diff2 = _mm_subs_epu8(circle, HI);\
      z = _mm_setzero_si128();\
      diff = _mm_cmpeq_epi8(diff, z);\
      diff2 = _mm_cmpeq_epi8(diff2, z);\
      RES = ~(_mm_movemask_epi8(diff) | (_mm_movemask_epi8(diff2) << 16));\

#define TOADD(I,J)\
{\
   toadd.i = I;\
    toadd.j = J;\
    toadd.score = 0;\
    toadd.level = lvl;\
    rox_dynvec_segment_point_append(points, &toadd);\
}

int rox_ansi_check_possible_sse ( int * possible, int barrier, int tabs[16], unsigned char * ptr )
{
   int error = 0;

   int ans_6_7;
   int ans_14_15;
   int ans_1_2;
   int ans_9_10;
   int ans_15_0;
   int ans_7_8;
   int ans_12_13;
   int ans_4_5;
   int ans_2_3;
   int ans_11_12;
   int ans_3_4;
   int ans_10_11;
   int ans_0;
   int ans_1;
   int ans_2;
   int ans_3;
   int ans_4;
   int ans_5;
   int ans_6;
   int ans_7;
   int ans_8;
   int ans_9;
   int ans_10;
   int ans_11;
   int ans_12;
   int ans_13;
   int ans_14;
   int ans_15;

   __m128i diff, diff2, z, circle;
   register const __m128i vbarrier = _mm_set1_epi8( (Rox_Uchar) barrier );

   __m128i nucleus = _mm_loadu_si128((__m128i *) ptr);

   __m128i low  = _mm_subs_epu8(nucleus, vbarrier);
   __m128i high = _mm_adds_epu8(nucleus, vbarrier);

   CHECKER(ans_0, low, high, 0);
   CHECKER(ans_8, low, high, 8);

   *possible = ans_0 | ans_8;
   if (!*possible) goto function_terminate;

   CHECKER(ans_15, low, high, 15);
   CHECKER(ans_1, low, high, 1);

   *possible &= ans_8 | (ans_15 & ans_1);
   if (!*possible) goto function_terminate;

   CHECKER(ans_7, low, high, 7);
   CHECKER(ans_9, low, high, 9);

   *possible &= ans_9 | (ans_0 & ans_1);
   *possible &= ans_7 | (ans_15 & ans_0);
   if (!*possible) goto function_terminate;

   CHECKER(ans_12, low, high, 12);
   CHECKER(ans_4, low, high, 4);

   *possible &= ans_12 | (ans_4 & (ans_1 | ans_7));
   *possible &= ans_4 | (ans_12 & (ans_9 | ans_15));
   if (!*possible) goto function_terminate;

   CHECKER(ans_14, low, high, 14);
   CHECKER(ans_6, low, high, 6);

   ans_6_7 = ans_6 & ans_7;
   *possible &= ans_14 | (ans_6_7 & (ans_4 | (ans_8 & ans_9)));
   *possible &= ans_1 | (ans_6_7) | ans_12;
   ans_14_15 = ans_14 & ans_15;
   *possible &= ans_6 | (ans_14_15 & (ans_12 | (ans_0 & ans_1)));
   *possible &= ans_9 | (ans_14_15) | ans_4;

   if (!*possible) goto function_terminate;

   CHECKER(ans_2, low, high, 2);
   CHECKER(ans_10, low, high, 10);

   ans_1_2 = ans_1 & ans_2;
   *possible &= ans_10 | (ans_1_2 & ((ans_0 & ans_15) | ans_4));
   *possible &= ans_12 | (ans_1_2) | (ans_6 & ans_7);
   ans_9_10 = ans_9 & ans_10;
   *possible &= ans_2 | (ans_9_10 & ((ans_7 & ans_8) | ans_12));
   *possible &= ans_4 | (ans_9_10) | (ans_14 & ans_15);
   *possible &= ans_8 | ans_14 | ans_2;
   *possible &= ans_0 | ans_10 | ans_6;
   if (!*possible) goto function_terminate;

   CHECKER(ans_5, low, high, 5);
   CHECKER(ans_13, low, high, 13);

   ans_15_0 = ans_15 & ans_0;
   ans_7_8 = ans_7 & ans_8;
   ans_12_13 = ans_12 & ans_13;
   *possible &= ans_5 | (ans_12_13 & ans_14 & ((ans_15_0) | ans_10));
   *possible &= ans_7 | (ans_1 & ans_2) | (ans_12_13);
   *possible &= ans_2 | (ans_12_13) | (ans_7_8);
   ans_4_5 = ans_4 & ans_5;
   ans_9_10 = ans_9 & ans_10;
   *possible &= ans_13 | (ans_4_5 & ans_6 & ((ans_7_8) | ans_2));
   *possible &= ans_15 | (ans_4_5) | (ans_9_10);
   *possible &= ans_10 | (ans_4_5) | (ans_15_0);
   *possible &= ans_15 | (ans_9_10) | (ans_4_5);
   *possible &= ans_8 | (ans_13 & ans_14) | ans_2;
   *possible &= ans_0 | (ans_5 & ans_6) | ans_10;
   if (!*possible) goto function_terminate;

   CHECKER(ans_3, low, high, 3);
   CHECKER(ans_11, low, high, 11);

   ans_2_3 = ans_2 & ans_3;
   *possible &= ans_11 | (ans_2_3 & ans_4 & ((ans_0 & ans_1) | (ans_5 & ans_6)));
   *possible &= ans_13 | (ans_7 & ans_8) | (ans_2_3);
   *possible &= ans_8 | (ans_2_3) | (ans_13 & ans_14);
   ans_11_12 = ans_11 & ans_12;
   *possible &= ans_3 | (ans_10 & ans_11_12 & ((ans_8 & ans_9) | (ans_13 & ans_14)));
   *possible &= ans_1 | (ans_11_12) | (ans_6 & ans_7);
   *possible &= ans_6 | (ans_0 & ans_1) | (ans_11_12);
   ans_3_4 = ans_3 & ans_4;
   *possible &= ans_9 | (ans_3_4) | (ans_14 & ans_15);
   *possible &= ans_14 | (ans_8 & ans_9) | (ans_3_4);
   ans_10_11 = ans_10 & ans_11;
   *possible &= ans_5 | (ans_15 & ans_0) | (ans_10_11);
   *possible &= ans_0 | (ans_10_11) | (ans_5 & ans_6);
   if (!*possible) goto function_terminate;

   *possible |= (*possible >> 16);

function_terminate:
   return error;
}

Rox_ErrorCode rox_fastst_detector (
   Rox_DynVec_Segment_Point points, 
   Rox_Image source, 
   Rox_Sint barrier, 
   Rox_Uint lvl
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Segment_Point_Struct toadd;

   if (!points || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_segment_point_reset(points);
   
   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uchar_get_size ( &height, &width, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stride = 0;
   error = rox_array2d_uchar_get_stride ( &stride, source );   
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &data, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   int tabs[16];
   tabs[0] = - (int) stride * 3;
   tabs[1] = 1 - stride * 3;
   tabs[2] = 2 - stride * 2;
   tabs[3] = 3 - stride * 1;
   tabs[4] = 3;
   tabs[5] = 3 + stride * 1;
   tabs[6] = 2 + stride * 2;
   tabs[7] = 1 + stride * 3;
   tabs[8] = stride * 3;
   tabs[9] = -1 + stride * 3;
   tabs[10] = -2 + stride * 2;
   tabs[11] = -3 + stride * 1;
   tabs[12] = -3;
   tabs[13] = -3 - stride * 1;
   tabs[14] = -2 - stride * 2;
   tabs[15] = -1 - stride * 3;

   int possible = 0;

   for ( Rox_Sint i = 5 ; i < height - 5; i++)
   {
      Rox_Uchar * row = data[i];
      Rox_Uchar * ptr = &row[5];

      for ( Rox_Sint j = 5; j < width - 5; j+=16, ptr+=16)
      {
         error = rox_ansi_check_possible_sse ( &possible, barrier, tabs, ptr );
         if ( !possible ) continue;

         if (possible & (1 << 0)) TOADD(i, j + 0);
         if (possible & (1 << 1)) TOADD(i, j + 1);
         if (possible & (1 << 2)) TOADD(i, j + 2);
         if (possible & (1 << 3)) TOADD(i, j + 3);
         if (possible & (1 << 4)) TOADD(i, j + 4);
         if (possible & (1 << 5)) TOADD(i, j + 5);
         if (possible & (1 << 6)) TOADD(i, j + 6);
         if (possible & (1 << 7)) TOADD(i, j + 7);
         if (possible & (1 << 8)) TOADD(i, j + 8);
         if (possible & (1 << 9)) TOADD(i, j + 9);
         if (possible & (1 << 10)) TOADD(i, j + 10);
         if (possible & (1 << 11)) TOADD(i, j + 11);
         if (possible & (1 << 12)) TOADD(i, j + 12);
         if (possible & (1 << 13)) TOADD(i, j + 13);
         if (possible & (1 << 14)) TOADD(i, j + 14);
         if (possible & (1 << 15)) TOADD(i, j + 15);
      }
   }

function_terminate:
   return error;
}
