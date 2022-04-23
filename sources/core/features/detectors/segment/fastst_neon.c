//==============================================================================
//
//    OPENROX   : File fastst_neon.c
//
//    Contents  : Implementation of fastst module
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
      circle = vld1q_u8(ptr + tabs[CIRCLEID]);\
      testlow = vcltq_u8(circle, LO);\
      testlow = vandq_u8(testlow, ones);\
      testhi = vcgtq_u8(circle, HI);\
      testhi = vandq_u8(testhi, twos);\
      test16 = vorrq_u8(testhi, testlow);\
      test16 = vrshlq_u8(test16, mask);\
      test8 = vpaddlq_u8(test16);\
      test4 = vpaddlq_u16(test8);\
      test4 = vrshlq_u32(test4, mask2);\
      test2 = vpaddlq_u32(test4);\
      RES = (vgetq_lane_u64(test2, 0) + vgetq_lane_u64(test2, 1));\

#define TOADD(I,J)\
{\
   toadd.i = I;\
   toadd.j = J;\
   toadd.score = 0;\
   toadd.level = lvl;\
   rox_dynvec_segment_point_append(points, &toadd);\
}

int rox_neon_check_possible ( int * possible, int barrier, int tabs[16], unsigned char * ptr )
{
   int error = 0;

   // To be completed

//function_terminate:
   return error;
}

Rox_ErrorCode rox_fastst_detector (
   Rox_DynVec_Segment_Point points, 
   const Rox_Image source, 
   const Rox_Sint barrier, 
   const Rox_Uint lvl
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   Rox_Segment_Point_Struct toadd;

   // Rox_Sint nucleus, circle, low, high, res, possible;
   int  possible;
   uint8x16_t nucleus, circle, low, high;
   uint8x16_t vbarrier, ones, twos, testlow, testhi, test16;
   uint16x8_t test8;
   uint32x4_t test4;
   uint64x2_t test2;
   int8x16_t mask;
   int32x4_t mask2;

   int tabs[16];
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

   mask = vdupq_n_s8(0);
   mask2 = vdupq_n_s32(0);

   mask = vsetq_lane_s8(0, mask, 0);
   mask = vsetq_lane_s8(2, mask, 1);
   mask = vsetq_lane_s8(4, mask, 2);
   mask = vsetq_lane_s8(6, mask, 3);
   mask = vsetq_lane_s8(0, mask, 4);
   mask = vsetq_lane_s8(2, mask, 5);
   mask = vsetq_lane_s8(4, mask, 6);
   mask = vsetq_lane_s8(6, mask, 7);
   mask = vsetq_lane_s8(0, mask, 8);
   mask = vsetq_lane_s8(2, mask, 9);
   mask = vsetq_lane_s8(4, mask, 10);
   mask = vsetq_lane_s8(6, mask, 11);
   mask = vsetq_lane_s8(0, mask, 12);
   mask = vsetq_lane_s8(2, mask, 13);
   mask = vsetq_lane_s8(4, mask, 14);
   mask = vsetq_lane_s8(6, mask, 15);

   mask2 = vsetq_lane_s32(0, mask2, 0);
   mask2 = vsetq_lane_s32(8, mask2, 1);
   mask2 = vsetq_lane_s32(16, mask2, 2);
   mask2 = vsetq_lane_s32(24, mask2, 3);

   if (!points || !source)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_segment_point_reset(points);

   Rox_Sint width, height;
   error = rox_array2d_uchar_get_size(&height, &width, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint stride = 0;
   error = rox_array2d_uchar_get_stride(&stride, source);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uchar ** data;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &data, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   tabs[0] = - stride * 3;
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

   vbarrier = vdupq_n_u8(barrier);
   ones = vdupq_n_u8(1);
   twos = vdupq_n_u8(2);

   for (Rox_Uint i = 5 ; i < height - 5; i++)
   {
      Rox_Uchar *row = data[i];
      Rox_Uchar *ptr = &row[5];

      for (Rox_Uint j = 5; j < width - 5; j+=16, ptr+=16)
      {
         nucleus = vld1q_u8(ptr);

         low = vqsubq_u8(nucleus, vbarrier);
         high = vqaddq_u8(nucleus, vbarrier);

         CHECKER(ans_0, low, high, 0);
         CHECKER(ans_8, low, high, 8);

         possible = ans_0 | ans_8;
         if (!possible)
            continue;

         CHECKER(ans_15, low, high, 15);
         CHECKER(ans_1, low, high, 1);

         possible &= ans_8 | (ans_15 & ans_1);
         if (!possible)
            continue;

         CHECKER(ans_7, low, high, 7);
         CHECKER(ans_9, low, high, 9);

         possible &= ans_9 | (ans_0 & ans_1);
         possible &= ans_7 | (ans_15 & ans_0);
         if (!possible) continue;

         CHECKER(ans_12, low, high, 12);
         CHECKER(ans_4, low, high, 4);

         possible &= ans_12 | (ans_4 & (ans_1 | ans_7));
         possible &= ans_4 | (ans_12 & (ans_9 | ans_15));
         if (!possible) continue;

         CHECKER(ans_14, low, high, 14);
         CHECKER(ans_6, low, high, 6);

         ans_6_7 = ans_6 & ans_7;
         possible &= ans_14 | (ans_6_7 & (ans_4 | (ans_8 & ans_9)));
         possible &= ans_1 | (ans_6_7) | ans_12;
         ans_14_15 = ans_14 & ans_15;
         possible &= ans_6 | (ans_14_15 & (ans_12 | (ans_0 & ans_1)));
         possible &= ans_9 | (ans_14_15) | ans_4;

         if (!possible) continue;

         CHECKER(ans_2, low, high, 2);
         CHECKER(ans_10, low, high, 10);

         ans_1_2 = ans_1 & ans_2;
         possible &= ans_10 | (ans_1_2 & ((ans_0 & ans_15) | ans_4));
         possible &= ans_12 | (ans_1_2) | (ans_6 & ans_7);
         ans_9_10 = ans_9 & ans_10;
         possible &= ans_2 | (ans_9_10 & ((ans_7 & ans_8) | ans_12));
         possible &= ans_4 | (ans_9_10) | (ans_14 & ans_15);
         possible &= ans_8 | ans_14 | ans_2;
         possible &= ans_0 | ans_10 | ans_6;
         if (!possible) continue;

         CHECKER(ans_5, low, high, 5);
         CHECKER(ans_13, low, high, 13);

         ans_15_0 = ans_15 & ans_0;
         ans_7_8 = ans_7 & ans_8;
         ans_12_13 = ans_12 & ans_13;
         possible &= ans_5 | (ans_12_13 & ans_14 & ((ans_15_0) | ans_10));
         possible &= ans_7 | (ans_1 & ans_2) | (ans_12_13);
         possible &= ans_2 | (ans_12_13) | (ans_7_8);
         ans_4_5 = ans_4 & ans_5;
         ans_9_10 = ans_9 & ans_10;
         possible &= ans_13 | (ans_4_5 & ans_6 & ((ans_7_8) | ans_2));
         possible &= ans_15 | (ans_4_5) | (ans_9_10);
         possible &= ans_10 | (ans_4_5) | (ans_15_0);
         possible &= ans_15 | (ans_9_10) | (ans_4_5);
         possible &= ans_8 | (ans_13 & ans_14) | ans_2;
         possible &= ans_0 | (ans_5 & ans_6) | ans_10;
         if (!possible) continue;

         CHECKER(ans_3, low, high, 3);
         CHECKER(ans_11, low, high, 11);

         ans_2_3 = ans_2 & ans_3;
         possible &= ans_11 | (ans_2_3 & ans_4 & ((ans_0 & ans_1) | (ans_5 & ans_6)));
         possible &= ans_13 | (ans_7 & ans_8) | (ans_2_3);
         possible &= ans_8 | (ans_2_3) | (ans_13 & ans_14);
         ans_11_12 = ans_11 & ans_12;
         possible &= ans_3 | (ans_10 & ans_11_12 & ((ans_8 & ans_9) | (ans_13 & ans_14)));
         possible &= ans_1 | (ans_11_12) | (ans_6 & ans_7);
         possible &= ans_6 | (ans_0 & ans_1) | (ans_11_12);
         ans_3_4 = ans_3 & ans_4;
         possible &= ans_9 | (ans_3_4) | (ans_14 & ans_15);
         possible &= ans_14 | (ans_8 & ans_9) | (ans_3_4);
         ans_10_11 = ans_10 & ans_11;
         possible &= ans_5 | (ans_15 & ans_0) | (ans_10_11);
         possible &= ans_0 | (ans_10_11) | (ans_5 & ans_6);
         if (!possible) continue;

         if (possible & (3 << 0)) TOADD(i, j + 0);
         if (possible & (3 << 2)) TOADD(i, j + 1);
         if (possible & (3 << 4)) TOADD(i, j + 2);
         if (possible & (3 << 6)) TOADD(i, j + 3);
         if (possible & (3 << 8)) TOADD(i, j + 4);
         if (possible & (3 << 10)) TOADD(i, j + 5);
         if (possible & (3 << 12)) TOADD(i, j + 6);
         if (possible & (3 << 14)) TOADD(i, j + 7);
         if (possible & (3 << 16)) TOADD(i, j + 8);
         if (possible & (3 << 18)) TOADD(i, j + 9);
         if (possible & (3 << 20)) TOADD(i, j + 10);
         if (possible & (3 << 22)) TOADD(i, j + 11);
         if (possible & (3 << 24)) TOADD(i, j + 12);
         if (possible & (3 << 26)) TOADD(i, j + 13);
         if (possible & (3 << 28)) TOADD(i, j + 14);
         if (possible & (3 << 30)) TOADD(i, j + 15);
      }
   }

function_terminate:
   return error;
}

#if 0
res = flags[0] & flags[1] & flags[2] & flags[3] & flags[4] & flags[5] & flags[6] & flags[7] & flags[8]; if (res) goto on_add;
res = flags[1] & flags[2] & flags[3] & flags[4] & flags[5] & flags[6] & flags[7] & flags[8] & flags[9]; if (res) goto on_add;
res = flags[2] & flags[3] & flags[4] & flags[5] & flags[6] & flags[7] & flags[8] & flags[9] & flags[10]; if (res) goto on_add;
res = flags[3] & flags[4] & flags[5] & flags[6] & flags[7] & flags[8] & flags[9] & flags[10] & flags[11]; if (res) goto on_add;
res = flags[4] & flags[5] & flags[6] & flags[7] & flags[8] & flags[9] & flags[10] & flags[11] & flags[12]; if (res) goto on_add;
res = flags[5] & flags[6] & flags[7] & flags[8] & flags[9] & flags[10] & flags[11] & flags[12] & flags[13]; if (res) goto on_add;
res = flags[6] & flags[7] & flags[8] & flags[9] & flags[10] & flags[11] & flags[12] & flags[13] & flags[14]; if (res) goto on_add;
res = flags[7] & flags[8] & flags[9] & flags[10] & flags[11] & flags[12] & flags[13] & flags[14] & flags[15]; if (res) goto on_add;
res = flags[8] & flags[9] & flags[10] & flags[11] & flags[12] & flags[13] & flags[14] & flags[15] & flags[0]; if (res) goto on_add;
res = flags[9] & flags[10] & flags[11] & flags[12] & flags[13] & flags[14] & flags[15] & flags[0] & flags[1]; if (res) goto on_add;
res = flags[10] & flags[11] & flags[12] & flags[13] & flags[14] & flags[15] & flags[0] & flags[1] & flags[2]; if (res) goto on_add;
res = flags[11] & flags[12] & flags[13] & flags[14] & flags[15] & flags[0] & flags[1] & flags[2] & flags[3]; if (res) goto on_add;
res = flags[12] & flags[13] & flags[14] & flags[15] & flags[0] & flags[1] & flags[2] & flags[3] & flags[4]; if (res) goto on_add;
res = flags[13] & flags[14] & flags[15] & flags[0] & flags[1] & flags[2] & flags[3] & flags[4] & flags[5]; if (res) goto on_add;
res = flags[14] & flags[15] & flags[0] & flags[1] & flags[2] & flags[3] & flags[4] & flags[5] & flags[6]; if (res) goto on_add;
res = flags[15] & flags[0] & flags[1] & flags[2] & flags[3] & flags[4] & flags[5] & flags[6] & flags[7]; if (res) goto on_add;
#endif