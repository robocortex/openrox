//==============================================================================
//
//    OPENROX   : File shicorner9x9_sse.c
//
//    Contents  : Implementation of shicorner9x9 module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "shicorner9x9.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

int rox_sse_shicorner9x9_test (
   float * response, 
   unsigned char ** image_data, 
   unsigned int v, 
   unsigned int u
)
{
   int error = 0;
   const float norm = 1.0f / 162.0f;
   unsigned int pos_v = v - 5;
   unsigned int pos_u = u - 6;

   // Init to zero
   *response = 0.0f;

   __m128i accdxx1 = _mm_setzero_si128();
   __m128i accdxx2 = _mm_setzero_si128();
   __m128i accdxy1 = _mm_setzero_si128();
   __m128i accdxy2 = _mm_setzero_si128();
   __m128i accdyy1 = _mm_setzero_si128();
   __m128i accdyy2 = _mm_setzero_si128();
   
   // Load 16 uchar into a 128 bit register (128 = 16*8)
   __m128i row1 = _mm_loadu_si128((__m128i *) &image_data[pos_v  ][pos_u]);

   // Convert 8 uchar (8bits) to 8 short (16 bit)
   __m128i row11 = _mm_cvtepu8_epi16(row1);
   
   // Convert 6 uchar (8bits) to 6 short (16 bit)
   __m128i row12 = _mm_cvtepu8_epi16(_mm_srli_si128(row1, 6));

   // Load 16 uchar into a 128 bit register (128 = 16*8)
   __m128i row2 = _mm_loadu_si128((__m128i *)&image_data[pos_v+1][pos_u]);

   // Convert 8 uchar (8bits) to 8 short (16 bit)
   __m128i row21 = _mm_cvtepu8_epi16(row2);
   
   // Convert 6 uchar (8bits) to 6 short (16 bit)
   __m128i row22 = _mm_cvtepu8_epi16(_mm_srli_si128(row2, 6));

   for ( int k = 2; k < 11; k++)
   {
      // Load 16 uchar into a 128 bit register (128 = 16*8)
      __m128i row3  = _mm_loadu_si128( (__m128i *) &image_data[pos_v+k][pos_u] );

      // Convert 8 uchar (8bits) to 8 short (16 bit)
      __m128i row31 = _mm_cvtepu8_epi16(row3);
      
      // Convert 6 uchar (8bits) to 6 short (16 bit)
      __m128i row32 = _mm_cvtepu8_epi16(_mm_srli_si128(row3, 6));

      // Substract next and previous line (gy), then shift result cells to the left to associate gy and gx
      __m128i suby1 = _mm_srli_si128(_mm_sub_epi16(row31, row11), 2);
      __m128i suby2 = _mm_srli_si128(_mm_sub_epi16(row32, row12), 2);

      // Compute gx1
      __m128i curl1 = row21;
      __m128i curl2 = row22;
      __m128i curr1 = _mm_srli_si128(row21, 2*2);
      __m128i curr2 = _mm_srli_si128(row22, 2*2);
      __m128i subx1 = _mm_sub_epi16(curr1, curl1);
      __m128i subx2 = _mm_sub_epi16(curr2, curl2);

      // Cancel first cell (to allow next pairwise instructions to be correct)
      subx1 = _mm_insert_epi16(subx1, 0, 0);
      suby1 = _mm_insert_epi16(suby1, 0, 0);

      __m128i subdxx1 = _mm_madd_epi16(subx1, subx1); // Dxx1 4*32
      __m128i subdxx2 = _mm_madd_epi16(subx2, subx2); // Dxx2 4*32
      __m128i subdxy1 = _mm_madd_epi16(subx1, suby1); // Dxy1 4*32
      __m128i subdxy2 = _mm_madd_epi16(subx2, suby2); // Dxy2 4*32
      __m128i subdyy1 = _mm_madd_epi16(suby1, suby1); // Dyy1 4*32
      __m128i subdyy2 = _mm_madd_epi16(suby2, suby2); // Dyy2 4*32

      accdxx1 = _mm_add_epi32(accdxx1, subdxx1);
      accdxx2 = _mm_add_epi32(accdxx2, subdxx2);
      accdxy1 = _mm_add_epi32(accdxy1, subdxy1);
      accdxy2 = _mm_add_epi32(accdxy2, subdxy2);
      accdyy1 = _mm_add_epi32(accdyy1, subdyy1);
      accdyy2 = _mm_add_epi32(accdyy2, subdyy2);

      // switch rows
      row11 = row21;
      row12 = row22;
      row21 = row31;
      row22 = row32;
   }

   float dxx = (float) ((signed)_mm_extract_epi32(accdxx1, 0) + (signed)_mm_extract_epi32(accdxx1, 1) + (signed)_mm_extract_epi32(accdxx1, 2) + (signed)_mm_extract_epi32(accdxx2, 0) + (signed)_mm_extract_epi32(accdxx2, 1));
   float dyy = (float) ((signed)_mm_extract_epi32(accdyy1, 0) + (signed)_mm_extract_epi32(accdyy1, 1) + (signed)_mm_extract_epi32(accdyy1, 2) + (signed)_mm_extract_epi32(accdyy2, 0) + (signed)_mm_extract_epi32(accdyy2, 1));
   float dxy = (float) ((signed)_mm_extract_epi32(accdxy1, 0) + (signed)_mm_extract_epi32(accdxy1, 1) + (signed)_mm_extract_epi32(accdxy1, 2) + (signed)_mm_extract_epi32(accdxy2, 0) + (signed)_mm_extract_epi32(accdxy2, 1));

   dxx = dxx * norm;
   dyy = dyy * norm;
   dxy = dxy * norm;

   // eigenvalues of the matrix [dxx dxy; dxy dyy] are :
   // l1 = dYY / 0.2e1 + dXX / 0.2e1 + sqrt(dYY * dYY - 0.2e1 * dYY * dXX + dXX * dXX + 0.4e1 * dXY * dXY) / 0.2e1;
   // l2 = dYY / 0.2e1 + dXX / 0.2e1 - sqrt(dYY * dYY - 0.2e1 * dYY * dXX + dXX * dXX + 0.4e1 * dXY * dXY) / 0.2e1;
   // l2 is obviously always less than l1 ! so l2 is min(l1,l2) as in shi paper

   *response = 0.5f * (dyy + dxx - sqrtf(dyy * dyy - 2.0f * dyy * dxx + dxx * dxx + 4.0f * dxy * dxy));

   return error;
}

Rox_ErrorCode rox_shicorner9x9_test (
   Rox_Float * response, 
   Rox_Uchar ** image_data, 
   const Rox_Uint v, 
   const Rox_Uint u
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !response || !image_data ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( v <= 8 || u <= 8 ) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_sse_shicorner9x9_test ( response, image_data, v, u );
   ROX_ERROR_CHECK_TERMINATE(error);

function_terminate:
   return error;
}