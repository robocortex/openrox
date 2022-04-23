//==============================================================================
//
//    OPENROX   : File array2d_uchar_zncc_neon.c
//
//    Contents  : Implementation of array2d_uchar_zncc module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "array2d_uchar_zncc.h"
#include <baseproc/maths/maths_macros.h>
#include <float.h>
#include <stdio.h>

#include "inout/system/errors_print.h"

int rox_ansi_array2d_uchar_zncc_neon (
   double * zncc,
   unsigned char ** one_data,
   unsigned char ** two_data,
   unsigned int  ** mask_data,
   int rows,
   int cols
)
{
   int error = 0;

   Rox_Slint sum1 = 0;
   Rox_Slint sum2 = 0;
   Rox_Slint sumsq1 = 0;
   Rox_Slint sumsq2 = 0;
   Rox_Uint count = 0;
   Rox_Slint gcc = 0;
 
   Rox_Uint vcount[4], vsum1[4], vsum2[4], vsq1[4], vsq2[4], vcc[4], vmask[4];

   uint32x4_t neoncount = vdupq_n_u32(0);
   uint32x4_t neonsum1 = vdupq_n_u32(0);
   uint32x4_t neonsum2 = vdupq_n_u32(0);
   uint32x4_t neonsq1 = vdupq_n_u32(0);
   uint32x4_t neonsq2 = vdupq_n_u32(0);
   uint32x4_t neoncc = vdupq_n_u32(0);
   uint32x4_t neonone = vdupq_n_u32(1);
   uint32x4_t neonmask2 = vdupq_n_u32(~0);

   Rox_Sint cols4 = cols / 4;
   if (cols4 * 4 < cols)
   {
      for (Rox_Uint j = 0; j < 4; j++)
      {
         if (cols4 * 4 + j < cols)
         {
            vmask[j] = ~0;
         }
         else
         {
            vmask[j] = 0;
         }
      }
      neonmask2 = vld1q_u32(vmask);
      cols4++;
   }

   for ( Rox_Uint i = 0; i < rows; i++ )
   {
      Rox_Uint * rowm = mask_data[i];

      Rox_Uchar * rowd1 = one_data[i];
      Rox_Uchar * rowd2 = two_data[i];

      for ( Rox_Uint j = 0; j < cols4; j++ )
      {
         uint32x4_t neonmask = vld1q_u32(rowm);

         // Last column
         if ( j == cols4 - 1 )
         {
            neonmask = vandq_u32(neonmask, neonmask2);
         }

         uint16x8_t neonin1 = vmovl_u8(vld1_u8(rowd1));
         uint16x8_t neonin2 = vmovl_u8(vld1_u8(rowd2));

         uint16x4_t neonin1_half = vget_low_u16(neonin1);
         uint16x4_t neonin2_half = vget_low_u16(neonin2);

         uint32x4_t neonin132 = vmovl_u16(neonin1_half);
         uint32x4_t neonin232 = vmovl_u16(neonin2_half);

         neonin132 = vandq_u32(neonin132, neonmask);
         neonin232 = vandq_u32(neonin232, neonmask);
         
         uint32x4_t neoninc = vandq_u32(neonone, neonmask);
         
         neoncount = vaddq_u32(neoncount, neoninc);

         neonsum1 = vaddq_u32(neonsum1, neonin132);
         neonsum2 = vaddq_u32(neonsum2, neonin232);

         neonsq1 = vaddq_u32(neonsq1, vmulq_u32(neonin132, neonin132));
         neonsq2 = vaddq_u32(neonsq2, vmulq_u32(neonin232, neonin232));
         neoncc = vaddq_u32(neoncc, vmulq_u32(neonin132, neonin232));

         rowm+=4;
         rowd1+=4;
         rowd2+=4;
      }
   }

   vst1q_u32(vcount, neoncount);
   vst1q_u32(vsum1, neonsum1);
   vst1q_u32(vsum2, neonsum2);
   vst1q_u32(vsq1, neonsq1);
   vst1q_u32(vsq2, neonsq2);
   vst1q_u32(vcc, neoncc);

   count = vcount[0] + vcount[1] + vcount[2] + vcount[3];

   if (count == 0) 
   { error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE ( error ); }

   sum1 = vsum1[0] + vsum1[1] + vsum1[2] + vsum1[3];
   sum2 = vsum2[0] + vsum2[1] + vsum2[2] + vsum2[3];
   sumsq1 = vsq1[0] + vsq1[1] + vsq1[2] + vsq1[3];
   sumsq2 = vsq2[0] + vsq2[1] + vsq2[2] + vsq2[3];
   gcc = vcc[0] + vcc[1] + vcc[2] + vcc[3];

   Rox_Double mean1 = ((double)sum1) / ((double)count);
   Rox_Double mean2 = ((double)sum2) / ((double)count);

   Rox_Double nom = ((Rox_Double)gcc) - mean1*mean2*(Rox_Double)count;
   
   Rox_Double denom1 = (Rox_Double)sumsq1 - mean1 * mean1 * (Rox_Double) count;
   Rox_Double denom2 = (Rox_Double)sumsq2 - mean2 * mean2 * (Rox_Double) count;
   
   if (denom1 < DBL_EPSILON || denom2 < DBL_EPSILON) 
   { error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *zncc = nom / (sqrt(denom1)*sqrt(denom2));

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_zncc ( 
   Rox_Double *zncc, 
   const Rox_Array2D_Uchar one, 
   const Rox_Array2D_Uchar two, 
   const Rox_Array2D_Uint mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
  
   if ( !zncc )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !one || !two || !mask ) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   *zncc = 0.0;

   Rox_Sint cols = 0, rows = 0;
   error = rox_array2d_uchar_get_size ( &rows, &cols, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_check_size ( two, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size ( mask, rows, cols ); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** one_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &one_data, one );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uchar ** two_data = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &two_data, two );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** mask_data = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &mask_data, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifdef integrated
   Rox_Slint sum1 = 0;
   Rox_Slint sum2 = 0;
   Rox_Slint sumsq1 = 0;
   Rox_Slint sumsq2 = 0;
   Rox_Uint count = 0;
   Rox_Slint gcc = 0;
 
   Rox_Uint vcount[4], vsum1[4], vsum2[4], vsq1[4], vsq2[4], vcc[4], vmask[4];

   uint32x4_t neoncount = vdupq_n_u32(0);
   uint32x4_t neonsum1 = vdupq_n_u32(0);
   uint32x4_t neonsum2 = vdupq_n_u32(0);
   uint32x4_t neonsq1 = vdupq_n_u32(0);
   uint32x4_t neonsq2 = vdupq_n_u32(0);
   uint32x4_t neoncc = vdupq_n_u32(0);
   uint32x4_t neonone = vdupq_n_u32(1);
   uint32x4_t neonmask2 = vdupq_n_u32(~0);

   Rox_Sint cols4 = cols / 4;
   if (cols4 * 4 < cols)
   {
      for (Rox_Uint j = 0; j < 4; j++)
      {
         if (cols4 * 4 + j < cols)
         {
            vmask[j] = ~0;
         }
         else
         {
            vmask[j] = 0;
         }
      }
      neonmask2 = vld1q_u32(vmask);
      cols4++;
   }

   for ( Rox_Uint i = 0; i < rows; i++ )
   {
      Rox_Uint * rowm = mask_data[i];

      Rox_Uchar * rowd1 = one_data[i];
      Rox_Uchar * rowd2 = two_data[i];

      for ( Rox_Uint j = 0; j < cols4; j++ )
      {
         uint32x4_t neonmask = vld1q_u32(rowm);

         // Last column
         if ( j == cols4 - 1 )
         {
            neonmask = vandq_u32(neonmask, neonmask2);
         }

         uint16x8_t neonin1 = vmovl_u8(vld1_u8(rowd1));
         uint16x8_t neonin2 = vmovl_u8(vld1_u8(rowd2));

         uint16x4_t neonin1_half = vget_low_u16(neonin1);
         uint16x4_t neonin2_half = vget_low_u16(neonin2);

         uint32x4_t neonin132 = vmovl_u16(neonin1_half);
         uint32x4_t neonin232 = vmovl_u16(neonin2_half);

         neonin132 = vandq_u32(neonin132, neonmask);
         neonin232 = vandq_u32(neonin232, neonmask);
         
         uint32x4_t neoninc = vandq_u32(neonone, neonmask);
         
         neoncount = vaddq_u32(neoncount, neoninc);

         neonsum1 = vaddq_u32(neonsum1, neonin132);
         neonsum2 = vaddq_u32(neonsum2, neonin232);

         neonsq1 = vaddq_u32(neonsq1, vmulq_u32(neonin132, neonin132));
         neonsq2 = vaddq_u32(neonsq2, vmulq_u32(neonin232, neonin232));
         neoncc = vaddq_u32(neoncc, vmulq_u32(neonin132, neonin232));

         rowm+=4;
         rowd1+=4;
         rowd2+=4;
      }
   }

   vst1q_u32(vcount, neoncount);
   vst1q_u32(vsum1, neonsum1);
   vst1q_u32(vsum2, neonsum2);
   vst1q_u32(vsq1, neonsq1);
   vst1q_u32(vsq2, neonsq2);
   vst1q_u32(vcc, neoncc);

   count = vcount[0] + vcount[1] + vcount[2] + vcount[3];

   if (count == 0) 
   { error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE ( error ); }

   sum1 = vsum1[0] + vsum1[1] + vsum1[2] + vsum1[3];
   sum2 = vsum2[0] + vsum2[1] + vsum2[2] + vsum2[3];
   sumsq1 = vsq1[0] + vsq1[1] + vsq1[2] + vsq1[3];
   sumsq2 = vsq2[0] + vsq2[1] + vsq2[2] + vsq2[3];
   gcc = vcc[0] + vcc[1] + vcc[2] + vcc[3];

   Rox_Double mean1 = ((double)sum1) / ((double)count);
   Rox_Double mean2 = ((double)sum2) / ((double)count);

   Rox_Double nom = ((Rox_Double)gcc) - mean1*mean2*(Rox_Double)count;
   
   Rox_Double denom1 = (Rox_Double)sumsq1 - mean1 * mean1 * (Rox_Double)count;
   Rox_Double denom2 = (Rox_Double)sumsq2 - mean2 * mean2 * (Rox_Double)count;
   
   if (denom1 < DBL_EPSILON || denom2 < DBL_EPSILON) 
   { error = ROX_ERROR_ZNCC_UNDEFINED; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *zncc = nom / (sqrt(denom1)*sqrt(denom2));
#else
   error = rox_ansi_array2d_uchar_zncc_neon ( zncc, one_data, two_data, mask_data, rows, cols );
   ROX_ERROR_CHECK_TERMINATE ( error );
#endif

function_terminate:
   return error;
}
