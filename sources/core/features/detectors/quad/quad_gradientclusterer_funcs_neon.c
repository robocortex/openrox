//==============================================================================
//
//    OPENROX   : File quad_gradientclusterer_neon.c
//
//    Contents  : Implementation of quad_gradientclusterer module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include <generated/array2d_uint.h>
#include "quad_gradientclusterer.h"
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/base/basemaths.h>
#include <inout/system/errors_print.h>

// Compute magnitude and angle of the image gradient
Rox_ErrorCode rox_gradientclusterer_buildgradients (
   Rox_Array2D_Uint Imagval, 
   Rox_Uint * Imag, 
   Rox_Float * Itheta, 
   const Rox_Image source, 
   const Rox_Imask mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if ( !Imagval || !Imag || !Itheta )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if ( !source || !mask )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint width = 0, height = 0;
   error = rox_array2d_uchar_get_size ( &height, &width, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint width8 = (width - 2) / 8;
   if (width8 * 8 < width - 2) width8++;

   Rox_Sint width4 = (width - 4) / 4;
   if (width4 * 4 < width - 4) width4++;

   Rox_Uchar ** src = NULL;
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &src, source );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dm, mask );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint ** ptrMag = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &ptrMag, Imagval );
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint * ptrTMag = Imag;
   Rox_Float * ptrTheta = Itheta;

   uint8x8_t neonprcc8, neonprpc8, neonprnc8, neonnrcc8, neonnrpc8, neonnrnc8, neoncrpc8, neoncrnc8;
   int16x8_t neonprcc16, neonprpc16, neonprnc16, neonnrcc16, neonnrpc16, neonnrnc16, neoncrpc16, neoncrnc16 ;
   int16x8_t neondym, neondyp, neondxm, neondxp, neondx, neondy;
   int32x4_t neondx32,neondy32,neonmag,neonmag2;
   uint32x4_t neonmag_prpc, neonmag_prcc, neonmag_prnc;
   uint32x4_t neonmag_crpc, neonmag_crcc, neonmag_crnc;
   uint32x4_t neonmag_nrpc, neonmag_nrcc, neonmag_nrnc;
   uint32x4_t neonmagzone, neon14400, neon57600, neontest_14400, neontest_57600;

   short vdx[8], vdy[8];

   if ( !src || !dm || !ptrMag || !ptrTMag || !ptrTheta )
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // WARNING: the sthresholds are not the same in quad_gradientclusterer_funcs !!!
   neon14400 = vdupq_n_u32(14400);
   neon57600 = vdupq_n_u32(57600);

   for (Rox_Uint i = 1; i < height - 1; i++)
   {
      Rox_Uchar * ptrprpc = &src[i - 1][0];
      Rox_Uchar * ptrprcc = &src[i - 1][1];
      Rox_Uchar * ptrprnc = &src[i - 1][2];
      Rox_Uchar * ptrcrpc = &src[i][0];
      Rox_Uchar * ptrcrcc = &src[i][1];
      Rox_Uchar * ptrcrnc = &src[i][2];
      Rox_Uchar * ptrnrpc = &src[i + 1][0];
      Rox_Uchar * ptrnrcc = &src[i + 1][1];
      Rox_Uchar * ptrnrnc = &src[i + 1][2];
      
      Rox_Uint idx = i * width + 1;
      
      Rox_Uint * rowmag = &ptrMag[i][1];
      Rox_Float * rowtheta = &ptrTheta[idx];

      for (Rox_Uint j = 0; j < width8; j++)
      {
         neonprcc8 = vld1_u8(ptrprcc);
         neonprpc8 = vld1_u8(ptrprpc);
         neonprnc8 = vld1_u8(ptrprnc);
         neonnrcc8 = vld1_u8(ptrnrcc);
         neonnrpc8 = vld1_u8(ptrnrpc);
         neonnrnc8 = vld1_u8(ptrnrnc);
         neoncrpc8 = vld1_u8(ptrcrpc);
         neoncrnc8 = vld1_u8(ptrcrnc);

         neonprcc16 = vreinterpretq_s16_u16(vmovl_u8(neonprcc8));
         neonprpc16 = vreinterpretq_s16_u16(vmovl_u8(neonprpc8));
         neonprnc16 = vreinterpretq_s16_u16(vmovl_u8(neonprnc8));
         neonnrcc16 = vreinterpretq_s16_u16(vmovl_u8(neonnrcc8));
         neonnrpc16 = vreinterpretq_s16_u16(vmovl_u8(neonnrpc8));
         neonnrnc16 = vreinterpretq_s16_u16(vmovl_u8(neonnrnc8));
         neoncrpc16 = vreinterpretq_s16_u16(vmovl_u8(neoncrpc8));
         neoncrnc16 = vreinterpretq_s16_u16(vmovl_u8(neoncrnc8));

         // Classic and simple image gradient
         neondym = vaddq_s16(vaddq_s16(vaddq_s16(neonprcc16, neonprcc16), neonprpc16), neonprnc16);
         neondyp = vaddq_s16(vaddq_s16(vaddq_s16(neonnrcc16, neonnrcc16), neonnrpc16), neonnrnc16);
         neondxm = vaddq_s16(vaddq_s16(vaddq_s16(neoncrpc16, neoncrpc16), neonprpc16), neonnrpc16);
         neondxp = vaddq_s16(vaddq_s16(vaddq_s16(neoncrnc16, neoncrnc16), neonprnc16), neonnrnc16);
         neondx = vsubq_s16(neondxp, neondxm);
         neondy = vsubq_s16(neondyp, neondym);
         vst1q_s16(vdx, neondx);
         vst1q_s16(vdy, neondy);

         neondx32 = vmovl_s16(vget_low_s16(neondx));
         neondy32 = vmovl_s16(vget_low_s16(neondy));
         neonmag2 = vaddq_s32(vmulq_s32(neondx32, neondx32), vmulq_s32(neondy32, neondy32));

         neondx32 = vmovl_s16(vget_high_s16(neondx));
         neondy32 = vmovl_s16(vget_high_s16(neondy));
         neonmag = vaddq_s32(vmulq_s32(neondx32, neondx32), vmulq_s32(neondy32, neondy32));
         vst1q_u32(rowmag, vreinterpretq_u32_s32(neonmag2));
         vst1q_u32(rowmag + 4, vreinterpretq_u32_s32(neonmag));

         ptrprpc+=8;
         ptrprcc+=8;
         ptrprnc+=8;
         ptrcrpc+=8;
         ptrcrcc+=8;
         ptrcrnc+=8;
         ptrnrpc+=8;
         ptrnrcc+=8;
         ptrnrnc+=8;

         // Compute angle of gradient
         for (Rox_Uint k = 0; k < 8; k++)
         {
            Rox_Float theta = 0;
            Rox_Sint dx = vdx[k];
            Rox_Sint dy = vdy[k];

            if (rowmag[k] < 14400)
            {
               theta = 0;
               continue;
            }

            theta = quad_arctan2(dy, dx);

            rowtheta[k] = theta;
         }

         rowtheta+=8;
         rowmag+=8;
      }
   }


   for (Rox_Uint i = 2; i < height - 2; i++)
   {
      Rox_Uint * ptrmag_prpc, *ptrmag_prnc, *ptrmag_prcc;
      Rox_Uint * ptrmag_crpc, *ptrmag_crnc, *ptrmag_crcc;
      Rox_Uint * ptrmag_nrpc, *ptrmag_nrnc, *ptrmag_nrcc;

      ptrmag_prpc = &ptrMag[i - 1][1];
      ptrmag_prcc = &ptrMag[i - 1][2];
      ptrmag_prnc = &ptrMag[i - 1][3];
      ptrmag_crpc = &ptrMag[i    ][1];
      ptrmag_crcc = &ptrMag[i    ][2];
      ptrmag_crnc = &ptrMag[i    ][3];
      ptrmag_nrpc = &ptrMag[i + 1][1];
      ptrmag_nrcc = &ptrMag[i + 1][2];
      ptrmag_nrnc = &ptrMag[i + 1][3];

      Rox_Uint idx = i * width + 2;
      Rox_Uint * rowmag = &ptrTMag[idx];

      for (Rox_Uint j = 0; j < width4; j++)
      {
         // int test_all;
         // Rox_Uint vtest[4];

         neonmag_prpc = vld1q_u32(ptrmag_prpc);
         neonmag_prcc = vld1q_u32(ptrmag_prcc);
         neonmag_prnc = vld1q_u32(ptrmag_prnc);
         neonmag_crpc = vld1q_u32(ptrmag_crpc);
         neonmag_crcc = vld1q_u32(ptrmag_crcc);
         neonmag_crnc = vld1q_u32(ptrmag_crnc);
         neonmag_nrpc = vld1q_u32(ptrmag_nrpc);
         neonmag_nrcc = vld1q_u32(ptrmag_nrcc);
         neonmag_nrnc = vld1q_u32(ptrmag_nrnc);

         // Compute max value in zone
         neonmagzone = vmaxq_u32(neonmag_prcc, neonmag_prpc);
         neonmagzone = vmaxq_u32(neonmagzone, neonmag_prnc);
         neonmagzone = vmaxq_u32(neonmagzone, neonmag_crpc);
         neonmagzone = vmaxq_u32(neonmagzone, neonmag_crnc);
         neonmagzone = vmaxq_u32(neonmagzone, neonmag_nrpc);
         neonmagzone = vmaxq_u32(neonmagzone, neonmag_nrcc);
         neonmagzone = vmaxq_u32(neonmagzone, neonmag_nrnc);

         // Set 0 to max value under 57600 in zone
         neonmagzone = vcgeq_u32(neonmagzone, neon57600);

          // Remove vector values under 14400
         neontest_14400 = vcgeq_u32(neonmag_crcc, neon14400);

         // Set bits to all 1 for all elements below 57600
         neontest_57600 = vcltq_u32(neonmag_crcc, neon57600);

         neonmagzone = vandq_u32(neonmagzone, neontest_57600);
         neonmagzone = vandq_u32(neonmagzone, neontest_14400);

         neonmag_crcc = vbicq_u32(neonmag_crcc, neontest_57600);
         neonmag_crcc = vorrq_u32(neonmag_crcc, neonmagzone);

         vst1q_u32(rowmag, neonmag_crcc);

         ptrmag_prpc+=4; ptrmag_prnc+=4; ptrmag_prcc+=4;
         ptrmag_crpc+=4; ptrmag_crnc+=4; ptrmag_crcc+=4;
         ptrmag_nrpc+=4; ptrmag_nrnc+=4; ptrmag_nrcc+=4;
         rowmag+=4;
      }
   }

function_terminate:
   return error;
}
