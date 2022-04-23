//============================================================================
//
//    OPENROX   : File region_zncc_search_mask_template_mask_neon.c
//
//    Contents  : Implementation of region_zncc_search_mask_template_mask module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "region_zncc_search_mask_template_mask.h"
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

int rox_ansi_array2d_float_region_zncc_search_mask_template_mask_neon (
   float * res_score,
   int * res_topleft_x,
   int * res_topleft_y,
   float ** ds,
   unsigned int ** dsm,
   int sheight,
   int swidth,
   float ** dt,
   unsigned int ** dtm,
   int theight,
   int twidth
)
{
   int error = 0;

   Rox_Uint * drowsm, * drowtm;
   Rox_Float * drows, * drowt;

   Rox_Sint count;
   Rox_Float cc;
   Rox_Float sumt, sums;
   Rox_Float sumsqt, sumsqs;

   Rox_Double meant, means;
   Rox_Double rcc, rsumsqt, rsumsqs, nom, denoms, denomt;
   Rox_Double curscore;

   Rox_Uint shiftwidth = swidth - twidth;
   Rox_Uint shiftheight = sheight - theight;
   
   Rox_Sint twidth4;
   twidth4 = twidth / 4;
   if (twidth % 4) twidth4++;

   // Worst possible score is -1
   Rox_Double bestscore = -1.1;
   Rox_Uint posbestscorex = 0;
   Rox_Uint posbestscorey = 0;

   Rox_Neon_Float usse0123;

   usse0123.tab[0] = 0;
   usse0123.tab[1] = 1;
   usse0123.tab[2] = 2;
   usse0123.tab[3] = 3;

   // Loop over possible "configurations"
   for (Rox_Uint shifty = 0; shifty < shiftheight; shifty++)
   {
      // Loop over possible "configurations"
      for (Rox_Uint shiftx = 0; shiftx < shiftwidth; shiftx++)
      {
         uint32x4_t ssemask;
         float32x2_t reductor;
         float32x4_t ssecol, ssevs, ssevt;

         float32x4_t ssewidth = vdupq_n_f32(twidth);
         float32x4_t sse1 = vdupq_n_f32(1);
         float32x4_t sse4 = vdupq_n_f32(4);
         float32x4_t ssecount = vdupq_n_f32(0);
         float32x4_t ssesumt = vdupq_n_f32(0);
         float32x4_t ssesums = vdupq_n_f32(0);
         float32x4_t ssesumsqs = vdupq_n_f32(0);
         float32x4_t ssesumsqt = vdupq_n_f32(0);
         float32x4_t ssecc = vdupq_n_f32(0);

         count = 0;
         sumt = 0;
         sums = 0;
         sumsqs = 0;
         sumsqt = 0;
         cc = 0;

         // Compute zncc per pixel components
         for ( Rox_Uint i = 0; i < theight; i++ )
         {
            drowsm = &(dsm[shifty + i][shiftx]);
            drowtm = dtm[i];
            drows = &(ds[shifty + i][shiftx]);
            drowt = dt[i];

            ssecol = usse0123.ssetype;

            for ( Rox_Uint j = 0; j < twidth4; j++ )
            {
               sse1 = vdupq_n_f32(1);

               ssemask = vcltq_f32(ssecol, ssewidth);
               ssemask = vandq_u32(ssemask, vld1q_u32(drowsm));
               ssemask = vandq_u32(ssemask, vld1q_u32(drowtm));

               sse1 = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(sse1)));
               ssevs = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(vld1q_f32(drows))));
               ssevt = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(vld1q_f32(drowt))));

               ssecc = vaddq_f32(ssecc, vmulq_f32(ssevs, ssevt));
               ssesums = vaddq_f32(ssesums, ssevs);
               ssesumt = vaddq_f32(ssesumt, ssevt);
               ssesumsqs = vmlaq_f32(ssesumsqs, ssevs, ssevs);
               ssesumsqt = vmlaq_f32(ssesumsqt, ssevt, ssevt);

               ssecount = vaddq_f32(ssecount, sse1);
               ssecol = vaddq_f32(ssecol, sse4);

               drowsm+=4;
               drowtm+=4;
               drows+=4;
               drowt+=4;
            }
         }

         reductor = vadd_f32(vget_high_f32(ssecc), vget_low_f32(ssecc));
         cc = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssecount), vget_low_f32(ssecount));
         count = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssesumsqs), vget_low_f32(ssesumsqs));
         sumsqs = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssesumsqt), vget_low_f32(ssesumsqt));
         sumsqt = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssesums), vget_low_f32(ssesums));
         sums = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssesumt), vget_low_f32(ssesumt));
         sumt = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         // Compute zncc value
         if (count == 0) continue;

         rcc = cc;
         rsumsqs = sumsqs;
         rsumsqt = sumsqt;
         meant = sumt / ((Rox_Double)count);
         means = sums / ((Rox_Double)count);
         nom = rcc - meant*means*(Rox_Double)count;
         denoms = rsumsqs - means*means*(Rox_Double)count;
         denomt = rsumsqt - meant*meant*(Rox_Double)count;
         curscore = nom / (sqrt(denoms)*sqrt(denomt));

         // Store the better results
         if (curscore > bestscore)
         {
            bestscore = curscore;
            posbestscorex = shiftx;
            posbestscorey = shifty;
         }
      }
   }

   // Return result
   *res_score = 0.5 * (bestscore + 1.0);
   *res_topleft_x = posbestscorex;
   *res_topleft_y = posbestscorey;

   return error;
}

Rox_ErrorCode rox_array2d_float_region_zncc_search_mask_template_mask (
   Rox_Float * res_score, Rox_Sint * res_topleft_x, Rox_Sint * res_topleft_y, 
   const Rox_Array2D_Float isearch, 
   const Rox_Imask isearch_mask, 
   const Rox_Array2D_Float itemplate, 
   const Rox_Imask itemplate_mask
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!res_score || !res_topleft_x || !res_topleft_y)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!itemplate || !itemplate_mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!isearch || !isearch_mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint twidth, theight;
   error = rox_array2d_float_get_size ( &theight, &twidth, itemplate );
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Sint swidth, sheight;
   error = rox_array2d_float_get_size(&sheight, &swidth, isearch);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(itemplate_mask, theight, twidth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(isearch_mask, sheight, swidth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (swidth < twidth || sheight < theight) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Uint ** dsm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dsm, isearch_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Uint ** dtm = NULL;
   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dtm, itemplate_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** ds = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &ds, isearch);
   ROX_ERROR_CHECK_TERMINATE ( error );

   Rox_Float ** dt = NULL;
   error = rox_array2d_float_get_data_pointer_to_pointer ( &dt, itemplate);
   ROX_ERROR_CHECK_TERMINATE ( error );

#ifdef integrated
   Rox_Sint twidth4;
   twidth4 = twidth / 4;
   if (twidth % 4) twidth4++;

   Rox_Uint shiftwidth = swidth - twidth;
   Rox_Uint shiftheight = sheight - theight;

   Rox_Uint * drowsm, * drowtm;
   Rox_Float * drows, * drowt;
   Rox_Neon_Float usse0123;

   Rox_Sint count;
   Rox_Float cc;
   Rox_Float sumt, sums;
   Rox_Float sumsqt, sumsqs;

   Rox_Double meant, means;
   Rox_Double rcc, rsumsqt, rsumsqs, nom, denoms, denomt;
   Rox_Double curscore, bestscore;

   Rox_Uint posbestscorex, posbestscorey;

   // Worst possible score is -1
   bestscore = -1.1;
   posbestscorex = 0;
   posbestscorey = 0;

   usse0123.tab[0] = 0;
   usse0123.tab[1] = 1;
   usse0123.tab[2] = 2;
   usse0123.tab[3] = 3;

   // Loop over possible "configurations"
   for (Rox_Uint shifty = 0; shifty < shiftheight; shifty++)
   {
      // Loop over possible "configurations"
      for (Rox_Uint shiftx = 0; shiftx < shiftwidth; shiftx++)
      {
         uint32x4_t ssemask;
         float32x2_t reductor;
         float32x4_t ssecol, ssevs, ssevt;

         float32x4_t ssewidth = vdupq_n_f32(twidth);
         float32x4_t sse1 = vdupq_n_f32(1);
         float32x4_t sse4 = vdupq_n_f32(4);
         float32x4_t ssecount = vdupq_n_f32(0);
         float32x4_t ssesumt = vdupq_n_f32(0);
         float32x4_t ssesums = vdupq_n_f32(0);
         float32x4_t ssesumsqs = vdupq_n_f32(0);
         float32x4_t ssesumsqt = vdupq_n_f32(0);
         float32x4_t ssecc = vdupq_n_f32(0);

         count = 0;
         sumt = 0;
         sums = 0;
         sumsqs = 0;
         sumsqt = 0;
         cc = 0;

         // Compute zncc per pixel components
         for ( Rox_Uint i = 0; i < theight; i++ )
         {
            drowsm = &(dsm[shifty + i][shiftx]);
            drowtm = dtm[i];
            drows = &(ds[shifty + i][shiftx]);
            drowt = dt[i];

            ssecol = usse0123.ssetype;

            for ( Rox_Uint j = 0; j < twidth4; j++ )
            {
               sse1 = vdupq_n_f32(1);

               ssemask = vcltq_f32(ssecol, ssewidth);
               ssemask = vandq_u32(ssemask, vld1q_u32(drowsm));
               ssemask = vandq_u32(ssemask, vld1q_u32(drowtm));

               sse1 = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(sse1)));
               ssevs = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(vld1q_f32(drows))));
               ssevt = vreinterpretq_f32_u32(vandq_u32(ssemask, vreinterpretq_u32_f32(vld1q_f32(drowt))));

               ssecc = vaddq_f32(ssecc, vmulq_f32(ssevs, ssevt));
               ssesums = vaddq_f32(ssesums, ssevs);
               ssesumt = vaddq_f32(ssesumt, ssevt);
               ssesumsqs = vmlaq_f32(ssesumsqs, ssevs, ssevs);
               ssesumsqt = vmlaq_f32(ssesumsqt, ssevt, ssevt);

               ssecount = vaddq_f32(ssecount, sse1);
               ssecol = vaddq_f32(ssecol, sse4);

               drowsm+=4;
               drowtm+=4;
               drows+=4;
               drowt+=4;
            }
         }

         reductor = vadd_f32(vget_high_f32(ssecc), vget_low_f32(ssecc));
         cc = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssecount), vget_low_f32(ssecount));
         count = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssesumsqs), vget_low_f32(ssesumsqs));
         sumsqs = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssesumsqt), vget_low_f32(ssesumsqt));
         sumsqt = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssesums), vget_low_f32(ssesums));
         sums = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         reductor = vadd_f32(vget_high_f32(ssesumt), vget_low_f32(ssesumt));
         sumt = vget_lane_f32(vpadd_f32(reductor, reductor), 0);

         // Compute zncc value
         if (count == 0) continue;

         rcc = cc;
         rsumsqs = sumsqs;
         rsumsqt = sumsqt;
         meant = sumt / ((Rox_Double)count);
         means = sums / ((Rox_Double)count);
         nom = rcc - meant*means*(Rox_Double)count;
         denoms = rsumsqs - means*means*(Rox_Double)count;
         denomt = rsumsqt - meant*meant*(Rox_Double)count;
         curscore = nom / (sqrt(denoms)*sqrt(denomt));

         // Store the better results
         if (curscore > bestscore)
         {
            bestscore = curscore;
            posbestscorex = shiftx;
            posbestscorey = shifty;
         }
      }
   }

   // Return result
   *res_score = 0.5 * (bestscore + 1.0);
   *res_topleft_x = posbestscorex;
   *res_topleft_y = posbestscorey;

#else
   error = rox_ansi_array2d_float_region_zncc_search_mask_template_mask_neon ( res_score, res_topleft_x, res_topleft_y, ds, dsm, sheight, swidth, dt, dtm, theight, twidth );
   ROX_ERROR_CHECK_TERMINATE ( error );
#endif

function_terminate:
   return error;
}

Rox_ErrorCode rox_array2d_uchar_region_zncc_search_mask_template_mask (
   Rox_Float * res_score, 
   Rox_Sint * res_topleft_x, 
   Rox_Sint * res_topleft_y, 
   Rox_Image isearch, 
   Rox_Imask isearch_mask, 
   Rox_Image itemplate, 
   Rox_Imask itemplate_mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint twidth, theight;
   Rox_Sint swidth, sheight;
   Rox_Uint shiftwidth, shiftheight;
   Rox_Uint shiftx, shifty;

   Rox_Uchar ** ds = NULL;
   Rox_Uchar ** dt = NULL;
   Rox_Uint ** dsm = NULL;
   Rox_Uint ** dtm = NULL;

   Rox_Float vt, vs;
   Rox_Sint count;
   Rox_Uint cc;

   Rox_Uint sums, sumt;
   Rox_Uint sumsqt, sumsqs;

   Rox_Double ratio;

   Rox_Double meant, means;
   Rox_Double rcc, rsumsqt, rsumsqs, nom, denoms, denomt, denom;
   Rox_Double curscore, bestscore;

   Rox_Uint posbestscorex, posbestscorey;

   if (!res_score || !res_topleft_x || !res_topleft_y)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!itemplate || !itemplate_mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (!isearch || !isearch_mask)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uchar_get_size(&theight, &twidth, itemplate);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_get_size(&sheight, &swidth, isearch);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(itemplate_mask, theight, twidth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_check_size(isearch_mask, sheight, swidth);
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (swidth < twidth || sheight < theight)
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   shiftwidth = swidth - twidth;
   shiftheight = sheight - theight;

   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dsm, isearch_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uint_get_data_pointer_to_pointer ( &dtm,  itemplate_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &ds,  isearch);
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uchar_get_data_pointer_to_pointer ( &dt,  itemplate);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Worst possible zncc score is -1
   bestscore = -1.1;
   posbestscorex = 0;
   posbestscorey = 0;

   // Loop over possible "configurations"
   for (shifty = 0; shifty < shiftheight; shifty++)
   {
      // Loop over possible "configurations"
      for (shiftx = 0; shiftx < shiftwidth; shiftx++)
      {
         count = 0;
         sumt = 0.0;
         sums = 0.0;
         sumsqs = 0.0;
         sumsqt = 0.0;
         cc = 0.0;

         // Compute zncc per pixel components
         for (Rox_Uint i = 0; i < theight; i++)
         {
            for (Rox_Uint j = 0; j < twidth; j++)
            {
               // Mask handle
               if (!dsm[shifty + i][shiftx + j]) continue;
               if (!dtm[i][j]) continue;

               // Values
               vs = ds[shifty + i][shiftx + j];
               vt = dt[i][j];

               // Zncc parts
               count++;
               cc += vs * vt;
               sumt += vt;
               sums += vs;
               sumsqs += vs * vs;
               sumsqt += vt * vt;
            }
         }

         // Compute zncc value
         if (count == 0) continue;
         ratio = (Rox_Double) count / (Rox_Double) (twidth*theight);

         // Check if there is enough data to compute zncc
         if (ratio < MIN_VISIBLE_RATIO) continue;

         rcc = (Rox_Double) cc;
         rsumsqs = (Rox_Double) sumsqs;
         rsumsqt = (Rox_Double) sumsqt;
         meant = sumt / ((Rox_Double) count);
         means = sums / ((Rox_Double) count);
         nom = rcc - meant * means * (Rox_Double) count;
         denoms = rsumsqs - means * means * (Rox_Double) count;
         denomt = rsumsqt - meant * meant * (Rox_Double) count;

         if (denoms < DBL_EPSILON) continue;
         if (denomt < DBL_EPSILON) continue;

         denom = sqrt(denoms) * sqrt(denomt);

         if (denom < DBL_EPSILON) continue;
         curscore = nom / denom;

         // Store the better results
         if (curscore > bestscore)
         {
            bestscore = curscore;
            posbestscorex = shiftx;
            posbestscorey = shifty;
         }
      }
   }

   // Return results
   *res_score = 0.5 * (bestscore + 1.0);
   *res_topleft_x = posbestscorex;
   *res_topleft_y = posbestscorey;

function_terminate:
   return error;
}
