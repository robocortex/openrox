//============================================================================
//
//    OPENROX   : File region_zncc_search_mask_template_mask_sse.c
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
#include <system/memory/memory.h>
#include <system/vectorisation/sse.h>
#include <inout/system/print.h>
#include <inout/system/errors_print.h>

int rox_ansi_array2d_float_region_zncc_search_mask_template_mask_sse (
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
   
   // Rox_Float* buf_aligned = NULL;
   // Rox_Float* buf = NULL;

   //According to Intel docs, the buffer given to _mm_store1_ps must be aligned https://software.intel.com/en-us/node/583174 or (Store Intrinsics chapter if site is offline)
   //MSVC or GNU-GCC compiler reference and code don't mention it, but by looking at xmmintrin.h implementation, you can see that all those "store" methods are calling the same builtin function which requires an aligned address.
   //if address is not aligned, there's a great chance it probably won't do anything really harmful, but debug version of windows compilation will consistentlty crash.
   //buf = (Rox_Float*) rox_memory_allocate_aligned((void**)&buf_aligned, sizeof(Rox_Float), 4, ROX_DEFAULT_ALIGNMENT);
   //if (buf == NULL)
   //{ error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   union ssevector buffer;

   int shiftwidth  = swidth  - twidth;
   int shiftheight = sheight - theight;

   int twidth4 = twidth / 4;
   if (twidth % 4) twidth4++;

   // Worst possible score is -1.0
   double bestscore = -1.0;
   int posbestscorex = 0;
   int posbestscorey = 0;

   // Loop over possible "configurations"
   for ( int shifty = 0; shifty < shiftheight; shifty++ )
   {
      // Loop over possible "configurations"
      for ( int shiftx = 0; shiftx < shiftwidth; shiftx++ )
      {
         __m128 ssemask;
         __m128 ssecol, ssevs, ssevt;

         __m128 ssewidth = _mm_set_ps1((float)twidth);
         __m128 sse1 = _mm_set1_ps(1);
         __m128 sse4 = _mm_set1_ps(4);
         __m128 ssecount = _mm_set1_ps(0);
         __m128 ssesumt = _mm_set1_ps(0);
         __m128 ssesums = _mm_set1_ps(0);
         __m128 ssesumsqs = _mm_set1_ps(0);
         __m128 ssesumsqt = _mm_set1_ps(0);
         __m128 ssecc = _mm_set1_ps(0);

         // Init counter
         int count = 0;
         
         float sumt = 0;
         float sums = 0;

         float sumsqs = 0;
         float sumsqt = 0;
         
         float cc = 0;

         // Compute zncc per pixel components
         for ( int i = 0; i < theight; i++ )
         {
            unsigned int * drowsm = &(dsm[shifty + i][shiftx]);
            unsigned int * drowtm = dtm[i];

            float * drows = &(ds[shifty + i][shiftx]);
            float * drowt = dt[i];

            ssecol = _mm_set_ps(3, 2, 1, 0);

            for ( int j = 0; j < twidth4; j++ )
            {
               sse1 = _mm_set1_ps(1);

               ssemask = _mm_cmplt_ps(ssecol, ssewidth);
               ssemask = _mm_and_ps(ssemask, _mm_loadu_ps((Rox_Float*)drowsm));
               ssemask = _mm_and_ps(ssemask, _mm_loadu_ps((Rox_Float*)drowtm));

               sse1 = _mm_and_ps(ssemask, sse1);
               ssevs = _mm_and_ps(ssemask, _mm_loadu_ps(drows));
               ssevt = _mm_and_ps(ssemask, _mm_loadu_ps(drowt));

               ssecc = _mm_add_ps(ssecc, _mm_mul_ps(ssevs, ssevt));
               ssesums = _mm_add_ps(ssesums, ssevs);
               ssesumt = _mm_add_ps(ssesumt, ssevt);
               ssesumsqs = _mm_add_ps(ssesumsqs, _mm_mul_ps(ssevs, ssevs));
               ssesumsqt = _mm_add_ps(ssesumsqt, _mm_mul_ps(ssevt, ssevt));

               ssecount = _mm_add_ps(ssecount, sse1);
               ssecol = _mm_add_ps(ssecol, sse4);

               drowsm+=4;
               drowtm+=4;
               drows+=4;
               drowt+=4;
            }
         }

         ssecc = _mm_hadd_ps(ssecc, ssecc);
         ssecc = _mm_hadd_ps(ssecc, ssecc);
         _mm_store1_ps(buffer.tab, ssecc);
         cc = buffer.tab[0];

         ssecount = _mm_hadd_ps(ssecount, ssecount);
         ssecount = _mm_hadd_ps(ssecount, ssecount);
         _mm_store1_ps(buffer.tab, ssecount);
         count = (int)buffer.tab[0];

         ssesumsqs = _mm_hadd_ps(ssesumsqs, ssesumsqs);
         ssesumsqs = _mm_hadd_ps(ssesumsqs, ssesumsqs);
         _mm_store1_ps(buffer.tab, ssesumsqs);
         sumsqs = buffer.tab[0];

         ssesumsqt = _mm_hadd_ps(ssesumsqt, ssesumsqt);
         ssesumsqt = _mm_hadd_ps(ssesumsqt, ssesumsqt);
         _mm_store1_ps(buffer.tab, ssesumsqt);
         sumsqt = buffer.tab[0];

         ssesums = _mm_hadd_ps(ssesums, ssesums);
         ssesums = _mm_hadd_ps(ssesums, ssesums);
         _mm_store1_ps(buffer.tab, ssesums);
         sums = buffer.tab[0];

         ssesumt = _mm_hadd_ps(ssesumt, ssesumt);
         ssesumt = _mm_hadd_ps(ssesumt, ssesumt);
         _mm_store1_ps(buffer.tab, ssesumt);
         sumt = buffer.tab[0];

         // Compute zncc value
         if (count == 0) continue;

         double ratio = (Rox_Double) count / (Rox_Double) (twidth*theight);

         // Check if there is enough data to compute a meaningful zncc
         if (ratio < MIN_VISIBLE_RATIO) continue;

         double rcc = cc;
         
         double rsumsqs = sumsqs;
         double rsumsqt = sumsqt;
         
         double meant = sumt / ((Rox_Double) count);
         double means = sums / ((Rox_Double) count);

         double nom = rcc - meant*means*(Rox_Double)count;

         double denoms = rsumsqs - means*means*(Rox_Double)count;
         double denomt = rsumsqt - meant*meant*(Rox_Double)count;
         
         double curscore = nom / (sqrt(denoms)*sqrt(denomt));

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
   *res_score = (Rox_Float) (0.5 * (bestscore + 1.0));
   *res_topleft_x = posbestscorex;
   *res_topleft_y = posbestscorey;

   // rox_memory_delete(buf);

   return error;
}

Rox_ErrorCode rox_array2d_float_region_zncc_search_mask_template_mask (
   Rox_Float * res_score,
   Rox_Sint * res_topleft_x,
   Rox_Sint * res_topleft_y,
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

   Rox_Sint twidth = 0, theight = 0;
   error = rox_array2d_float_get_size(&theight, &twidth, itemplate); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Sint swidth = 0, sheight = 0;
   error = rox_array2d_float_get_size(&sheight, &swidth, isearch); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   if (swidth < twidth || sheight < theight) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_uint_check_size(itemplate_mask, theight, twidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_check_size(isearch_mask, sheight, swidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   Rox_Uint ** dsm = NULL; 
   rox_array2d_uint_get_data_pointer_to_pointer( &dsm, isearch_mask);
   
   Rox_Uint ** dtm = NULL; 
   rox_array2d_uint_get_data_pointer_to_pointer( &dtm, itemplate_mask);
   
   Rox_Float ** ds = NULL; 
   rox_array2d_float_get_data_pointer_to_pointer( &ds, isearch);
   
   Rox_Float ** dt = NULL; 
   rox_array2d_float_get_data_pointer_to_pointer( &dt, itemplate);

   error = rox_ansi_array2d_float_region_zncc_search_mask_template_mask_sse ( res_score, res_topleft_x, res_topleft_y, ds, dsm, sheight, swidth, dt, dtm, theight, twidth );
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

#ifdef SSE_UCHAR
Rox_ErrorCode rox_array2d_uchar_region_zncc_search_mask_template_mask(
   Rox_Float * res_score, Rox_Sint * res_topleft_x, Rox_Sint * res_topleft_y, Rox_Image isearch, Rox_Imask isearch_mask, Rox_Image itemplate, Rox_Imask itemplate_mask)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint twidth, twidth4, theight;
   Rox_Sint swidth, sheight;
   Rox_Sint shiftwidth, shiftheight;
   Rox_Sint shiftx, shifty;
   Rox_Uint * drowsm, * drowtm;
   Rox_Float * drows, * drowt;

   //Rox_Float* buf_aligned = NULL;
   //Rox_Float* buf = NULL;

   union ssevector buffer;

   // Input Data
   Rox_Uchar ** ds;
   Rox_Uchar ** dt;
   // INput Masks
   Rox_Uint ** dsm;
   Rox_Uint ** dtm;

   Rox_Uint i, j;
   Rox_Sint count;
   Rox_Float cc;
   Rox_Float sumt, sums;
   Rox_Float sumsqt, sumsqs;

   Rox_Double meant, means;
   Rox_Double rcc, rsumsqt, rsumsqs, nom, denoms, denomt;
   Rox_Double curscore, bestscore;

   Rox_Sint posbestscorex, posbestscorey;

   if (!res_score || !res_topleft_x || !res_topleft_y) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!itemplate || !itemplate_mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!isearch || !isearch_mask) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   //According to Intel docs, the buffer given to _mm_store1_ps must be aligned https://software.intel.com/en-us/node/583174 or (Store Intrinsics chapter if site is offline)
   //MSVC or GNU-GCC compiler reference and code don't mention it, but by looking at xmmintrin.h implementation, you can see that all those "store" methods are calling the same builtin function which requires an aligned address.
   //if address is not aligned, there's a great chance it probably won't do anything really harmful, but debug version of windows compilation will consistentlty crash.
   //buf = (Rox_Float*) rox_memory_allocate_aligned((void**) &buf_aligned, sizeof(Rox_Float), 4, ROX_DEFAULT_ALIGNMENT);
   //if (buf == NULL)
   //{ error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_array2d_float_get_size(&theight, &twidth, itemplate); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_float_get_size(&sheight, &swidth, isearch); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   twidth4 = twidth / 4;
   if (twidth % 4) twidth4++;

   error = rox_array2d_uint_check_size(itemplate_mask, theight, twidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_array2d_uint_check_size(isearch_mask, sheight, swidth); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   if (swidth < twidth || sheight < theight) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   shiftwidth = swidth - twidth;
   shiftheight = sheight - theight;

   error = rox_array2d_uint_get_data_pointer_to_pointer(&dsm, isearch_mask);
   error = rox_array2d_uint_get_data_pointer_to_pointer(&dtm, itemplate_mask);

   error = rox_array2d_uchar_get_data_pointer_to_pointer(&ds, isearch);
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&dt, itemplate);

   // Worst possible score is -1
   bestscore = -1.1;
   posbestscorex = 0;
   posbestscorey = 0;

   // Loop over possible "configurations"
   for (shifty = 0; shifty < shiftheight; shifty++)
   {
      // Loop over possible "configurations"
      for (shiftx = 0; shiftx < shiftwidth; shiftx++)
      {
         __m128 ssemask;
         __m128 ssecol, ssevs, ssevt;

         __m128 ssewidth = _mm_set_ps1((float)twidth);
         __m128 sse1 = _mm_set1_ps(1);
         __m128 sse4 = _mm_set1_ps(4);
         __m128 ssecount = _mm_set1_ps(0);
         __m128 ssesumt = _mm_set1_ps(0);
         __m128 ssesums = _mm_set1_ps(0);
         __m128 ssesumsqs = _mm_set1_ps(0);
         __m128 ssesumsqt = _mm_set1_ps(0);
         __m128 ssecc = _mm_set1_ps(0);

         count = 0;
         sumt = 0;
         sums = 0;
         sumsqs = 0;
         sumsqt = 0;
         cc = 0;

         // Compute zncc per pixel components
         for (i = 0; i < theight; i++)
         {
            drowsm = &(dsm[shifty + i][shiftx]);
            drowtm = dtm[i];
            drows = &(ds[shifty + i][shiftx]);
            drowt = dt[i];

            ssecol = _mm_set_ps(3, 2, 1, 0);

            for (j = 0; j < twidth4; j++)
            {
               sse1 = _mm_set1_ps(1);

               ssemask = _mm_cmplt_ps(ssecol, ssewidth);
               ssemask = _mm_and_ps(ssemask, _mm_loadu_ps((Rox_Float*) drowsm));
               ssemask = _mm_and_ps(ssemask, _mm_loadu_ps((Rox_Float*) drowtm));

               sse1 = _mm_and_ps(ssemask, sse1);
               ssevs = _mm_and_ps(ssemask, _mm_loadu_ps(drows));
               ssevt = _mm_and_ps(ssemask, _mm_loadu_ps(drowt));

               ssecc = _mm_add_ps(ssecc, _mm_mul_ps(ssevs, ssevt));
               ssesums = _mm_add_ps(ssesums, ssevs);
               ssesumt = _mm_add_ps(ssesumt, ssevt);
               ssesumsqs = _mm_add_ps(ssesumsqs, _mm_mul_ps(ssevs, ssevs));
               ssesumsqt = _mm_add_ps(ssesumsqt, _mm_mul_ps(ssevt, ssevt));

               ssecount = _mm_add_ps(ssecount, sse1);
               ssecol = _mm_add_ps(ssecol, sse4);

               drowsm+=4;
               drowtm+=4;

               drows+=4;
               drowt+=4;
            }
         }

         ssecc = _mm_hadd_ps(ssecc, ssecc);
         ssecc = _mm_hadd_ps(ssecc, ssecc);
         _mm_store1_ps(buffer.tab, ssecc);
         cc = buffer.tab[0];

         ssecount = _mm_hadd_ps(ssecount, ssecount);
         ssecount = _mm_hadd_ps(ssecount, ssecount);
         _mm_store1_ps(buffer.tab, ssecount);
         count = (int)buffer.tab[0];

         ssesumsqs = _mm_hadd_ps(ssesumsqs, ssesumsqs);
         ssesumsqs = _mm_hadd_ps(ssesumsqs, ssesumsqs);
         _mm_store1_ps(buffer.tab, ssesumsqs);
         sumsqs = buffer.tab[0];

         ssesumsqt = _mm_hadd_ps(ssesumsqt, ssesumsqt);
         ssesumsqt = _mm_hadd_ps(ssesumsqt, ssesumsqt);
         _mm_store1_ps(buffer.tab, ssesumsqt);
         sumsqt = buffer.tab[0];

         ssesums = _mm_hadd_ps(ssesums, ssesums);
         ssesums = _mm_hadd_ps(ssesums, ssesums);
         _mm_store1_ps(buffer.tab, ssesums);
         sums = buffer.tab[0];

         ssesumt = _mm_hadd_ps(ssesumt, ssesumt);
         ssesumt = _mm_hadd_ps(ssesumt, ssesumt);
         _mm_store1_ps(buffer.tab, ssesumt);
         sumt = buffer.tab[0];

         // Compute zncc value
         if (count == 0) continue;

         Rox_Double rcc = cc;
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

function_terminate:
   // rox_memory_delete(buf);
   return error;
}
#endif

#define ANSI
#ifdef ANSI

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
   Rox_Sint shiftwidth, shiftheight;
   Rox_Sint shiftx, shifty;

   Rox_Uchar ** ds;
   Rox_Uchar ** dt;
   Rox_Uint ** dsm;
   Rox_Uint ** dtm;

   Rox_Sint i, j;
   Rox_Float vt, vs;
   Rox_Sint count;
   Rox_Uint cc;

   Rox_Uint sums, sumt;
   Rox_Uint sumsqt, sumsqs;

   Rox_Double ratio;

   Rox_Double meant, means;
   Rox_Double rsumsqt, rsumsqs, nom, denoms, denomt, denom;
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

   error = rox_array2d_uint_get_data_pointer_to_pointer( &dsm, isearch_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_uint_get_data_pointer_to_pointer( &dtm, itemplate_mask);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &ds, isearch);
   ROX_ERROR_CHECK_TERMINATE ( error );
   error = rox_array2d_uchar_get_data_pointer_to_pointer( &dt, itemplate);
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
         sumt = 0;
         sums = 0;
         sumsqs = 0;
         sumsqt = 0;
         cc = 0;

         // Compute zncc per pixel components
         for (i = 0; i < theight; i++)
         {
            for (j = 0; j < twidth; j++)
            {
               // Mask handle
               if (!dsm[shifty + i][shiftx + j]) continue;
               if (!dtm[i][j]) continue;

               // Values
               vs = ds[shifty + i][shiftx + j];
               vt = dt[i][j];

               // Zncc parts
               count++;
               cc += (Rox_Uint) (vs * vt);
               sumt += (Rox_Uint) vt;
               sums += (Rox_Uint) vs;
               sumsqs += (Rox_Uint) (vs * vs);
               sumsqt += (Rox_Uint) (vt * vt);
            }
         }

         // Compute zncc value
         if (count == 0) continue;
         ratio = (Rox_Double) count / (Rox_Double) (twidth*theight);

         // Check if there is enough data to compute zncc
         if (ratio < MIN_VISIBLE_RATIO) continue;

         Rox_Double rcc = (Rox_Double) cc;
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
   *res_score = (Rox_Float) (0.5 * (bestscore + 1.0));
   *res_topleft_x = posbestscorex;
   *res_topleft_y = posbestscorey;

function_terminate:
   return error;
}
#endif