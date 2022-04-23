//============================================================================
//
//    OPENROX   : File ansi_region_zncc_search_mask_template_mask.c
//
//    Contents  : Implementation of region_zncc_search_mask_template_mask module in ANSI C
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "ansi_region_zncc_search_mask_template_mask.h"
#include <float.h>
#include <math.h>

int rox_ansi_array2d_float_region_zncc_search_mask_template_mask (
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

   int shiftwidth = swidth - twidth;
   int shiftheight = sheight - theight;
   
   Rox_Float vt, vs;
   Rox_Sint count;
   Rox_Double cc;
   Rox_Double sums, sumt, ratio;
   Rox_Double sumsqt, sumsqs;

   Rox_Double meant, means;
   Rox_Double rcc, rsumsqt, rsumsqs, nom, denoms, denomt, denom;
   Rox_Double curscore, bestscore;

   Rox_Sint posbestscorex, posbestscorey;

   // Worst possible zncc score is -1.0
   bestscore = -1.1;
   posbestscorex = 0;
   posbestscorey = 0;

   // Loop over possible "configurations"
   for (int shifty = 0; shifty < shiftheight; shifty++)
   {
      // Loop over possible "configurations"
      for (int shiftx = 0; shiftx < shiftwidth; shiftx++)
      {
         count = 0;
         sumt = 0.0;
         sums = 0.0;
         sumsqs = 0.0;
         sumsqt = 0.0;
         cc = 0.0;

         // Compute zncc per pixel components
         for (int i = 0; i < theight; i++)
         {
            for (int j = 0; j < twidth; j++)
            {
               // Mask handle
               if (!dsm[shifty + i][shiftx + j]) continue;
               if (!dtm[i][j]) continue;

               // Values
               vs = ds[shifty + i][shiftx + j];
               vt = dt[i][j];

               // Zncc parts
               count++;
               cc += (double) vs * vt;
               sumt += vt;
               sums += vs;
               sumsqs += (double) vs * vs;
               sumsqt += (double) vt * vt;
            }
         }

         // Compute zncc value
         if (count == 0) continue;
         ratio = (double) count / (double) (twidth*theight);

         // Check if there is enough data to compute zncc 
         if (ratio < MIN_VISIBLE_RATIO) continue;

         rcc = cc;
         rsumsqs = sumsqs;
         rsumsqt = sumsqt;
         meant = sumt / ((double) count);
         means = sums / ((double) count);
         nom = rcc - meant * means * (double) count;
         denoms = rsumsqs - means * means * (double) count;
         denomt = rsumsqt - meant * meant * (double) count;

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
 
   return error;
}

int rox_ansi_array2d_uchar_region_zncc_search_mask_template_mask (
   float * res_score,
   int * res_topleft_x,
   int * res_topleft_y,
   unsigned char ** ds,
   unsigned int ** dsm,
   int sheight,
   int swidth,
   unsigned char ** dt,
   unsigned int ** dtm,
   int theight,
   int twidth
)
{
   int error = 0;

   Rox_Float vt, vs;
   Rox_Sint count;
   Rox_Uint cc;

   Rox_Uint sums, sumt;
   Rox_Uint sumsqt, sumsqs;

   Rox_Double ratio;

   Rox_Double meant, means;
   Rox_Double rcc, rsumsqt, rsumsqs, nom, denoms, denomt, denom;
   Rox_Double curscore, bestscore;

   Rox_Sint posbestscorex, posbestscorey;

   int shiftwidth = swidth - twidth;
   int shiftheight = sheight - theight;
   
   // Worst possible zncc score is -1
   bestscore = -1.1;
   posbestscorex = 0;
   posbestscorey = 0;

   // Loop over possible "configurations"
   for (int shifty = 0; shifty < shiftheight; shifty++)
   {
      // Loop over possible "configurations"
      for (int shiftx = 0; shiftx < shiftwidth; shiftx++)
      {
         count = 0;
         sumt = 0.0;
         sums = 0.0;
         sumsqs = 0.0;
         sumsqt = 0.0;
         cc = 0.0;

         // Compute zncc per pixel components
         for (int i = 0; i < theight; i++)
         {
            for (int j = 0; j < twidth; j++)
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

   return error;
}
