//==============================================================================
//
//    OPENROX   : File sraiddesc.c
//
//    Contents  : Implementation of sraiddesc module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sraiddesc.h"

#include <iso646.h>

#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/base/basemaths.h>
#include <generated/dynvec_sraiddesc_struct.h>

#include <inout/system/errors_print.h>

#define SRAID_DESCR_WIDTH       4
#define SRAID_DESCR_HIST_BINS   8
#define SRAID_ORI_HIST_BINS     36
#define SRAID_ORI_SMOOTH_PASSES 2
#define SRAID_ORI_SIG_FCTR      1.5f
#define SRAID_ORI_RADIUS        3.0f*SRAID_ORI_SIG_FCTR
#define SRAID_ORI_PEAK_RATIO    0.7f
#define SRAID_ORI_SMOOTH_PASSES 2
#define SRAID_DESCR_MAG_THR     0.2f
#define SRAID_DESCR_SCL_FCTR    3.0f
#define SRAID_INT_DESCR_FCTR    512.0f
#define interp_hist_peak( l, c, r ) ( 0.5f * ( (l)-(r) ) / ( (l) - 2.0f*(c) + (r) ) )
#define SRAID_DESC_SIZE  SRAID_DESCR_WIDTH*SRAID_DESCR_WIDTH*SRAID_DESCR_HIST_BINS

void interp_hist_entry(Rox_Float ***hist, Rox_Float rbin, Rox_Float cbin, Rox_Float obin, Rox_Float mag, int d, int n);

void interp_hist_entry(Rox_Float ***hist, Rox_Float rbin, Rox_Float cbin, Rox_Float obin, Rox_Float mag, int d, int n)
{
   Rox_Float d_r, d_c, d_o;
   int r0, c0, o0;
   int ob0, ob1;
   int rb0, rb1;
   int cb0, cb1;
   Rox_Float v0_1, v0_2;
   Rox_Float c0_1, c0_2;
   Rox_Float r0_1, r0_2;
   Rox_Float c0_1xv0_1, c0_1xv0_2, c0_2xv0_1, c0_2xv0_2;

   r0 = (rbin < 0) ? -1 : (int)rbin;
   c0 = (cbin < 0) ? -1 : (int)cbin;
   o0 = (obin < 0) ? -1 : (int)obin;
   d_r = rbin - r0;
   d_c = cbin - c0;
   d_o = obin - o0;

   v0_1 = 1.0f - d_o;
   v0_2 = d_o;
   c0_1 = 1.0f - d_c;
   c0_2 = d_c;
   r0_1 = mag * (1.0f - d_r);
   r0_2 = mag * d_r;

   c0_1xv0_1 = c0_1 * v0_1;
   c0_1xv0_2 = c0_1 * v0_2;
   c0_2xv0_1 = c0_2 * v0_1;
   c0_2xv0_2 = c0_2 * v0_2;

   rb0 = r0;
   rb1 = r0 + 1;
   cb0 = c0;
   cb1 = c0 + 1;
   ob0 = o0 % n;
   ob1 = (o0 + 1) % n;

   // The entry is distributed into up to 8 bins.  Each entry into a bin
   // is multiplied by a weight of 1 - d for each dimension, where d is the
   // distance from the center value of the bin measured in bin units.

   if (rb0 >= 0 && rb0 < d)
   {
      if (cb0 >= 0 && cb0 < d) hist[rb0][cb0][ob0] += r0_1 * c0_1xv0_1;
      if (cb0 >= 0 && cb0 < d) hist[rb0][cb0][ob1] += r0_1 * c0_1xv0_2;
      if (cb1 >= 0 && cb1 < d) hist[rb0][cb1][ob0] += r0_1 * c0_2xv0_1;
      if (cb1 >= 0 && cb1 < d) hist[rb0][cb1][ob1] += r0_1 * c0_2xv0_2;
   }
   if (rb1 >= 0 && rb1 < d)
   {
      if (cb0 >= 0 && cb0 < d) hist[rb1][cb0][ob0] += r0_2 * c0_1xv0_1;
      if (cb0 >= 0 && cb0 < d) hist[rb1][cb0][ob1] += r0_2 * c0_1xv0_2;
      if (cb1 >= 0 && cb1 < d) hist[rb1][cb1][ob0] += r0_2 * c0_2xv0_1;
      if (cb1 >= 0 && cb1 < d) hist[rb1][cb1][ob1] += r0_2 * c0_2xv0_2;
   }
}

Rox_Float ** *createFullHist()
{
   Rox_Float *** ret;
   Rox_Uint i, j;

   ret = (Rox_Float ** *)rox_memory_allocate(sizeof(Rox_Float **), SRAID_DESCR_WIDTH);
   for (i = 0; i < SRAID_DESCR_WIDTH; i++)
   {
      ret[i] = (Rox_Float **)rox_memory_allocate(sizeof(Rox_Float *), SRAID_DESCR_WIDTH);
      for (j = 0; j < SRAID_DESCR_WIDTH; j++)
      {
         ret[i][j] = (Rox_Float *)rox_memory_allocate(sizeof(Rox_Float), SRAID_DESCR_HIST_BINS);
      }
   }

   return ret;
}

Rox_Void deleteFullHist(Rox_Float *** ptr)
{
   Rox_Uint i, j;

   for (i = 0; i < SRAID_DESCR_WIDTH; i++)
   {
      for (j = 0; j < SRAID_DESCR_WIDTH; j++)
      {
         rox_memory_delete(ptr[i][j]);
      }
      rox_memory_delete(ptr[i]);
   }
   rox_memory_delete(ptr);
}


Rox_ErrorCode
rox_sraiddescriptor_process(
   Rox_DynVec_SRAID_Feature  sraidfeatures,
   Rox_Dog_Feature dogfeatures,
   Rox_Uint                         countdogfeatures,
   Rox_Array2D_Float_Collection     scalespace,
   Rox_Uint                         object_id)
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;
   const Rox_Uint size = SRAID_DESC_SIZE;
   Rox_Uint       idfeat;

   Rox_Sint rad, i, j, k, r, c, bin, iprev, inext;
   Rox_Sint cols = 0, rows = 0;
   Rox_Uint id;

   Rox_Float sigma, denom, mag, ori, w, prevhist, nexthist, tmphist, maxhist, magthresh, fbin;
   Rox_Float bins_per_radian, exp_denom, histogram_cols, cosori, sinori, ibin, jbin, obin, grad_mag, grad_ori;
   Rox_Float i_rot, j_rot, descriptor_norm, descriptor_normthresh, descval, ri, risquare, rj;
   Rox_Sint  radius, pos;
   Rox_Float dx, dy, orihistinv2PI, piinvorihist, isinori, icosori, halfdescrcols, invhistogramcols, invexpdenom;

   Rox_Float            histogram[SRAID_ORI_HIST_BINS];
   Rox_Float         ***fullhist;
   Rox_Float            descriptor_float[SRAID_DESC_SIZE];
   Rox_Array2D_Float    curimg;
   Rox_Double             possibleorientations[SRAID_ORI_HIST_BINS];
   Rox_Uint             countpossibleorientations;
   Rox_Float          **dimg;
   static int           val = 0;

   if (!dogfeatures) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   if (!scalespace) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   if (!sraidfeatures) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   fullhist = createFullHist();

   curimg = rox_array2d_float_collection_get(scalespace, 0);

   error = rox_array2d_float_get_size(&rows, &cols, curimg);
   ROX_ERROR_CHECK_TERMINATE(error);

   bins_per_radian = ((Rox_Float)SRAID_DESCR_HIST_BINS) / (Rox_Float) ROX_2PI;
   exp_denom = SRAID_DESCR_WIDTH;
   exp_denom = (Rox_Float)(exp_denom * exp_denom * 0.5);
   invexpdenom = 1.0f / exp_denom;
   orihistinv2PI = ((Rox_Float)SRAID_ORI_HIST_BINS) / (Rox_Float)ROX_2PI;
   piinvorihist = ((Rox_Float) ROX_2PI) / ((Rox_Float)SRAID_ORI_HIST_BINS);
   halfdescrcols = ((Rox_Float)(SRAID_DESCR_WIDTH / 2)) - 0.5f;

   // Loop over detected features
   for (idfeat = 0; idfeat < countdogfeatures; idfeat++)
   {
      sigma = dogfeatures[idfeat].octave_scale * SRAID_ORI_SIG_FCTR;
      rad = ROX_ROUND_FLOAT(dogfeatures[idfeat].octave_scale * SRAID_ORI_RADIUS);
      denom = 1.0f / (2.0f * sigma * sigma);

      curimg = rox_array2d_float_collection_get(scalespace, dogfeatures[idfeat].lvl);
      if (!curimg) continue;

      error = rox_array2d_float_get_data_pointer_to_pointer(&dimg, curimg);
      for (i = 0; i < SRAID_ORI_HIST_BINS; i++)
         histogram[i] = 0;

      // Compute histogram
      for (i = -rad; i <= rad; i++)
      {
         r = dogfeatures[idfeat].i + i;

         if ((r <= 0) || (r >= rows - 1))
            continue;

         ri = (Rox_Float)i;
         risquare = ri * ri;

         for (j = -rad; j <= rad; j++)
         {
            rj = (Rox_Float)j;
            c = dogfeatures[idfeat].j + j;

            if ((c <= 0) || (c >= cols - 1))
               continue;

            dx = dimg[r][c + 1] - dimg[r][c - 1];
            dy = dimg[r - 1][c] - dimg[r + 1][c];

            mag = (Rox_Float)(sqrt(dx*dx + dy*dy));
            ori = fast_atan2f2(dy, dx);
            w = fast_expfneg(-(risquare + rj * rj) * denom);

            bin = ROX_ROUND_DOUBLE(orihistinv2PI * (ori + ROX_PI));
            bin = (bin < SRAID_ORI_HIST_BINS) ? bin : 0;

            histogram[bin] += w * mag;
         }
      }

      // Smooth histogram
      for (j = 0; j < SRAID_ORI_SMOOTH_PASSES; j++)
      {
         prevhist = histogram[SRAID_ORI_HIST_BINS - 1];

         for (i = 0; i < SRAID_ORI_HIST_BINS; i++)
         {
            tmphist = histogram[i];
            histogram[i] = 0.25f*prevhist
               + 0.50f*histogram[i]
               + 0.25f*((i + 1 == SRAID_ORI_HIST_BINS) ? histogram[0] : histogram[i + 1]);
            prevhist = tmphist;
         }
      }

      // Search for maximum histogram value
      maxhist = histogram[0];
      for (i = 1; i < SRAID_ORI_HIST_BINS; i++)
         if (histogram[i] > maxhist)
            maxhist = histogram[i];

      magthresh = maxhist * SRAID_ORI_PEAK_RATIO;

      // Find peaks
      countpossibleorientations = 0;
      for (id = 0; id < SRAID_ORI_HIST_BINS; id++)
      {
         iprev = (0 == id) ? (SRAID_ORI_HIST_BINS - 1) : (id - 1);
         inext = ((SRAID_ORI_HIST_BINS - 1) == id) ? (0) : (id + 1);

         prevhist = histogram[iprev];
         nexthist = histogram[inext];

         if (histogram[id] < magthresh) continue;
         if (histogram[id] <= prevhist) continue;
         if (histogram[id] <= nexthist) continue;

         fbin = ((Rox_Float)id) + interp_hist_peak(histogram[iprev], histogram[id], histogram[inext]);
         if (fbin < 0)
         {
            fbin = ((Rox_Float)SRAID_ORI_HIST_BINS) + fbin;
         }
         else if (fbin >= SRAID_ORI_HIST_BINS)
         {
            fbin = fbin - ((Rox_Float)SRAID_ORI_HIST_BINS);
         }

         possibleorientations[countpossibleorientations] = (fbin * piinvorihist) - ROX_PI;
         countpossibleorientations++;
      }

      val = 0;
      val += countpossibleorientations;

      // Compute description histogram
      histogram_cols = SRAID_DESCR_SCL_FCTR * dogfeatures[idfeat].octave_scale;
      invhistogramcols = 1.0f / histogram_cols;
      radius = (Rox_Sint) (histogram_cols * sqrtf(2.0f) * (SRAID_DESCR_WIDTH + 1.0) * 0.5 + 0.5);

      // Several possible peaks
      for (id = 0; id < countpossibleorientations; id++)
      {
         ori = (Rox_Float) possibleorientations[id];

         cosori = cosf(ori);
         sinori = sinf(ori);

         for (i = 0; i < SRAID_DESCR_WIDTH; i++)
            for (j = 0; j < SRAID_DESCR_WIDTH; j++)
               for (k = 0; k < SRAID_DESCR_HIST_BINS; k++)
                  fullhist[i][j][k] = 0;

         // For each pixel of the region
         for (i = -radius; i <= radius; i++)
         {
            isinori = ((Rox_Float)i) * sinori;
            icosori = ((Rox_Float)i) * cosori;

            r = dogfeatures[idfeat].i + i;

            if (r <= 0 || r >= rows - 1)
               continue;

            for (j = -radius; j <= radius; j++)
            {
               // rotate
               j_rot = (((Rox_Float)j) * cosori - isinori) * invhistogramcols;
               i_rot = (((Rox_Float)j) * sinori + icosori) * invhistogramcols;
               ibin = i_rot + halfdescrcols;
               jbin = j_rot + halfdescrcols;

               if (!(ibin > -1.0 && ibin < SRAID_DESCR_WIDTH && jbin > -1.0 && jbin < SRAID_DESCR_WIDTH))
                  continue;

               c = dogfeatures[idfeat].j + j;

               if (c <= 0 || c >= cols - 1)
                  continue;

               dx = dimg[r][c + 1] - dimg[r][c - 1];
               dy = dimg[r - 1][c] - dimg[r + 1][c];
               grad_mag = (dx*dx + dy*dy);
               grad_ori = fast_atan2f2(dy, dx);

               grad_ori -= ori;
               if (grad_ori < 0.0)      grad_ori += (Rox_Float)ROX_2PI;
               if (grad_ori >= ROX_2PI) grad_ori -= (Rox_Float)ROX_2PI;

               obin = grad_ori * bins_per_radian;
               w = fast_expfneg(-(j_rot*j_rot + i_rot*i_rot) * invexpdenom);

               interp_hist_entry(fullhist, ibin, jbin, obin, grad_mag * w, SRAID_DESCR_WIDTH, SRAID_DESCR_HIST_BINS);
            }
         }

         // Set descriptor float values
         pos = 0;
         descriptor_norm = 0;
         for (i = 0; i < SRAID_DESCR_WIDTH; i++)
            for (j = 0; j < SRAID_DESCR_WIDTH; j++)
               for (k = 0; k < SRAID_DESCR_HIST_BINS; k++)
               {
                  descriptor_float[pos] = fullhist[i][j][k];
                  descriptor_norm += descriptor_float[pos] * descriptor_float[pos];
                  pos++;
               }

         descriptor_norm = (Rox_Float)(1.0 / sqrt(descriptor_norm));

         // Thresholding
         descriptor_normthresh = 0;
         for (i = 0; i < size; i++)
         {
            descval = descriptor_float[i] * descriptor_norm;
            if (descval > SRAID_DESCR_MAG_THR)
               descval = SRAID_DESCR_MAG_THR;
            descriptor_normthresh += descval * descval;
            descriptor_float[i] = descval;
         }

         // normalize and convert to int
         descriptor_normthresh = (Rox_Float)(1.0 / sqrtf(descriptor_normthresh));
         for (i = 0; i < size; i++)
         {
            descriptor_float[i] = (Rox_Float)((int)(SRAID_INT_DESCR_FCTR * descriptor_float[i] * descriptor_normthresh));
            sraidfeatures->data[sraidfeatures->used].descriptor[i] = (Rox_Ushort)ROX_MIN(255.0, descriptor_float[i]);
         }

         sraidfeatures->data[sraidfeatures->used].x = dogfeatures[idfeat].x;
         sraidfeatures->data[sraidfeatures->used].y = dogfeatures[idfeat].y;
         sraidfeatures->data[sraidfeatures->used].is_minimal = dogfeatures[idfeat].isminimal;
         sraidfeatures->data[sraidfeatures->used].ori = ori;
         sraidfeatures->data[sraidfeatures->used].object_id = object_id;
         sraidfeatures->data[sraidfeatures->used].level = dogfeatures[idfeat].lvl;

         rox_dynvec_sraiddesc_usecells(sraidfeatures, 1);
      }
   }

   deleteFullHist(fullhist);

function_terminate:
   return error;
}


Rox_ErrorCode rox_sraiddescriptor_process_fixed(
   Rox_DynVec_SRAID_Feature      sraidfeatures,
   Rox_Dog_Feature dogfeatures,
   Rox_Uint                      countdogfeatures,
   Rox_Array2D_Float_Collection  scalespace,
   Rox_Uint                      object_id)
{
   Rox_ErrorCode  error = ROX_ERROR_NONE;
   const Rox_Uint size = SRAID_DESC_SIZE;
   Rox_Uint       idfeat;

   Rox_Sint rad, i, j, k, r, c, bin, iprev, inext;
   Rox_Sint cols = 0, rows = 0;
   Rox_Uint id;

   Rox_Float sigma, denom, mag, ori, w, prevhist, tmphist, maxhist, fbin;
   //Rox_Float magthresh;
   Rox_Float bins_per_radian, exp_denom, histogram_cols, cosori, sinori, ibin, jbin, obin, grad_mag, grad_ori;
   Rox_Float i_rot, j_rot, descriptor_norm, descriptor_normthresh, descval, ri, risquare, rj;
   Rox_Sint  radius, pos;
   Rox_Float dx, dy, orihistinv2PI, piinvorihist, isinori, icosori, halfdescrcols, invhistogramcols, invexpdenom;

   Rox_Float            histogram[SRAID_ORI_HIST_BINS];
   Rox_Float         ***fullhist;
   Rox_Float            descriptor_float[SRAID_DESC_SIZE];
   Rox_Array2D_Float    curimg;
   Rox_Double             possibleorientations[SRAID_ORI_HIST_BINS];
   Rox_Uint             countpossibleorientations;
   Rox_Float          **dimg;
   static int           val = 0;

   if (!dogfeatures) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   if (!scalespace) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }
   if (!sraidfeatures) { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   fullhist = createFullHist();

   curimg = rox_array2d_float_collection_get(scalespace, 0);

   error = rox_array2d_float_get_size(&rows, &cols, curimg);
   ROX_ERROR_CHECK_TERMINATE(error);

   bins_per_radian = ((Rox_Float)SRAID_DESCR_HIST_BINS) / (Rox_Float) ROX_2PI;
   exp_denom = SRAID_DESCR_WIDTH;
   exp_denom = (Rox_Float)(exp_denom * exp_denom * 0.5);
   invexpdenom = 1.0f / exp_denom;
   orihistinv2PI = ((Rox_Float)SRAID_ORI_HIST_BINS) / (Rox_Float) ROX_2PI;
   piinvorihist = (Rox_Float)(ROX_2PI / ((Rox_Float)SRAID_ORI_HIST_BINS));
   halfdescrcols = ((Rox_Float)(SRAID_DESCR_WIDTH / 2)) - 0.5f;

   // Loop over detected features
   for (idfeat = 0; idfeat < countdogfeatures; idfeat++)
   {
      sigma = dogfeatures[idfeat].octave_scale * SRAID_ORI_SIG_FCTR;
      rad = ROX_ROUND_FLOAT(dogfeatures[idfeat].octave_scale * SRAID_ORI_RADIUS);
      denom = 1.0f / (2.0f * sigma * sigma);

      curimg = rox_array2d_float_collection_get(scalespace, dogfeatures[idfeat].lvl);
      if (!curimg) continue;

      error = rox_array2d_float_get_data_pointer_to_pointer(&dimg, curimg);

      for (i = 0; i < SRAID_ORI_HIST_BINS; i++)
         histogram[i] = 0;

      // Compute histogram
      for (i = -rad; i <= rad; i++)
      {
         r = dogfeatures[idfeat].i + i;

         if ((r <= 0) || (r >= rows - 1))
            continue;

         ri = (Rox_Float)i;
         risquare = ri * ri;

         for (j = -rad; j <= rad; j++)
         {
            rj = (Rox_Float)j;
            c = dogfeatures[idfeat].j + j;

            if ((c <= 0) || (c >= cols - 1))
               continue;

            dx = dimg[r][c + 1] - dimg[r][c - 1];
            dy = dimg[r - 1][c] - dimg[r + 1][c];

            mag = (Rox_Float)sqrt(dx*dx + dy*dy);
            ori = fast_atan2f2(dy, dx);
            w = fast_expfneg(-(risquare + rj * rj) * denom);

            bin = ROX_ROUND_DOUBLE(orihistinv2PI * (ori + ROX_PI));
            bin = (bin < SRAID_ORI_HIST_BINS) ? bin : 0;

            histogram[bin] += w * mag;
         }
      }

      // Smooth histogram
      for (j = 0; j < SRAID_ORI_SMOOTH_PASSES; j++)
      {
         prevhist = histogram[SRAID_ORI_HIST_BINS - 1];

         for (i = 0; i < SRAID_ORI_HIST_BINS; i++)
         {
            tmphist = histogram[i];
            histogram[i] = 0.25f*prevhist
               + 0.50f*histogram[i]
               + 0.25f*((i + 1 == SRAID_ORI_HIST_BINS) ? histogram[0] : histogram[i + 1]);
            prevhist = tmphist;
         }
      }

      // Search for maximum histogram value
      int max_id = 0;
      maxhist = histogram[0];
      for (i = 1; i < SRAID_ORI_HIST_BINS; i++)
         if (histogram[i] > maxhist)
         {
            maxhist = histogram[i];
            max_id = i;
         }

      // magthresh = maxhist * SRAID_ORI_PEAK_RATIO;

      // Only one peak -> the "best"
      countpossibleorientations = 0;
      id = max_id;
      {
         iprev = (0 == id) ? (SRAID_ORI_HIST_BINS - 1) : (id - 1);
         inext = ((SRAID_ORI_HIST_BINS - 1) == id) ? (0) : (id + 1);

         fbin = ((Rox_Float)id) + interp_hist_peak(histogram[iprev], histogram[id], histogram[inext]);
         if (fbin < 0)
         {
            fbin = ((Rox_Float)SRAID_ORI_HIST_BINS) + fbin;
         }
         else if (fbin >= SRAID_ORI_HIST_BINS)
         {
            fbin = fbin - ((Rox_Float)SRAID_ORI_HIST_BINS);
         }

         possibleorientations[countpossibleorientations] = (fbin * piinvorihist) - ROX_PI;
         countpossibleorientations++;
      }

      val = 0;
      val += countpossibleorientations;

      // Compute description histogram
      histogram_cols = SRAID_DESCR_SCL_FCTR * dogfeatures[idfeat].octave_scale;
      invhistogramcols = 1.0f / histogram_cols;
      radius = (Rox_Sint)(histogram_cols * sqrtf(2.0f) * (SRAID_DESCR_WIDTH + 1.0) * 0.5 + 0.5);

      // Several possible peaks
      for (id = 0; id < countpossibleorientations; id++)
      {
         ori = (Rox_Float)possibleorientations[id];

         cosori = cosf(ori);
         sinori = sinf(ori);

         for (i = 0; i < SRAID_DESCR_WIDTH; i++)
            for (j = 0; j < SRAID_DESCR_WIDTH; j++)
               for (k = 0; k < SRAID_DESCR_HIST_BINS; k++)
                  fullhist[i][j][k] = 0;

         // For each pixel of the region
         for (i = -radius; i <= radius; i++)
         {
            isinori = ((Rox_Float)i) * sinori;
            icosori = ((Rox_Float)i) * cosori;

            r = dogfeatures[idfeat].i + i;

            if (r <= 0 || r >= rows - 1)
               continue;

            for (j = -radius; j <= radius; j++)
            {
               // rotate
               j_rot = (((Rox_Float)j) * cosori - isinori) * invhistogramcols;
               i_rot = (((Rox_Float)j) * sinori + icosori) * invhistogramcols;
               ibin = i_rot + halfdescrcols;
               jbin = j_rot + halfdescrcols;

               if (!(ibin > -1.0 && ibin < SRAID_DESCR_WIDTH && jbin > -1.0 && jbin < SRAID_DESCR_WIDTH))
                  continue;

               c = dogfeatures[idfeat].j + j;

               if (c <= 0 || c >= cols - 1)
                  continue;

               dx = dimg[r][c + 1] - dimg[r][c - 1];
               dy = dimg[r - 1][c] - dimg[r + 1][c];
               grad_mag = (dx*dx + dy*dy);
               grad_ori = fast_atan2f2(dy, dx);

               grad_ori -= ori;
               if (grad_ori < 0.0)      grad_ori += (Rox_Float)ROX_2PI;
               if (grad_ori >= ROX_2PI) grad_ori -= (Rox_Float)ROX_2PI;

               obin = grad_ori * bins_per_radian;
               w = fast_expfneg(-(j_rot*j_rot + i_rot*i_rot) * invexpdenom);

               interp_hist_entry(fullhist, ibin, jbin, obin, grad_mag * w, SRAID_DESCR_WIDTH, SRAID_DESCR_HIST_BINS);
            }
         }

         // Set descriptor float values
         pos = 0;
         descriptor_norm = 0;
         for (i = 0; i < SRAID_DESCR_WIDTH; i++)
            for (j = 0; j < SRAID_DESCR_WIDTH; j++)
               for (k = 0; k < SRAID_DESCR_HIST_BINS; k++)
               {
                  descriptor_float[pos] = fullhist[i][j][k];
                  descriptor_norm += descriptor_float[pos] * descriptor_float[pos];
                  pos++;
               }

         descriptor_norm = (Rox_Float)(1.0 / sqrt(descriptor_norm));

         // Thresholding
         descriptor_normthresh = 0;
         for (i = 0; i < size; i++)
         {
            descval = descriptor_float[i] * descriptor_norm;
            if (descval > SRAID_DESCR_MAG_THR)
               descval = SRAID_DESCR_MAG_THR;
            descriptor_normthresh += descval * descval;
            descriptor_float[i] = descval;
         }

         // normalize and convert to int
         descriptor_normthresh = (Rox_Float)(1.0 / sqrtf(descriptor_normthresh));
         for (i = 0; i < size; i++)
         {
            descriptor_float[i] = (Rox_Float)((int)(SRAID_INT_DESCR_FCTR * descriptor_float[i] * descriptor_normthresh));
            sraidfeatures->data[sraidfeatures->used].descriptor[i] = (Rox_Ushort)ROX_MIN(255.0, descriptor_float[i]);
         }

         sraidfeatures->data[sraidfeatures->used].x = dogfeatures[idfeat].x;
         sraidfeatures->data[sraidfeatures->used].y = dogfeatures[idfeat].y;
         sraidfeatures->data[sraidfeatures->used].is_minimal = dogfeatures[idfeat].isminimal;
         sraidfeatures->data[sraidfeatures->used].ori = ori;
         sraidfeatures->data[sraidfeatures->used].object_id = object_id;
         sraidfeatures->data[sraidfeatures->used].level = dogfeatures[idfeat].lvl;

         rox_dynvec_sraiddesc_usecells(sraidfeatures, 1);
      }
   }

   deleteFullHist(fullhist);

function_terminate:
   return error;
}
