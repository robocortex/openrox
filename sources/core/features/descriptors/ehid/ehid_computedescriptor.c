//==============================================================================
//
//    OPENROX   : File ehid_computedescriptor.c
//
//    Contents  : Implementation of ehid_computedescriptor module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "ehid.h"

#include <baseproc/maths/maths_macros.h>
#include <generated/dynvec_ehid_point_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_ehid_point_quantizepatch (
   Rox_Uchar * out_mean, 
   Rox_Uchar output[64], 
   Rox_Double lums[64]
)
{
   Rox_Double dif;
   Rox_Double sum;
   Rox_Double mean;
   Rox_Double stddev;
   Rox_Double bounds[4];

   // Bounds as described in chapter 3 of the paper
   bounds[0] = -0.841621233572914;
   bounds[1] = -0.253347103135800;
   bounds[2] =  0.253347103135800;
   bounds[3] =  0.841621233572915;

   // Compute mean of patch
   sum = 0;
   for ( Rox_Sint i = 0; i < 64 ; i++ )
   {
      sum += lums[i];
   }
   mean = sum / 64.0;

   // Compute variance of patch
   sum = 0;
   for ( Rox_Sint i = 0; i < 64; i++ )
   {
      dif = (lums[i] - mean);
      sum += dif * dif;
   }

   // Compute standard deviation for patch
   stddev = sqrt(sum / 63.0);

   // Compute bounds for each histogram interval
   bounds[0] = mean + bounds[0]*stddev;
   bounds[1] = mean + bounds[1]*stddev;
   bounds[2] = mean + bounds[2]*stddev;
   bounds[3] = mean + bounds[3]*stddev;

   // Assign quantized histogram
   for ( Rox_Sint i = 0; i < 64; i++ )
   {
      if (lums[i] < bounds[0]) output[i] = 0;
      else if (lums[i] < bounds[1]) output[i] = 1;
      else if (lums[i] < bounds[2]) output[i] = 2;
      else if (lums[i] < bounds[3]) output[i] = 3;
      else output[i] = 4;
   }

   *out_mean = (Rox_Uchar)(0.5 + mean);

   return ROX_ERROR_NONE;
}

Rox_ErrorCode rox_ehid_points_computedescriptor(Rox_DynVec_Ehid_Point ptr, Rox_Image source)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uchar ** source_data = NULL;

   Rox_Ehid_Point  pt = NULL;

   Rox_Uint n;
   Rox_Double baseu, basev;
   Rox_Double cth, sth;

   Rox_Sint i,j,iu,iv;
   Rox_Double u, v, dx, dy, fval;
   Rox_Uint pos;
   Rox_Double I00, I01, I10, I11;
   Rox_Uchar val, index, mean;

   Rox_Uchar blums[64], center;
   Rox_Double lums[64];
   Rox_Uint desc[64][5];

   if (!ptr || !source) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Source image information
   error = rox_array2d_uchar_get_data_pointer_to_pointer(&source_data, source);
   ROX_ERROR_CHECK_TERMINATE ( error );

   // For each point, compute descriptor
   for (n = 0; n < ptr->used; n++)
   {
      pt = &ptr->data[n];

      // Retrieve point parameters
      baseu = pt->pos.u;
      basev = pt->pos.v;
      iu = (Rox_Sint)baseu;
      iv = (Rox_Sint)basev;
      center = source_data[iv][iu];

      // Direction normalized equal to cos/sin
      cth = pt->dir.u;
      sth = pt->dir.v;

      pos = 0;
      // Retrieve luminosity for selected points in patch around feature
      for (i = -7; i <= 7; i+=2)
      {
         for (j = -7; j <= 7; j+=2)
         {
            // rotate point around corner, using std 2D rotation and computed orientation
            u = cth*(double)j - sth*(double)i;
            v = sth*(double)j + cth*(double)i;

            // Add corner coordinates to retrieve pixel
            u = baseu + u;
            v = basev + v;

            // Retrieve pixel value and put it in the descriptor
            iu = (Rox_Sint)u;
            iv = (Rox_Sint)v;

            // Bilinear interpolation
            dx = u - (Rox_Double)iu;
            dy = v - (Rox_Double)iv;
            I00 = source_data[iv][iu];
            I01 = source_data[iv][iu + 1];
            I10 = source_data[iv + 1][iu];
            I11 = source_data[iv + 1][iu + 1];
            fval = (I00 + dx * (I01 - I00)) + dy * ((I10 + dx * (I11 - I10)) - (I00 + dx * (I01 - I00)));
            if (fval < 0) fval = 0;
            if (fval > 255.0) fval = 255.0;

            // Assign luminosity
            lums[pos] = fval;
            pos++;
         }
      }

      // Quantize luminosity in 5 values
      rox_ehid_point_quantizepatch(&mean, blums, lums);

      // Fill up "histogram" given quantized patch
      for (i = 0; i < 64; i++)
      {
         desc[i][0] = 0;
         desc[i][1] = 0;
         desc[i][2] = 0;
         desc[i][3] = 0;
         desc[i][4] = 0;

         val = blums[i];
         // Histogram is 1 on the quantized value interval
         desc[i][val] = 1;
      }

      index = 0;
      if (lums[10] > mean) index = 1;
      if (lums[15] > mean) index |= 2;
      if (lums[49] > mean) index |= 4;
      if (lums[54] > mean) index |= 8;
      if (center > mean) index |= 16;
      pt->index = index;

      // Transform histogram to binary descriptor (we want a binary descriptor for fast computation)
      rox_ehid_description_from_int_to_bits(pt->Description, desc);
   }

function_terminate:
   return error;
}
