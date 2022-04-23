//==============================================================================
//
//    OPENROX   : File shicorner.h
//
//    Contents  : API of shicorner module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "shicorner.h"

#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_shicorner_test (
   Rox_Float *response, 
   Rox_Uchar ** image_data, 
   const Rox_Uint i, 
   const Rox_Uint j, 
   const Rox_Uint radius
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   
   if (!response || !image_data) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint idxx = 0;
   Rox_Sint idxy = 0;
   Rox_Sint idyy = 0;

   for (Rox_Uint y = i - radius; y <= i + radius; y++)
   {
      for (Rox_Uint x = j - radius; x <= j + radius; x++)
      {
         Rox_Sint dx = (Rox_Sint) image_data[y][x+1] - (Rox_Sint) image_data[y][x-1];
         Rox_Sint dy = (Rox_Sint) image_data[y+1][x] - (Rox_Sint) image_data[y-1][x];

         Rox_Sint ldxx = dx*dx;
         Rox_Sint ldyy = dy*dy;
         Rox_Sint ldxy = dx*dy;

         idxx += ldxx;
         idyy += ldyy;
         idxy += ldxy;
      }
   }

   Rox_Float dxx = (Rox_Float) idxx;
   Rox_Float dxy = (Rox_Float) idxy;
   Rox_Float dyy = (Rox_Float) idyy;

   Rox_Uint size = radius * 2 + 1;
   Rox_Float norm = 1.0f / (Rox_Float)(2 * size * size); // "2" is here for gradient normalization

   dxx = dxx * norm;
   dyy = dyy * norm;
   dxy = dxy * norm;

   // eigenvalues of the matrix [dxx dxy; dxy dyy] are :
   // l1 = dYY / 0.2e1 + dXX / 0.2e1 + sqrt(dYY * dYY - 0.2e1 * dYY * dXX + dXX * dXX + 0.4e1 * dXY * dXY) / 0.2e1;
   // l2 = dYY / 0.2e1 + dXX / 0.2e1 - sqrt(dYY * dYY - 0.2e1 * dYY * dXX + dXX * dXX + 0.4e1 * dXY * dXY) / 0.2e1;
   // l2 is obviously always less than l1 ! so l2 is min(l1,l2) as in shi paper

   *response = 0.5f * (dyy + dxx - sqrtf(dyy * dyy - 2.0f * dyy * dxx + dxx * dxx + 4 * dxy * dxy));

function_terminate:
   return error;
}