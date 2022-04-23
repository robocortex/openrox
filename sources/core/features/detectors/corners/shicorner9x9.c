//==============================================================================
//
//    OPENROX   : File shicorner9x9.c
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

int rox_ansi_shicorner9x9_test (
   float * response, 
   unsigned char ** image_data, 
   unsigned int v, 
   unsigned int u
)
{
   int error = 0;
   const float norm = 1.0f / 162.0f;

   int idxx = 0;
   int idxy = 0;
   int idyy = 0;

   for ( int y = v - 4; y <= v + 4; y++)
   {
      for ( int x = u - 4; x <= u + 4; x++)
      {
         int dx = (int) image_data[y][x+1] - (int) image_data[y][x-1];
         int dy = (int) image_data[y+1][x] - (int) image_data[y-1][x];

         int ldxx = dx*dx;
         int ldyy = dy*dy;
         int ldxy = dx*dy;

         idxx += ldxx;
         idyy += ldyy;
         idxy += ldxy;
      }
   }

   float dxx = idxx;
   float dxy = idxy;
   float dyy = idyy;

   dxx = dxx * norm;
   dyy = dyy * norm;
   dxy = dxy * norm;

   // eigenvalues of the matrix [dxx dxy; dxy dyy] are :
   // l1 = dYY / 0.2e1 + dXX / 0.2e1 + sqrt(dYY * dYY - 0.2e1 * dYY * dXX + dXX * dXX + 0.4e1 * dXY * dXY) / 0.2e1;
   // l2 = dYY / 0.2e1 + dXX / 0.2e1 - sqrt(dYY * dYY - 0.2e1 * dYY * dXX + dXX * dXX + 0.4e1 * dXY * dXY) / 0.2e1;
   // l2 is obviously always less than l1 ! so l2 is min(l1,l2) as in shi paper

   *response = 0.5f * (dyy + dxx - sqrtf(dyy * dyy - 2.0f * dyy * dxx + dxx * dxx + 4 * dxy * dxy));
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
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error); }

   if ( v <= 8 || u <= 8 ) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_ansi_shicorner9x9_test ( response, image_data, v, u );
   ROX_ERROR_CHECK_TERMINATE(error);

function_terminate:
   return error;
}
