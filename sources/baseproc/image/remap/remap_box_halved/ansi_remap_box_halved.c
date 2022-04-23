//============================================================================
//
//    OPENROX   : File ansi_remap_box_halved.c
//
//    Contents  : Implementation of ansi_remap_box_halved module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#include "ansi_remap_box_halved.h"

int rox_ansi_remap_box_nomask_uchar_to_uchar_halved (
   unsigned char ** dd,
   unsigned char ** ds,
   int hrows,
   int hcols
)
{
   int error = 0;
   for ( int i = 0; i < hrows; i++ )
   {
      int di = i * 2;
      for ( int j = 0; j < hcols; j++ )
      {
         int dj = j * 2;
         unsigned short buffer = (unsigned short) ds[di][dj];
         // Compute average gray level over the 4 neirest neighboor pixels
         buffer += (unsigned short) ds[di][dj + 1];
         buffer += (unsigned short) ds[di + 1][dj];
         buffer += (unsigned short) ds[di + 1][dj + 1];
         buffer = buffer / 4;
         dd[i][j] = (unsigned char) buffer;
      }
   }
   return error;
}


int rox_ansi_remap_box_nomask_float_to_float_halved (
   float ** dd,
   float ** ds,
   int hrows,
   int hcols
)
{
   int error = 0;

   for ( int i = 0; i < hrows; i++ )
   {
      int di = 2*i;
      for ( int j = 0; j < hcols; j++)
      {
         int dj = 2*j;

         dd[i][j] = (ds[di][dj] + ds[di + 1][dj] + ds[di][dj + 1] + ds[di + 1][dj + 1]) / 4.0f;
      }
   }

   return error;
}
