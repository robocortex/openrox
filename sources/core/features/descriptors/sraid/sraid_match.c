//==============================================================================
//
//    OPENROX   : File sraid_match.c
//
//    Contents  : Implementation of sraid_match module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "sraid_match.h"

Rox_Uint rox_sraid_match(Rox_Ushort * feat1, Rox_Ushort * feat2)
{
   Rox_Sint dist = 0;
   
   for ( Rox_Sint i = 0; i < 128; i++)
   {
      Rox_Sint diff = (Rox_Sint) feat1[i] - (Rox_Sint) feat2[i];
      dist += diff * diff;
   }

   return dist;
}