//==============================================================================
//
//    OPENROX   : File integralaccess.c
//
//    Contents  : Implementation of integralaccess module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "integralaccess.h"

Rox_Slint rox_array2d_slint_integralaccess ( Rox_Slint ** integral, const int x, const int y, const int w, const int h )
{
   // Get value for a given rect using a integral image as input
   // f(x,y,w,h) = f(x,y) + f(x+w,y+h) - f(x, y+h) - f(x + w, y)
   // Un-reachable f(.,.) are considered null
   Rox_Slint output = integral[y + h - 1][x + w - 1];

   if (y > 0)
   {
      if (x > 0)
      {
         output += integral[y - 1][x - 1];
      }

      output -= integral[y - 1][x + w - 1];
   }

   if (x > 0)
   {
      output -= integral[y + h - 1][x - 1];
   }

   return output;
}
