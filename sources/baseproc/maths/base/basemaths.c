//==============================================================================
//
//    OPENROX   : File basemaths.c
//
//    Contents  : Implementation of basemaths module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "basemaths.h"

Rox_Uint _rox_popcount_slow(Rox_Uint v)
{
   // Should never be used when possible ... Very slow. Try to find a popcount dedicated to the aimed platform
   Rox_Uint retVal;
   retVal = 0;

   while (v)
   {
      retVal++;
      v &= v - 1;
   }

   return (retVal);
}

Rox_Uint rox_factorial(Rox_Uint x)
{
  Rox_Uint y = 1;
  
  for(Rox_Uint i = 1; i <= x; ++i)
  {
     y *= i;
  }
  
  return y;
}

Rox_Uint rox_array2d_coordinates_from_index(Rox_Uint * r, Rox_Uint * c, Rox_Uint cols, Rox_Uint index)
{
  *c = index % cols;
  *r = (index - *c)/cols;

  return 0;
}

Rox_Uint rox_array2d_index_from_coordinates(Rox_Uint * index, Rox_Uint cols, Rox_Uint r, Rox_Uint c)
{
  *index = r * cols + c;

  return 0;
}

Rox_ErrorCode rox_angle_isinrange ( 
   Rox_Sint * inrange, 
   const Rox_Double angle_model, 
   const Rox_Double angle_measure, 
   const Rox_Double angle_range
)
{
   Rox_Double error1 = fabs(angle_measure - angle_model);
   Rox_Double error2 = fabs(angle_measure - angle_model + 2*ROX_PI);
   Rox_Double error3 = fabs(angle_measure - angle_model - 2*ROX_PI);
   Rox_Double error4 = fabs(angle_measure - angle_model +   ROX_PI);
   Rox_Double error5 = fabs(angle_measure - angle_model -   ROX_PI);

   if ((error1 < angle_range) || (error2 < angle_range) || (error3 < angle_range) || (error4 < angle_range) || (error5 < angle_range) )
   {
      *inrange = 1;
   }
   else
   {
      *inrange = 0;
   }

   return ROX_ERROR_NONE;
}
