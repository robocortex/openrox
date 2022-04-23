//==============================================================================
//
//    OPENROX   : File inverse_lu.c
//
//    Contents  : Implementation of inverse lu module
//
//    Author(s) : R&D department leaded by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "inverse_lu.h"
#include "svdinverse.h"
#include "pseudoinverse.h"

Rox_ErrorCode rox_array2d_double_inverse_lu(Rox_Array2D_Double Mi, Rox_Array2D_Double M)
{
   // TODO: implement ansi version, now implementd only using MKL libraries  
   return rox_array2d_double_svdinverse(Mi, M);
}