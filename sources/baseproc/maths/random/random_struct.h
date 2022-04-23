//==============================================================================
//
//    OPENROX   : File random_struct.h
//
//    Contents  : API of random module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_RANDOM_STRUCT__
#define __OPENROX_RANDOM_STRUCT__

#include <system/memory/datatypes.h>

struct Rox_Random_Struct
{
   //! Seed
   Rox_Sint seed;
   
   //! Parameter a
   Rox_Sint a;

   //! Parameter b 
   Rox_Sint b;
   
   //! Parameter m 
   Rox_Sint m;
};

#endif