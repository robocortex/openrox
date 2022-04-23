//==============================================================================
//
//    OPENROX   : File brief_point_struct.h
//
//    Contents  : Struct of brief_point module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BRIEF_POINT_STRUCT__
#define __OPENROX_BRIEF_POINT_STRUCT__

#include <system/memory/datatypes.h>

//! \addtogroup BRIEF
//! @{

//! Indices for a point 
struct Rox_Brief_Point_Struct
{
   //! image coordinates along the u axis 
   Rox_Double u;
   //! image coordinates along the v axis 
   Rox_Double v;
   //! keypoint description 
   Rox_Uchar description[32];
};

//! @} 

#endif
