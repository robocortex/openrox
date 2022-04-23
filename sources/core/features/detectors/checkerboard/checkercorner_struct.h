//==============================================================================
//
//    OPENROX   : File checkercorner_struct.h
//
//    Contents  : API of checkercorner module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CHECKERCORNER__
#define __OPENROX_CHECKERCORNER__

#include <baseproc/geometry/point/point2d_struct.h>

//! Corner description
struct Rox_CheckerCorner_Struct
{
   //! Corner coordinates
   Rox_Point2D_Double_Struct coords;
   
   //! Corner first edge direction
   Rox_Point2D_Double_Struct edge1;
   
   //! Corner second edge direction
   Rox_Point2D_Double_Struct edge2;
};

//! Corner description
typedef struct Rox_CheckerCorner_Struct Rox_CheckerCorner_Struct;

#endif
