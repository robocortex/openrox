//==============================================================================
//
//    OPENROX   : File cylinder2d_struct.h
//
//    Contents  : Structures of cylinder2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CYLINDER2D_STRUCT__
#define __OPENROX_CYLINDER2D_STRUCT__

#include <baseproc/geometry/ellipse/ellipse2d.h>
#include <baseproc/geometry/segment/segment2d.h>

//! \ingroup Geometry
//! \addtogroup Cylinder2D
//! @{

//! 2D cylinder (in normalized or pixel coordinates)
//! Define a 2D cylinder as the set of two visible segments and two ellipses
struct Rox_Cylinder2D_Double_Struct
{
   //! The cylinder visible segment 1
   Rox_Segment2D s1;

   //! The cylinder visible segment 2
   Rox_Segment2D s2;
   
   //! The cylinder visible ellipse 1
   Rox_Ellipse2D e1;
      
   //! The cylinder visible ellipse 2
   Rox_Ellipse2D e2;
};

//! 2D cylinder structure
typedef struct Rox_Cylinder2D_Double_Struct Rox_Cylinder2D_Double_Struct;

//! define
#define ROX_TYPE_CYLINDER2D_DOUBLE (sizeof(struct Rox_Cylinder2D_Double_Struct) << 2)

//! @}

#endif
