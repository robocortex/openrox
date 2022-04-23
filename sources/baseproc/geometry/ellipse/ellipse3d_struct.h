//==============================================================================
//
//    OPENROX   : File ellipse3d_struct.h
//
//    Contents  : Structures of 3D ellipse module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ELLIPSE3D_STRUCT__
#define __OPENROX_ELLIPSE3D_STRUCT__

#include <system/memory/datatypes.h>
#include <baseproc/maths/linalg/matse3.h>

//! \ingroup Geometry
//! \addtogroup Ellipse3D
//! @{

//! The 3D ellipse (in meters) 
//! Define a 3D ellipse in frame F as the tranformation by Te of a centered ellipse on the z=0 plane in frame Fe
//! A point me of the ellipse in frame Fe is transformed to a point m in frame F with m = Te * me 
//! The centered ellipse on plane z = 0 in the ellipse frame Fe has equations:
//! (x/a)² + (y/b)² = 1                            [algebraic]
//! [x;y] = [a * cos(theta); b * cos(theta)]       [trigonometric parametric]
struct Rox_Ellipse3D_Double_Struct
{
   //! The scaling factor along x-axis coordinate in meters
   Rox_Double a;

   //! The scaling factor along y-axis coordinate in meters
   Rox_Double b;

   //! The transformation matrix in SE3 between a frame F and the ellipse frame Fe (i.e. m = Te * me, with m in F and me in Fe)
   Rox_MatSE3 Te;
};

//! 3D ellipse structure
typedef struct Rox_Ellipse3D_Double_Struct Rox_Ellipse3D_Double_Struct;

//! define
#define ROX_TYPE_ELLIPSE3D_DOUBLE (sizeof(struct Rox_Ellipse3D_Double_Struct) << 2)

//! @}

#endif
