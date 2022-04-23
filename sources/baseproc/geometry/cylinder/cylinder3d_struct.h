//==============================================================================
//
//    OPENROX   : File cylinder3d_struct.h
//
//    Contents  : Structures of cylinder3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_CYLINDER3D_STRUCT__
#define __OPENROX_CYLINDER3D_STRUCT__

#include <baseproc/maths/linalg/matse3.h>

//! \ingroup Geometry
//! \addtogroup Cylinder3D
//! @{

//! The 3D cylinder (in meters) 
//! Define a 3D cylinder in frame Fo as the tranformation by oTe of two centered 3D ellipses on the z = -h/2 and z = +h/2 planes in frame Fe
struct Rox_Cylinder3D_Double_Struct
{
   //! The scaling factor along x-axis coordinate in meters
   Rox_Double a;

   //! The scaling factor along y-axis coordinate in meters
   Rox_Double b;
   
   //! The height of the cylinder
   Rox_Double h;

   //! The transformation matrix in SE3 between the object frame F_obj and the cylinder frame F_cyl : T = obj_T_cyl
   Rox_MatSE3 T;
};

//! 3D cylinder structure
typedef struct Rox_Cylinder3D_Double_Struct Rox_Cylinder3D_Double_Struct;

//! define
#define ROX_TYPE_CYLINDER3D_DOUBLE (sizeof(struct Rox_Cylinder3D_Double_Struct) << 2)

//! @}

#endif
