//==============================================================================
//
//    OPENROX   : File point3d.h
//
//    Contents  : API of point3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_POINT3D__
#define __OPENROX_POINT3D__

#include <system/memory/datatypes.h>

//! \ingroup Geometry
//! \addtogroup Point3D
//! @{

//! 3D meters point pointer to structure
typedef struct Rox_Point3D_Sint_Struct * Rox_Point3D_Sint;

//! 3D meters point typedef
typedef struct Rox_Point3D_Double_Struct * Rox_Point3D_Double;

//! 3D point float pointer to structure
typedef struct Rox_Point3D_Float_Struct * Rox_Point3D_Float;

//! define
#define POINT3D_FLOAT_TO_DOUBLE(A,B) {A.X=B.X;A.Y=B.Y;A.Z=B.Z;}

//! @}

#endif // __OPENROX_POINT3D__
