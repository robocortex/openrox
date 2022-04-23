//==============================================================================
//
//    OPENROX   : File point3d_struct.h
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

#ifndef __OPENROX_POINT3D_STRUCT__
#define __OPENROX_POINT3D_STRUCT__

#include <system/memory/datatypes.h>

//! \ingroup Geometry
//! \addtogroup Point3D
//! @{

//! 3D meters point
struct Rox_Point3D_Double_Struct
{
   //! The x-axis coordinate in meters
   Rox_Double X;

   //! The y-axis coordinate in meters
   Rox_Double Y;

   //! The z-axis coordinate in meters
   Rox_Double Z;
};

//! 3D meters point
struct Rox_Point3D_Sint_Struct
{
   //! The x-axis coordinate in meters
   Rox_Sint X;

   //! The y-axis coordinate in meters
   Rox_Sint Y;

   //! The z-axis coordinate in meters
   Rox_Sint Z;
};

//! 3D meters point structure
typedef struct Rox_Point3D_Sint_Struct Rox_Point3D_Sint_Struct;

//! define
#define ROX_TYPE_POINT3D_SINT (sizeof(struct Rox_Point3D_Sint_Struct) << 2)

//! 3D meters point structure
typedef struct Rox_Point3D_Double_Struct Rox_Point3D_Double_Struct;

//! define
#define ROX_TYPE_POINT3D_DOUBLE (sizeof(struct Rox_Point3D_Double_Struct) << 2)

//! 3D point float
struct Rox_Point3D_Float_Struct
{
   // 3d meters point
   //! The x-axis coordinate in meters
   Rox_Float X;

   //! The y-axis coordinate in meters
   Rox_Float Y;

   //! The z-axis coordinate in meters
   Rox_Float Z;
};

//! 3D point float structure
typedef struct Rox_Point3D_Float_Struct Rox_Point3D_Float_Struct;

//! define
#define ROX_TYPE_POINT3D_FLOAT (sizeof(struct Rox_Point3D_Float_Struct) << 2)

//! define
#define POINT3D_FLOAT_TO_DOUBLE(A,B) {A.X=B.X;A.Y=B.Y;A.Z=B.Z;}

//! @}

#endif
