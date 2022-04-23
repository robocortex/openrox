//==============================================================================
//
//    OPENROX   : File triangle_struct.h
//
//    Contents  : API of triangle structure
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_TRIANGLE_STRUCT__
#define __OPENROX_TRIANGLE_STRUCT__

#include <system/memory/datatypes.h>
#include <baseproc/geometry/point/point3d_struct.h>

//! Triangle structure
struct Rox_Triangle_Double_Struct
{
   //! 3D points in meters
   Rox_Point3D_Double_Struct points[3];
};

//! Triangle structure
typedef struct Rox_Triangle_Double_Struct Rox_Triangle_Double_Struct;

#define ROX_TYPE_TRIANGLE_DOUBLE (sizeof(struct Rox_Triangle_Double_Struct) << 2)

//! Triangle structure
struct Rox_Triangle_Index_Struct
{
   //! 3D points indexes
   Rox_Uint points[3];
};

//! Triangle structure
typedef struct Rox_Triangle_Index_Struct Rox_Triangle_Index_Struct;

#define ROX_TYPE_TRIANGLE_INDEX (sizeof(struct Rox_Triangle_Index_Struct) << 2)

#endif
