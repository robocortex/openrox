//==============================================================================
//
//    OPENROX   : File plane_struct.h
//
//    Contents  : Structure of plane module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_PLANE_STRUCT__
#define __OPENROX_PLANE_STRUCT__

#include <system/memory/datatypes.h>

//! \ingroup Euclidean_Geometry
//! \defgroup Plane Plane

//! \addtogroup Plane
//! @{

//! Implicit representation [a, b, c, d]*[X;Y;Z;1]  = 0 
struct Rox_Plane3D_Double_Struct
{
   //! Implicit representation a coordinate
   Rox_Double a;
   //! Implicit representation b coordinate
   Rox_Double b;
   //! Implicit representation c coordinate
	Rox_Double c;
   //! Implicit representation d coordinate
   Rox_Double d;
};

//! Plane 3D double structure
typedef struct Rox_Plane3D_Double_Struct Rox_Plane3D_Double_Struct;

//! Plane 3D double pointer to structure
typedef struct Rox_Plane3D_Double_Struct * Rox_Plane3D_Double;

//! define 
#define ROX_TYPE_PLANE3D_DOUBLE (sizeof(struct Rox_Plane3D_Index_Struct) << 2)

//! @} 

#endif // __OPENROX_PLANE_STRUCT__
