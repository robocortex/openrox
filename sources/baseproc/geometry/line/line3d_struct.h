//==============================================================================
//
//    OPENROX   : File line3d_struct.h
//
//    Contents  : Structure of line module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_LINE3D_STRUCT__
#define __OPENROX_LINE3D_STRUCT__

#include <system/memory/datatypes.h>
#include <baseproc/geometry/point/points_struct.h>
#include <baseproc/geometry/plane/plane_struct.h>

//! \ingroup Euclidean_Geometry
//! \defgroup Line Line

//!	\addtogroup Line
//!	@{

//! 2 points index line
struct Rox_Line3D_Index_Struct
{
   //! 3D meters point index
   Rox_Uint points[2];
};

//! 2 points index line structure
typedef struct Rox_Line3D_Index_Struct Rox_Line3D_Index_Struct;

//! define
#define ROX_TYPE_LINE3D_INDEX (sizeof(struct Rox_Line3D_Index_Struct) << 2)

//! 2 planes based line
struct Rox_Line3D_Planes_Struct
{
   //! Intersection of two 3d planes, must not be filled manually
   Rox_Plane3D_Double_Struct planes[2];
};

//! 2 planes based line structure
typedef struct Rox_Line3D_Planes_Struct Rox_Line3D_Planes_Struct;

//! define
#define ROX_TYPE_LINE3D_PLANES (sizeof(struct Rox_Line3D_Planes_Struct) << 2)

//! plucker based line
struct Rox_Line3D_Plucker_Struct
{
   //! Moment
   Rox_Double moment[3];

   //! displacement
   Rox_Double displacement[3];
};

//! plucker based line structure
typedef struct Rox_Line3D_Plucker_Struct Rox_Line3D_Plucker_Struct;

//! define
#define ROX_TYPE_LINE3D_PLUCKER (sizeof(struct Rox_Line3D_Plucker_Struct) << 2)

//! parametric based line
struct Rox_Line3D_Parametric_Struct
{
   //! Origin
   Rox_Double origin[3];

   //! Direction
   Rox_Double direction[3];
};

//! parametric based line structure
typedef struct Rox_Line3D_Parametric_Struct Rox_Line3D_Parametric_Struct;

//! define
#define ROX_TYPE_LINE3D_PARAMETRIC (sizeof(struct Rox_Line3D_Parametric_Struct) << 2)


//! @}

#endif
