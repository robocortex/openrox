//============================================================================
//
//    OPENROX   : File coordinate_systems.h
//
//    Contents  : API to define the coordinate system
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_COMMON_COORDINATE_SYSTEMS__
#define __OPENROX_COMMON_COORDINATE_SYSTEMS__

//!  \ingroup baseproc
//! \addtogroup geometry
//! @{


//!  Define the different 3d coordinates conventions supported by
//!  a method/function that has a pose as input/output
typedef enum Rox_3D_Coordinate_System
{
   Rox_3D_Coordinate_System_Undefined = 0,
   Rox_RightHanded_XRight_YUp,
   Rox_RightHanded_XRight_YDown,
} Rox_3D_Coordinate_System;


//! @}

#endif //__OPENROX_COMMON_COORDINATE_SYSTEMS__