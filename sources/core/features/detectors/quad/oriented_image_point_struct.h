//==============================================================================
//
//    OPENROX   : File oriented_image_point_struct.h
//
//    Contents  : Structure of quad_detection module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_ORIENTED_IMAGE_POINT_STRUCT__
#define __OPENROX_ORIENTED_IMAGE_POINT_STRUCT__

#include <system/memory/datatypes.h>

//! \ingroup Vision
//! \addtogroup Quad
//! @{

//! To be commented
struct Rox_OrientedImagePoint_Struct
{
   //! Coordinate along u axis
   Rox_Uint i;

   //! Coordinate along v axis
   Rox_Uint j;

   //! Angle
   Rox_Double theta;

   //! Magnitude
   Rox_Double mag;
};

//! To be commented
typedef struct Rox_OrientedImagePoint_Struct Rox_OrientedImagePoint_Struct;

//! @}

#endif
