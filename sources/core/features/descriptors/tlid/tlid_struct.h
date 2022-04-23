//============================================================================
//
//    OPENROX   : File tlid_struct.h
//
//    Contents  : API of tlid_objects module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_TLID_STRUCT__
#define __OPENROX_TLID_STRUCT__

#include <baseproc/geometry/point/point2d_struct.h>

//! \addtogroup TLID
//! @{

//! Structure
struct Rox_TLID_Segment_Struct
{
   //! points
   Rox_Point2D_Double_Struct points[2];
   //! direction
   Rox_Point2D_Double_Struct direction;
   //! midpoint
   Rox_Point2D_Double_Struct midpoint;
   //! desc
   Rox_Double desc[12*12*4];
};

//! TLID segment structure
typedef struct Rox_TLID_Segment_Struct Rox_TLID_Segment_Struct;

//! To be commented
#define ROX_TYPE_TLID_SEGMENT (sizeof(struct Rox_TLID_Segment_Struct) << 2)

//! @}

#endif