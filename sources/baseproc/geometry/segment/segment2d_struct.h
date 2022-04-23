//==============================================================================
//
//    OPENROX   : File segment2d_struct.h
//
//    Contents  : Structure of segment2d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SEGMENT2D_STRUCT__
#define __OPENROX_SEGMENT2D_STRUCT__

#include <system/memory/datatypes.h>
#include <baseproc/geometry/point/point2d_struct.h>

//! \ingroup Euclidean_Geometry
//! \defgroup Segment Segment

//! \addtogroup Segment
//! @{

//! Segment 2D double structure
struct Rox_Segment2D_Struct
{
   //! The 2 extremal points of the segment
   Rox_Point2D_Double_Struct points[2];
};

//! Segment 2D double structure
typedef struct Rox_Segment2D_Struct Rox_Segment2D_Struct;

//! Define
#define ROX_TYPE_SEGMENT2D_DOUBLE (sizeof(struct Rox_Segment2D_Struct) << 2)

//! Segment 2D signed int structure
struct Rox_Segment2D_Sint_Struct
{
   //! The 2 extremal points of the segment
   Rox_Point2D_Sint_Struct points[2];
};

//! Segment 2D signed int structure
typedef struct Rox_Segment2D_Sint_Struct Rox_Segment2D_Sint_Struct;

//! Define
#define ROX_TYPE_SEGMENT2D_SINT_DOUBLE (sizeof(struct Rox_Segment2D_Sint_Struct) << 2)

//! @}

#endif
