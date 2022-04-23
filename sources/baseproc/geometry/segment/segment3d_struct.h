//==============================================================================
//
//    OPENROX   : File segment3d_struct.h
//
//    Contents  : Structure of segment3d module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SEGMENT3D_STRUCT__
#define __OPENROX_SEGMENT3D_STRUCT__

#include <system/memory/datatypes.h>
#include <baseproc/geometry/point/point3d_struct.h>

//! \ingroup Euclidean_Geometry
//! \defgroup Segment Segment

//! \addtogroup Segment
//! @{

//! Segment 3D double structure
struct Rox_Segment3D_Struct
{
   //! Extremal points of the segment
   Rox_Point3D_Double_Struct points[2];
};

//! Segment 3D double structure
typedef struct Rox_Segment3D_Struct Rox_Segment3D_Struct;

//! define
#define ROX_TYPE_SEGMENT3D_DOUBLE (sizeof(struct Rox_Segment3D_Struct) << 2)

//! Segment 3D float structure
struct Rox_Segment3D_Float_Struct
{
   //! Extremal points of the segment
   Rox_Point3D_Float_Struct points[2];
};

//! Segment 3D float structure
typedef struct Rox_Segment3D_Float_Struct Rox_Segment3D_Float_Struct;

//! define
#define ROX_TYPE_SEGMENT3D_DOUBLE (sizeof(struct Rox_Segment3D_Struct) << 2)

//! @}

#endif
