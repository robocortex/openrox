//==============================================================================
//
//    OPENROX   : File segment3d.h
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

#ifndef __OPENROX_SEGMENT3D__
#define __OPENROX_SEGMENT3D__

#include <system/memory/datatypes.h>

//! \ingroup Euclidean_Geometry
//! \defgroup Segment Segment

//! \addtogroup Segment
//! @{

//! Segment 3D double pointer to structure
typedef struct Rox_Segment3D_Struct * Rox_Segment3D;

//! Segment 3D float pointer to structure
typedef struct Rox_Segment3D_Float_Struct * Rox_Segment3D_Float;

ROX_API Rox_ErrorCode rox_segment3d_new ( Rox_Segment3D * segment3d );

ROX_API Rox_ErrorCode rox_segment3d_del ( Rox_Segment3D * segment3d );

ROX_API Rox_ErrorCode rox_segment3d_set ( 
   Rox_Segment3D segment3d, 
   Rox_Double X1, 
   Rox_Double Y1, 
   Rox_Double Z1, 
   Rox_Double X2, 
   Rox_Double Y2, 
   Rox_Double Z2
);

//! @}

#endif
