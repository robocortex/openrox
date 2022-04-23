//==============================================================================
//
//    OPENROX   : File segment3d_save.h
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

#ifndef __OPENROX_SEGMENT3D_PRINT__
#define __OPENROX_SEGMENT3D_PRINT__

#include <system/memory/datatypes.h>
#include <baseproc/geometry/segment/segment3d.h>

//! \ingroup Euclidean_Geometry
//! \defgroup Segment Segment

//! \addtogroup Segment
//! @{


ROX_API Rox_ErrorCode rox_segment3d_save ( Rox_Segment3D segment3d );

ROX_API Rox_ErrorCode rox_segment3d_float_save ( Rox_Segment3D_Float segment3d );

ROX_API Rox_ErrorCode rox_vector_segment3d_save ( const Rox_Char * filename, Rox_Segment3D segment3d, Rox_Sint nbs );

//! @}

#endif
