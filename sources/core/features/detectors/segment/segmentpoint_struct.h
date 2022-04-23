//==============================================================================
//
//    OPENROX   : File segmentpoint_struct.h
//
//    Contents  : Structure of segmentpoint module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SEGMENT_POINT_STRUCT__
#define __OPENROX_SEGMENT_POINT_STRUCT__

#include <system/memory/datatypes.h>

//! \ingroup Detectors
//! \defgroup Segment_Point Segment_Point

//! \addtogroup Segment_Point
//! @{

//! Description of a segment (Fast like) feature
struct Rox_Segment_Point_Struct
{
   //! Row
   Rox_Uint i;

   //! Column
   Rox_Uint j;

   //! Simple score
   Rox_Uint score;

   //! Response to advanced score like harris
   Rox_Float response;

   //! Orientation of feature if needed
   Rox_Float ori;

   //! Level in the scale space
   Rox_Uint level;
};

// Alias to use directly Rox_Segment_Point_Struct instead of "struct Rox_Segment_Point_Struct"
typedef struct Rox_Segment_Point_Struct Rox_Segment_Point_Struct;

//! @}

#endif
