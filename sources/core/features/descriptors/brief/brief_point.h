//==============================================================================
//
//    OPENROX   : File brief_point_object.h
//
//    Contents  : API of brief_point_obj module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_BRIEF_POINT_OBJECT__
#define __OPENROX_BRIEF_POINT_OBJECT__

#include <generated/array2d_uchar.h>
#include <system/memory/datatypes.h>

//! \addtogroup BRIEF
//! @{

//! Indices for a point
typedef struct Rox_Brief_Point_Struct Rox_Brief_Point_Struct;

//! define 
#define ROX_TYPE_BRIEF_POINT (sizeof(struct Rox_Brief_Point_Struct) << 2)

//! @} 

#endif
