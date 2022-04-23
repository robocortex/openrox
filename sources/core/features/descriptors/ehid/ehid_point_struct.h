//==============================================================================
//
//    OPENROX   : File ehid_struct.h
//
//    Contents  : Structure of ehid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_STRUCT__
#define __OPENROX_EHID_STRUCT__

#include "ehid_description.h"

#include <system/memory/datatypes.h>
#include <baseproc/geometry/point/point2d_struct.h>

//! \addtogroup EHID
//! @{

//! define
#define INDEX_MAX_VAL 32

//! Ehid point structure
struct Rox_Ehid_Point_Struct
{
   //! Feature unique id (used for reference points)
   Rox_Uint uid;

   //! Feature position
   Rox_Point2D_Double_Struct pos;

   //! Feature position in meter space
   Rox_Point2D_Double_Struct pos_meters;

   //! Feature direction (another orientation representation)
   Rox_Point2D_Double_Struct dir;

   //! Feature scale detection
   Rox_Double scale;

   //! Database owning this feature
   Rox_Uint dbid;

   //! Reference counter
   Rox_Uint refcount;

   //! Description
   Rox_Ehid_Description Description;

   //! Short description for index
   Rox_Uint index;
};

typedef struct Rox_Ehid_Point_Struct Rox_Ehid_Point_Struct;

//! define
#define ROX_TYPE_EHID_POINT (sizeof(struct Rox_Ehid_Point_Struct) << 2)

//! @} 

#endif
