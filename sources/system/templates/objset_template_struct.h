//==============================================================================
//
//    OPENROX   : File objset_template.h
//
//    Contents  : API of objset_template module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_OBJSET_@OBJSETTYPE@_STRUCT__
#define __OPENROX_OBJSET_@OBJSETTYPE@_STRUCT__

#include <system/memory/datatypes.h>
#include <system/memory/memory.h>
#include <system/errors/errors.h>

#include @OBJSETTYPE_INCLUDE@
#include @OBJSETTYPE_INCLUDE_STRUCT@

//! \ingroup ObjSet
//! @defgroup ObjSet_@OBJSETTYPE@ objset_@OBJSETTYPE@
//! @{

//! Dynamic vector base structure
struct Rox_ObjSet_@OBJSETTYPE@_Struct
{
   //! How many more blocks are allocated each time the allocated blocks are full
   Rox_Uint allocblocks;

   //! How many allocated cells are allocated
   Rox_Uint allocated;

   //! How many allocated cells are used
   Rox_Uint used;

   //! Real continuous memory storage
   Rox_@OBJSETTYPE@ * data;
};

//! @}

#endif
