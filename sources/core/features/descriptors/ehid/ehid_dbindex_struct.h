//==============================================================================
//
//    OPENROX   : File ehid_dbindex_structure.h
//
//    Contents  : API of ehid_dbindex module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_DBINDEX_STRUCT__
#define __OPENROX_EHID_DBINDEX_STRUCT__

#include <system/memory/datatypes.h>

//!	\addtogroup EHID
//!	@{

//! Indices for a point 
struct Rox_Ehid_DbIndex_Struct
{
   //! A flag per index
   Rox_Uint flag_indices[32];
};

typedef struct Rox_Ehid_DbIndex_Struct Rox_Ehid_DbIndex_Struct;

//! define 
#define ROX_TYPE_EHID_DBINDEX (sizeof(struct Rox_Ehid_DbIndex_Struct) << 2)

//! @} 

#endif
