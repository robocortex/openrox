//==============================================================================
//
//    OPENROX   : File ehid_match_struct.h
//
//    Contents  : Structure of ehid_match module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_MATCH_STRUCT__
#define __OPENROX_EHID_MATCH_STRUCT__

#include <system/memory/datatypes.h>

//! \addtogroup EHID
//! @{

//! Match structure 
struct Rox_Ehid_Match_Struct
{
   //! Reference feature id
   Rox_Uint dbid;
   //! Current feature id
   Rox_Uint curid;
   //! Matching score id 
   Rox_Uint score;
   //! Rotation error 
   Rox_Double roterr;
};

typedef struct Rox_Ehid_Match_Struct Rox_Ehid_Match_Struct;

//! define 
#define ROX_TYPE_EHID_MATCH (sizeof(struct Rox_Ehid_Match_Struct) << 2)

//! @} 

#endif
