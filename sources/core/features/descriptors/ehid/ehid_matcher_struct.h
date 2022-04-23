//==============================================================================
//
//    OPENROX   : File ehid_matcher_struct.h
//
//    Contents  : Structure of ehid_matcher module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_MATCHER_STRUCT__
#define __OPENROX_EHID_MATCHER_STRUCT__

#include <core/features/descriptors/ehid/ehid.h>
#include <core/features/descriptors/ehid/ehid_database.h>
#include <generated/objset_ehid_target.h>
#include <generated/dynvec_uint.h>
#include <generated/dynvec_ehid_dbindex.h>

#include <generated/dynvec_ehid_match_struct.h>

//! \addtogroup EHID
//! @{

//! Matcher object 
struct Rox_Ehid_Matcher_Struct
{
   //! A vector used as a matching result temporary container 
   Rox_DynVec_Ehid_Match results;

   //! How many templates we need to find per processing 
   Rox_Uint max_templates_per_query;
};

//! @} 

#endif
