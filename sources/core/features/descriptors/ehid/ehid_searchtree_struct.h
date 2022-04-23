//==============================================================================
//
//    OPENROX   : File ehid_searchtree_struct.h
//
//    Contents  : API of ehid_searchtree module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_SEARCHTREE_STRUCT__
#define __OPENROX_EHID_SEARCHTREE_STRUCT__

#include <generated/dynvec_ehid_dbnode_struct.h>
#include <generated/dynvec_ehid_point.h>
#include <generated/dynvec_ehid_match.h>
#include <stdio.h>

//!  \addtogroup EHID
//!  @{

//! EHID Database structure 
struct Rox_Ehid_SearchTree_Struct
{
   //! Is the database already compiled ? 
   Rox_Uint is_compiled;

   //! Maximum height of the tree 
   Rox_Uint max_height;

   //! Roots of the binary trees computed during compilation 
   Rox_DynVec_Ehid_DbNode roots;

   //! Stack for searching into the trees 
   Rox_Ehid_DbNode_Struct ** stack;
};

//! @} 

#endif
