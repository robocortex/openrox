//==============================================================================
//
//    OPENROX   : File ehid_compiler_struct.h
//
//    Contents  : API of ehid_compiler module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_COMPILER_STRUCT__
#define __OPENROX_EHID_COMPILER_STRUCT__

#include <generated/dynvec_double_struct.h>
#include <generated/dynvec_ehid_point_struct.h>

#include <core/features/descriptors/ehid/ehid.h>
#include <core/features/descriptors/ehid/ehid_dbindex.h>
#include <core/features/descriptors/ehid/ehid_database.h>
#include <core/features/descriptors/ehid/ehid_searchtree.h>

//! \addtogroup EHID
//! @{

//! Compiler structure
struct Rox_Ehid_Compiler_Struct
{
   //! Number of targets to compile
   Rox_Uint count_targets;

   //! Contains pixel width for each target
   Rox_DynVec_Double _pwidths;

   //! Contains pixel heights for each target
   Rox_DynVec_Double _pheights;

   //! Contains meter width for each target
   Rox_DynVec_Double _mwidths;

   //! Contains meter heights for each target
   Rox_DynVec_Double _mheights;

   //! Contains Full list of features
   Rox_DynVec_Ehid_Point flatlist;

   //! Contains full list of features per indices
   Rox_DynVec_Ehid_Point indexed_lists[INDEX_MAX_VAL];

   //! Contains All the trees compiled
   Rox_Ehid_SearchTree indexed_dbs[INDEX_MAX_VAL];
};

//! @} 

#endif
