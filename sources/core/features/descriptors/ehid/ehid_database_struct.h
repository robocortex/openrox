//==============================================================================
//
//    OPENROX   : File ehid_database_struct.h
//
//    Contents  : Structure of ehid_database module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_EHID_DATABASE_STRUCT__
#define __OPENROX_EHID_DATABASE_STRUCT__

#include <generated/dynvec_ehid_dbnode.h>
#include <generated/dynvec_ehid_match.h>
#include <generated/dynvec_ehid_point_struct.h>
#include <generated/objset_ehid_target_struct.h>
#include <core/features/descriptors/ehid/ehid_dbindex.h>
#include <core/features/descriptors/ehid/ehid_searchtree.h>
#include <generated/array2d_uchar.h>
#include <stdio.h>
#include <core/features/descriptors/ehid/ehid_target_struct.h>

//! \addtogroup EHID
//! @{

//! EHID Database structure 
struct Rox_Ehid_Database_Struct
{
   //! Trees for all features
   Rox_Ehid_SearchTree _trees[INDEX_MAX_VAL];

   //! Full list of features non sorted 
   Rox_DynVec_Ehid_Point _fulllist;

   //! List of targets in this database 
   Rox_ObjSet_Ehid_Target _targets;
};

//! @} 

#endif
