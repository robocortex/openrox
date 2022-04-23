//=============================================================================
//
//    OPENROX   : File ehid_dbnode_struct.h
//
//    Contents  : Structure of ehid_dbnode module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//============================================================================

#ifndef __OPENROX_EHID_DBNODE_STRUCT__
#define __OPENROX_EHID_DBNODE_STRUCT__

#include <system/memory/datatypes.h>
#include <core/features/descriptors/ehid/ehid_point_struct.h>

//! \addtogroup EHID
//! @{

//!  EHID database node for the binary trees 
struct Rox_Ehid_DbNode_Struct
{
   //! Depth of this node 
   Rox_Uint level;

   //! Database feature index
   Rox_Sint dbid;

   //! Description of this node
   Rox_Ehid_Description desc;

   //! Closest distance to another root node
   Rox_Uint closestdist;

   //! Closest root node (used during compilation)
   struct Rox_Ehid_DbNode_Struct * closest;

   //! Children for this node
   struct Rox_Ehid_DbNode_Struct * left;

   //! Children for this node
   struct Rox_Ehid_DbNode_Struct * right;
};

//! EHID database node for the binary trees 
typedef struct Rox_Ehid_DbNode_Struct Rox_Ehid_DbNode_Struct;

//! define 
#define ROX_TYPE_EHID_DBNODE (sizeof(struct Rox_Ehid_DbNode_Struct) << 2)

//! The Rox_DB_Ident_SL3 object 
typedef struct Rox_Ehid_DbNode_Struct * Rox_Ehid_DbNode;

//! @} 

#endif
