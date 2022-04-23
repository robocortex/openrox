//==============================================================================
//
//    OPENROX   : File ehid_searchtree.h
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

#ifndef __OPENROX_EHID_SEARCHTREE__
#define __OPENROX_EHID_SEARCHTREE__

#include "ehid_description.h"

#include <generated/dynvec_ehid_dbnode.h>
#include <generated/dynvec_ehid_point.h>
#include <generated/dynvec_ehid_match.h>
#include <stdio.h>

//! \addtogroup EHID
//! @{

//! EHID SearchTree object is a pointer to the opaque structure 
typedef struct Rox_Ehid_SearchTree_Struct * Rox_Ehid_SearchTree;

//! Create a new empty database
//! \param [out] obj a pointer to the newly created object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_new(Rox_Ehid_SearchTree * obj);

//! Delete a database
//! \param [out] obj a pointer to the object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_del(Rox_Ehid_SearchTree * obj);

//! Reset a database
//! \param obj a pointer to the object
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_reset(Rox_Ehid_SearchTree obj);

//! Compile the database.
//! \param obj the database object
//! \param db the feature list to inject
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_compile(Rox_Ehid_SearchTree obj, Rox_DynVec_Ehid_Point db);

//! Lookup all features in the database within a given distance from a base description
//! \param result the list of found features
//! \param obj the database
//! \param base the base description
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_lookup(Rox_DynVec_Ehid_Match result, Rox_Ehid_SearchTree obj, Rox_Ehid_Description base);

//! Save compiled tree to a file
//! \param obj the database object to use
//! \param file the file stream to save to
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_save_tree(Rox_Ehid_SearchTree obj, FILE * file);

//! Load compiled tree from a file
//! \param obj the database object to use
//! \param file the file stream to load from
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_load_tree(Rox_Ehid_SearchTree obj, FILE * file);

//!
//! \param [out] ser 
//! \param [in] tree 
//! \return An error code
//! \todo to be tested

ROX_API Rox_ErrorCode rox_ehid_searchtree_serialize(char* ser, const Rox_Ehid_SearchTree tree);

//! \param [out] tree
//! \param [in] ser 
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_deserialize(Rox_Ehid_SearchTree tree, const char* ser);

//! \param [out] size 
//! \param [in] tree
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_get_octet_size(Rox_Uint *size, const Rox_Ehid_SearchTree tree);

//! \param [out] ser 
//! \param [in] node 
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_serialize_tree_node(char* ser, const Rox_Ehid_DbNode_Struct * node);

//! \param [out] node 
//! \param [in] ser 
//! \param [in] first 
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_deserialize_tree_node(Rox_Ehid_DbNode_Struct ** node, const char* ser, const Rox_Uint first);

//! \param [out] size 
//! \param [in] node
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_ehid_searchtree_get_octet_size_tree_node(Rox_Uint *size, const Rox_Ehid_DbNode_Struct * node);

//! @} 

#endif
