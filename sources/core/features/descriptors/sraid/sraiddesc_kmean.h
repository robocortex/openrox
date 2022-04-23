//==============================================================================
//
//    OPENROX   : File sraiddesc_kmean.h
//
//    Contents  : API of sraiddesc_kmean module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_SRAID_DESC_KMEAN__
#define __OPENROX_SRAID_DESC_KMEAN__

#include "sraiddesc.h"
#include <generated/dynvec_sint.h>

//! \addtogroup SRAID
//! @{

//! kmean tree node structure
typedef struct Rox_Sraid_Tree_Node_Struct * Rox_Sraid_Tree_Node;

//! kmean tree node structure
typedef struct Rox_Sraid_Tree_Struct * Rox_Sraid_Tree;

//! Create a new k-means (a HKM tree node)
//! \param [out] obj a pointer to the created node
//! \param [in] nodeid the node id
//! \return An error code
ROX_API Rox_ErrorCode rox_sraid_tree_node_new(Rox_Sraid_Tree_Node * obj, Rox_Uint nodeid);

//! Delete a k-means (a HKM tree node)
//! \param [out] obj a pointer to the node
//! \param [in] tree a pointer to the container tree
//! \return An error code
ROX_API Rox_ErrorCode rox_sraid_tree_node_del(Rox_Sraid_Tree_Node * obj, Rox_Sraid_Tree tree);

//! Create a brand new empty hierarchical k-means tree (do not compute clusters)
//! \param [out] obj the pointer to the created tree
//! \param [in] features the features set to use to train the tree.
//! \param [in] branching_factor factor center count per levels
//! \param [in] max_level the maximum number of levels computed (or -1 for unlimited level count)
//! \return An error code
ROX_API Rox_ErrorCode rox_sraid_tree_new(Rox_Sraid_Tree * obj, Rox_DynVec_SRAID_Feature features, Rox_Uint branching_factor, Rox_Sint max_level);

//! Delete a HKM tree
//! \param [out] obj a pointer to the node
//! \return An error code
ROX_API Rox_ErrorCode rox_sraid_tree_del(Rox_Sraid_Tree * obj);

//! Build the tree by learning hierarchical clusters given a set of features
//! \param [in] obj a pointer to the container tree
//! \return An error code
ROX_API Rox_ErrorCode rox_sraid_tree_build(Rox_Sraid_Tree obj);

//! Match features using the learned HKM
//! \param [in] matchassocs a vector which contains the id of the reference feature matched for each current features.
//! \param [in] obj a pointer to the container tree
//! \param [in] features the features to match
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_sraid_tree_matchset(Rox_DynVec_Sint matchassocs, Rox_Sraid_Tree obj, Rox_DynVec_SRAID_Feature features);

//! @} 

#endif
