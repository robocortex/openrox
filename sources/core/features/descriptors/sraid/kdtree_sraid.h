//==============================================================================
//
//    OPENROX   : File kdtree_sraid.h
//
//    Contents  : API of kdtree_sraid module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_KDTREE_SRAID__
#define __OPENROX_KDTREE_SRAID__

#include <generated/dynvec_sraiddesc.h>
#include <generated/dynvec_uint.h>
#include <core/features/descriptors/sraid/sraid_matchresultset.h>
#include <core/features/descriptors/sraid/heap_branch.h>

//! \addtogroup SRAID
//! @{

//! define 
#define MEAN_SAMPLES 100
//! define 
#define TOP_RAND 5

//! Pointer to a branch of the tree +  score, for recursive scoring
struct Rox_Kdtree_Sraid_Node_Struct
{
   //! Index of the node 
   int _index;
   //! Dimension cut value 
   int _cut_val;
   //! Dimension index of cut  
   int _cut_index;

   //! Dimension cut left part  
   struct Rox_Kdtree_Sraid_Node_Struct * _child_left;
   //! Dimension cut right part  
   struct Rox_Kdtree_Sraid_Node_Struct * _child_right;
};

//! KDTREE for sraid
struct Rox_Kdtree_Sraid_Struct
{
   //! count of trees inside kdtre structure 
	Rox_Uint _count_trees;
	//! count of trees leaves inside kdtre structure 
	Rox_Uint _count_leaves;
	//! Buffer 
	Rox_DynVec_Uint _checked;
	//! Buffer 
	Rox_Heap_Branch _heap;

   //! Trees 
   Rox_Kdtree_Sraid_Node * _roots;
};

//! KDTREE for sraid
typedef struct Rox_Kdtree_Sraid_Struct * Rox_Kdtree_Sraid;

//! Create a new kdtree sraid
//! \param [out] obj pointer to the object created
//! \param [in] count_trees number of subtrees created
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kdtree_sraid_new(Rox_Kdtree_Sraid * obj, Rox_Uint count_trees);

//! Delete a kdtree sraid
//! \param [out] obj pointer to the object  to delete
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kdtree_sraid_del(Rox_Kdtree_Sraid * obj);

//! Clean a kdtree sraid
//! \param [in] obj the object to clean
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kdtree_sraid_clean(Rox_Kdtree_Sraid obj);

//! Build a kdtree sraid index given a set of features
//! \param [in] obj the object  to build into
//! \param [in] features the features to index
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kdtree_sraid_build(Rox_Kdtree_Sraid obj, Rox_DynVec_SRAID_Feature features);

//! Search for features neighboors
//! \param [in] results the result set of closest features
//! \param [in] obj the object  to search into
//! \param [in] features the features to which are indexed in this tree
//! \param [in] feat the feature to search for
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kdtree_sraid_search(Rox_SRAID_MatchResultSet results, Rox_Kdtree_Sraid obj, Rox_DynVec_SRAID_Feature features, Rox_SRAID_Feature_Struct * feat);

//! Save the index to a file
//! \param [in] obj the object  to save
//! \param [in] filename the file name to save to
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kdtree_sraid_save(Rox_Kdtree_Sraid obj, char *filename);

//! Load the index from a file
//! \param [in] obj the object  to load
//! \param [in] filename the file name to load from
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_kdtree_sraid_load(Rox_Kdtree_Sraid obj, char *filename);

//! @} 

#endif
