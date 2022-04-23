//==============================================================================
//
//    OPENROX   : File heap_branch.h
//
//    Contents  : API of heap_branch module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_HEAP_BRANCH__
#define __OPENROX_HEAP_BRANCH__

#include <generated/dynvec_uint.h>

//! \addtogroup SRAID
//! @{

//! Structure 
typedef struct Rox_Kdtree_Sraid_Node_Struct * Rox_Kdtree_Sraid_Node;

//! Pointer to a branch of the tree +  score, for recursive scoring
struct Rox_Branch_Struct
{
   //! Scored branch 
	Rox_Kdtree_Sraid_Node node;

	//! score 
	Rox_Uint score;
};

//! Pointer to a branch of the tree +  score, for recursive scoring
typedef struct Rox_Branch_Struct Rox_Branch_Struct;

//! Heap sorted structure for branch scoring 
struct Rox_Heap_Branch_Struct
{
   //! Root branch 
	Rox_Branch_Struct * _data;

	//! Current branch count 
	Rox_Uint _count_elems;

	//! Max accepted number of branches 
	Rox_Uint _max_elems;
};

//! Heap sorted structure for branch scoring 
typedef struct Rox_Heap_Branch_Struct * Rox_Heap_Branch;

//! Create a new heap for branch scoring
//! \param [out] obj heap object pointer
//! \param [in] size maximum size of the heap
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_heap_branch_new(Rox_Heap_Branch * obj, Rox_Uint size);

//! Delete a heap for branch scoring
//! \param [out] obj heap object pointer
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_heap_branch_del(Rox_Heap_Branch * obj);

//! Reset a heap for branch scoring
//! \param [in] obj heap object
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_heap_branch_reset(Rox_Heap_Branch obj);

//! Add a new branch to the sorted heap
//! \param [in] obj heap object
//! \param [in] node ew node to  push
//! \param [in] score score of this heap
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_heap_branch_push(Rox_Heap_Branch obj, Rox_Kdtree_Sraid_Node node, Rox_Uint score);

//! Retrieve the best branch
//! \param [in] obj heap object
//! \param [in] node node pulled
//! \param [in] score of the pulled heap
//! \return en error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_heap_branch_pop(Rox_Heap_Branch obj, Rox_Kdtree_Sraid_Node * node, Rox_Uint * score);

//! @} 

#endif
