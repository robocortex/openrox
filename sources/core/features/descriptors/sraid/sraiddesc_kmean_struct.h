//==============================================================================
//
//    OPENROX   : File sraiddesc_kmean_struct.h
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

#ifndef __OPENROX_SRAID_DESC_KMEAN_STRUCT__
#define __OPENROX_SRAID_DESC_KMEAN_STRUCT__

#include "sraiddesc_struct.h"
#include <generated/dynvec_sint.h>

//! \addtogroup SRAID
//! @{

//! kmean tree node structure
struct Rox_Sraid_Tree_Node_Struct
{
   //! Mean for cluster, note that we force alignment of first element
   //! \return define
 
   Rox_Ushort ROX_STATIC_ALIGN(16) mean[ROX_SRAID_DESCRIPTOR_SIZE];

   //! Accumulator
   Rox_Uint acc[ROX_SRAID_DESCRIPTOR_SIZE];

   //! Base pointer for deletion if aligned
   struct Rox_Sraid_Tree_Node_Struct * base_pointer;

   //! Children nodes
   struct Rox_Sraid_Tree_Node_Struct ** children;

   //! Indices in this level if terminal leaf
   Rox_Uint * indices_pool;

   //! Radius for cluster
   Rox_Sint radius;

   //! Current level
   Rox_Uint level;

   //! Current size
   Rox_Uint size;

   //! Current weight
   Rox_Uint idnode;
};

//! kmean tree node structure
struct Rox_Sraid_Tree_Struct
{
   //!How many clusters per level
   Rox_Uint branch_factor;

   //!Maximum level of the pyramid (-1 equal no limits)
   Rox_Sint max_level;

   //!Maximum iteration for k-mean algorithm convergence
   Rox_Uint max_iters;

   //!Factor of importance for borders
   Rox_Float border_factor;

   //!database (shallow copy, if deleted outside it is not valid anymore)
   Rox_DynVec_SRAID_Feature database;

   //!First level of hierarchical k-means
   Rox_Sraid_Tree_Node root;

   //!Indices of database
   Rox_Uint * indices;

   //!Buffer for centers indices
   Rox_Uint * centers;

   //!Random number set pool
   Rox_Uint * random_pool;

   //!Node counter
   Rox_Uint node_count;
};

//! @} 

#endif
