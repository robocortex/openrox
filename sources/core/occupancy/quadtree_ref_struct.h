//==============================================================================
//
//    OPENROX   : File quadtree_ref_struct.h
//
//    Contents  : API of quadtree_ref module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUADTREE_REF_STRUCT__
#define __OPENROX_QUADTREE_REF_STRUCT__

#include <generated/dllist_quadtree_item_struct.h>
#include <baseproc/geometry/rectangle/rectangle_struct.h>

//!   \addtogroup Occupancy
//!   @{

//! Quadtree node 
struct Rox_QuadTree_Ref_Node_Struct
{
   //! CHildren 
   struct Rox_QuadTree_Ref_Node_Struct * children[4];

   //! 2D region covered 
   Rox_Rect_Sint_Struct region;

   //! List of items 
   Rox_Dllist_QuadTree_Item refs;
};

//! Quadtree structure 
struct Rox_QuadTree_Ref_Struct
{
   //! Quadtree height 
   Rox_Sint height;

   //! Stack for search and insertion
   Rox_QuadTree_Ref_Node * stack;

   //! Quadtree root 
   Rox_QuadTree_Ref_Node root;
};

//! @} 

#endif
