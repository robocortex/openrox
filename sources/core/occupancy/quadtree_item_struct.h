//==============================================================================
//
//    OPENROX   : File quadtree_item_struct.h
//
//    Contents  : Stuct of quadtree_item module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUADTREE_ITEM_STRUCT__
#define __OPENROX_QUADTREE_ITEM_STRUCT__

#include <system/memory/datatypes.h>

//! \addtogroup Occupancy
//! @{

//! Quadtree item structure
struct Rox_QuadTree_Item_Struct
{
   //! Quadtree item x position
   Rox_Double x;

   //! Quadtree item y position
   Rox_Double y;

   //! Quadtree item id 
   Rox_Uint ref;
};

//! Quadtree Item object
typedef struct Rox_QuadTree_Item_Struct Rox_QuadTree_Item_Struct;

//! @} 

#endif
