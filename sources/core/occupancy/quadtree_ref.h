//==============================================================================
//
//    OPENROX   : File quadtree_ref.h
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

#ifndef __OPENROX_QUADTREE_REF__
#define __OPENROX_QUADTREE_REF__

#include <generated/dllist_quadtree_item.h>
#include <baseproc/geometry/rectangle/rectangle.h>

//! \addtogroup Occupancy
//! @{

//! Quadtree node 
typedef struct Rox_QuadTree_Ref_Node_Struct * Rox_QuadTree_Ref_Node;

//! Quadtree structure 
typedef struct Rox_QuadTree_Ref_Struct * Rox_QuadTree_Ref;

//! Create a new quadtree object using a given global 2D bounding box. Space will be divided in four parts hierarchically.
//! \param  [out] obj            The created quadtree.
//! \param  [in]  bounding_box   The limits of the 2D space
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_ref_new ( 
   Rox_QuadTree_Ref * obj, 
   Rox_Rect_Sint bounding_box
);

//! Delete a quadtree object
//! \param  [out] obj            The quadtree pointer to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_ref_del(Rox_QuadTree_Ref * obj);

//! Add a point to the quadtree. Each level of the quadtree will contains this points if the associated rectangle contains this point.
//! \param  [in]  obj            The object quadtree.
//! \param  [in]  x              The x coordinate of the point
//! \param  [in]  y              The y coordinate of the point
//! \param  [in]  val            The value to associate to this point (a convenience value mainly used to index things).
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_ref_add(Rox_QuadTree_Ref obj, Rox_Double x, Rox_Double y, Rox_Uint val);

//! Remove every points from quadtree
//! \param  [in]  obj            The quadtree to clean up
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_ref_reset(Rox_QuadTree_Ref obj);

//! Get a list of points inside a given rectangle using the quadtree
//! \param  [out] list           The result list
//! \param  [in]  obj            The quadtree object
//! \param  [in]  lookup         The rectangle to lookup
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_ref_search (
   Rox_Dllist_QuadTree_Item list, 
   Rox_QuadTree_Ref obj, 
   Rox_Rect_Sint lookup
);

//! @} 

#endif
