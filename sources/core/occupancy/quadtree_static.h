//==============================================================================
//
//    OPENROX   : File quadtree_static.h
//
//    Contents  : API of quadtree_static module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#ifndef __OPENROX_QUADTREE__
#define __OPENROX_QUADTREE__

#include <generated/dynvec_uint.h>
#include <generated/dynvec_uint_struct.h>
#include <generated/dynvec_point2d_double.h>
#include <baseproc/geometry/rectangle/rectangle.h>

//! \ingroup Geometry
//! \defgroup QuadTree QuadTree
//! \brief Partition of two dimensional space y recursively subdividing it into four quadrants or region.

//! \ingroup QuadTree
//!  \addtogroup Occupancy
//!   @{

//! Quadtree node 
struct Rox_QuadTree_Node_Struct
{
  //! X coordinates of this node in a given space 
  int _x;

  //! Y coordinates of this node in a given space 
  int _y;

  //! Height of the area managed by this node 
  int _height;

  //! Width of the area managed by this node 
  int _width;

  //! Count points managed by this node 
  Rox_Uint _count_points;

  //! List of indices to the point managed by this node 
  Rox_Uint * _indices;

  //! The 4 children of this node in clock-wise order (starting from top left)
  //! Either the 4 children are NULL, either all not NULL. 
  struct Rox_QuadTree_Node_Struct * children[4];
};

//! object 
typedef struct Rox_QuadTree_Node_Struct * Rox_QuadTree_Node;

//! Quadtree 
struct Rox_QuadTree_Struct
{
   //! Maximum number of points before a node is divided (do not apply if the minimum size reached
   Rox_Uint _max_points_per_node;

   //! Minimum size of the smallest dimension of a leaf.
   //! A node cannot be created with a size under this minima.
   Rox_Uint _min_node_size;

   //! Index containers, used by tree node
   Rox_DynVec_Uint _indices;

   //! Root node for the quadtree
   Rox_QuadTree_Node _root;

   //! Temporary stack for depth first search
   Rox_QuadTree_Node * _stack;
};

//! Pointer to the structure  
typedef struct Rox_QuadTree_Struct * Rox_QuadTree;

//! Create an empty Quadtree node
//! \param  [out] obj                  A pointer to the newly created object
//! \param  [in]  node_x               The smallest coordinate allowed in X
//! \param  [in]  node_y               The smallest coordinate allowed in Y
//! \param  [in]  node_width           The width of the 2D space managed by this node
//! \param  [in]  node_height          The height of the 2D space managed by this node
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_node_new (
  Rox_QuadTree_Node * obj, 
  Rox_Sint node_x, 
  Rox_Sint node_y, 
  Rox_Sint node_width, 
  Rox_Sint node_height
);

//! Delete a Quadtree node
//! \param [out] obj a pointer to the object to delete
//! \return An error code
//! \todo to be tested
ROX_API Rox_ErrorCode rox_quadtree_node_del (
  Rox_QuadTree_Node *obj);

//! Distribute points into this node and propage children if needed
//! \param  [in]  obj                  The object to fill
//! \param  [in]  tree                 The quadtree object
//! \param  [in]  points               The source point list
//! \param  [in]  indices              The indices list
//! \param  [in]  count                The indices size
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_node_distribute (
  Rox_QuadTree_Node obj, 
  Rox_QuadTree tree, 
  Rox_DynVec_Point2D_Double points, 
  Rox_Uint * indices, 
  Rox_Uint count
);

//! Create a new Quadtree object
//! \param  [out] obj                  A pointer to the newly created object
//! \param  [in]  tree_x               The smallest coordinate allowed in X
//! \param  [in]  tree_y               The smallest coordinate allowed in Y
//! \param  [in]  tree_width           The width of the 2D space managed by this tree
//! \param  [in]  tree_height          The height of the 2D space managed by this tree
//! \param  [in]  max_points_per_node  The maximum number of points per node (if not a terminal node)
//! \param  [in]  min_node_size        The minimum size of one node allowed for the smallest dimension
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_new (
  Rox_QuadTree *obj, Rox_Sint tree_x, Rox_Sint tree_y, Rox_Sint tree_width, 
  Rox_Sint tree_height, Rox_Uint max_points_per_node, Rox_Uint min_node_size);

//! Delete a Quadtree object
//! \param  [in]  obj                  A pointer to the object to delete
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_del (
  Rox_QuadTree *obj);

//! Make the quadtree empty
//! \param  [in]  obj                  The object to reset
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_reset (
  Rox_QuadTree obj);

//! Fill the quadtree with a new list of points (reset the existing quadtree)
//! \param  [in]  obj                  The quadtree object
//! \param  [in]  points               The points list to distribute
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_apply (
  Rox_QuadTree obj, 
  Rox_DynVec_Point2D_Double points);

//! Retrieve all the points inside a given rectangle
//! \param  [out] resindices           The list of found points indices
//! \param  [in]  obj                  The quadtree object to search into
//! \param  [in]  points               The list of points used to build the quadtree
//! \param  [in]  rectangle            To search bounding box
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_search_rectangle (
  Rox_DynVec_Uint resindices, Rox_QuadTree obj, 
  Rox_DynVec_Point2D_Double points, 
  Rox_Rect_Sint rectangle);

//! Retrieve all the points which lies on a given segment with a given tolerancy of orthogonal distance to the line
//! \param  [out] resindices           The list of found points indices
//! \param  [in]  obj                  The quadtree object to search into
//! \param  [in]  points               The list of points used to build the quadtree
//! \param  [in]  pt1                  The first endpoint of the segment
//! \param  [in]  pt2                  The second endpoint of the segment
//! \param  [in]  tol                  The tolerated orthogonal distance from a point to this segment
//! \return An error code
//! \todo   To be tested
ROX_API Rox_ErrorCode rox_quadtree_search_segment (
  Rox_DynVec_Uint resindices, Rox_QuadTree obj, 
  Rox_DynVec_Point2D_Double points, 
  Rox_Point2D_Double pt1, 
  Rox_Point2D_Double pt2, 
  Rox_Double tol
);

//! @} 

#endif
