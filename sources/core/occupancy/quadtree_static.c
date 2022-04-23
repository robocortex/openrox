//==============================================================================
//
//    OPENROX   : File quadtree_static.c
//
//    Contents  : Implementation of quadtree_static module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "quadtree_static.h"
#include <baseproc/maths/maths_macros.h>
#include <generated/dynvec_point2d_double_struct.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode rox_quadtree_node_new(
   Rox_QuadTree_Node *obj, Rox_Sint node_x, Rox_Sint node_y, Rox_Sint node_width, Rox_Sint node_height)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree_Node ret = NULL;


   if (node_width < 1 || node_height < 1) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!obj) 
   {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_QuadTree_Node) rox_memory_allocate(sizeof(struct Rox_QuadTree_Node_Struct), 1);
   if (!ret) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   ret->_x = node_x;
   ret->_y = node_y;
   ret->_width = node_width;
   ret->_height = node_height;

   ret->_count_points = 0;
   ret->_indices = NULL;
   ret->children[0] = NULL;
   ret->children[1] = NULL;
   ret->children[2] = NULL;
   ret->children[3] = NULL;

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_node_del(Rox_QuadTree_Node *obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree_Node todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Recursive deletion of children
   rox_quadtree_node_del(&todel->children[0]);
   rox_quadtree_node_del(&todel->children[1]);
   rox_quadtree_node_del(&todel->children[2]);
   rox_quadtree_node_del(&todel->children[3]);

   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_node_distribute(Rox_QuadTree_Node obj, Rox_QuadTree tree, Rox_DynVec_Point2D_Double points, Rox_Uint * indices, Rox_Uint count)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint halfwidth, halfheight;
   Rox_Sint limit_x, limit_y;
   Rox_Sint x_separ, y_separ, id, swap;
   Rox_Uint count_lx, count_hx;
   Rox_Uint count_lxly, count_hxly;
   Rox_Uint count_lxhy, count_hxhy;

   Rox_Uint *half_lx = NULL, *half_hx = NULL;
   Rox_Uint *quarter_lxly = NULL;
   Rox_Uint *quarter_hxly = NULL;
   Rox_Uint *quarter_hxhy = NULL;
   Rox_Uint *quarter_lxhy = NULL;



   if (!obj || !points || !indices) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (count < 1) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (points->used < 1) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   obj->_indices = indices;
   obj->_count_points = count;

   // Do we need to descend the tree
   if (count <= tree->_max_points_per_node)
   {
      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   halfwidth = obj->_width / 2;
   halfheight = obj->_height / 2;

   // CAN we descend the tree ?
   if (halfwidth < (Rox_Sint) (tree->_min_node_size))
   {
      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   if (halfheight < (Rox_Sint) (tree->_min_node_size))
   {
      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   // Compute half planes parameters
   limit_x = obj->_x + halfwidth;
   limit_y = obj->_y + halfheight;

   // Compute separation in X of data
   x_separ = 0;
   for (id = 0; id < (Rox_Sint) count; id++)
   {
      Rox_Uint curid = indices[id];
      Rox_Point2D_Double_Struct curpt = points->data[curid];

      if (curpt.u < limit_x)
      {
         swap = indices[x_separ];
         indices[x_separ] = indices[id];
         indices[id] = swap;
         x_separ++;
      }
   }

   // Divide indices
   half_lx = indices;
   count_lx = x_separ;
   half_hx = &indices[x_separ];
   count_hx = count - x_separ;

   // Compute separation in Y for low X
   y_separ = 0;
   for (id = 0; id < (Rox_Sint) count_lx; id++)
   {
      Rox_Uint curid = half_lx[id];
      Rox_Point2D_Double_Struct curpt = points->data[curid];

      if (curpt.v < limit_y)
      {
         swap = half_lx[y_separ];
         half_lx[y_separ] = half_lx[id];
         half_lx[id] = swap;
         y_separ++;
      }
   }

   quarter_lxly = half_lx;
   count_lxly = y_separ;
   quarter_lxhy = &half_lx[y_separ];
   count_lxhy = count_lx - y_separ;

   // Compute separation in Y for high X
   y_separ = 0;
   for (id = 0; id < (Rox_Sint) count_hx; id++)
   {
      Rox_Uint curid = half_hx[id];
      Rox_Point2D_Double_Struct curpt = points->data[curid];

      if (curpt.v < limit_y)
      {
         swap = half_hx[y_separ];
         half_hx[y_separ] = half_hx[id];
         half_hx[id] = swap;
         y_separ++;
      }
   }

   quarter_hxly = half_hx;
   count_hxly = y_separ;
   quarter_hxhy = &half_hx[y_separ];
   count_hxhy = count_hx - y_separ;

   // Create nodes
   obj->children[0] = NULL;
   obj->children[1] = NULL;
   obj->children[2] = NULL;
   obj->children[3] = NULL;


   error = rox_quadtree_node_new(&obj->children[0], obj->_x, obj->_y, halfwidth, halfheight); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_quadtree_node_new(&obj->children[1], obj->_x + halfwidth, obj->_y, obj->_width - halfwidth, halfheight); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_quadtree_node_new(&obj->children[2], obj->_x + halfwidth, obj->_y + halfheight, obj->_width - halfwidth, obj->_height - halfheight); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_quadtree_node_new(&obj->children[3], obj->_x, obj->_y + halfheight, halfwidth, obj->_height - halfheight);  
   ROX_ERROR_CHECK_TERMINATE ( error );

   // Distribute sub indices
   if (count_lxly > 0) error = rox_quadtree_node_distribute(obj->children[0], tree, points, quarter_lxly, count_lxly); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   if (count_hxly > 0) error = rox_quadtree_node_distribute(obj->children[1], tree, points, quarter_hxly, count_hxly); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   if (count_hxhy > 0) error = rox_quadtree_node_distribute(obj->children[2], tree, points, quarter_hxhy, count_hxhy); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   if (count_lxhy > 0) error = rox_quadtree_node_distribute(obj->children[3], tree, points, quarter_lxhy, count_lxhy); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   if (error)
   {
      rox_quadtree_node_del(&obj->children[0]);
      rox_quadtree_node_del(&obj->children[1]);
      rox_quadtree_node_del(&obj->children[2]);
      rox_quadtree_node_del(&obj->children[3]);
   }

   return error;
}

Rox_ErrorCode rox_quadtree_new(Rox_QuadTree *obj, Rox_Sint tree_x, Rox_Sint tree_y, Rox_Sint tree_width, Rox_Sint tree_height, Rox_Uint max_points_per_node, Rox_Uint min_node_size)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree ret = NULL;
   int mindim, tree_size;
   double ratio, dcount;


   if (tree_width < 1 || tree_height < 1) 
   { error = ROX_ERROR_INVALID_VALUE; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_QuadTree) rox_memory_allocate(sizeof(struct Rox_QuadTree_Struct), 1);

   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret->_max_points_per_node = max_points_per_node;
   ret->_min_node_size = min_node_size;
   ret->_indices = NULL;
   ret->_root = NULL;

   // What is the theorical number of subdivision of this quadtree ?
   // Given that all nodes are filled, the nodes will be subdivided until they reach min_node_size.
   // Using the minimal dimension of the original 2D space "min_dim"
   //
   // cur_min_dim = min_dim
   // while (cur_min_dim > min_node_size)
   // {
   //    count++;
   //    cur_min_dim = cur_min_dim / 2;
   // }
   //
   // min_dim*0.5^count = min_node_size
   // count = 1 + ln(min_node_size / mindim) / ln(2)

   mindim = tree_width;
   if (tree_height < mindim) mindim = tree_height;
   ratio = (double) min_node_size / (double) mindim;
   dcount = -log(ratio) / log(2.0);
   tree_size = (int) (1 + ceil(dcount));

   // Stack size for depth first search equals tree_size * 4
   ret->_stack = (Rox_QuadTree_Node*) rox_memory_allocate(sizeof(Rox_QuadTree_Node), 4 * tree_size);
   if (!ret->_stack)
   {
      error = ROX_ERROR_NULL_POINTER;
      ROX_ERROR_CHECK_TERMINATE(error)
   }


   error = rox_dynvec_uint_new(&ret->_indices, 100); 
   ROX_ERROR_CHECK_TERMINATE ( error );
   
   error = rox_quadtree_node_new(&ret->_root, tree_x, tree_y, tree_width, tree_height); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   *obj = ret;

function_terminate:
   if (error) rox_quadtree_del(&ret);
   return error;
}

Rox_ErrorCode rox_quadtree_del(Rox_QuadTree * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree todel = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   todel = *obj;
   *obj = NULL;


   if (!todel) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_quadtree_node_del(&todel->_root);
   rox_memory_delete(todel->_stack);
   rox_dynvec_uint_del(&todel->_indices);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_reset(Rox_QuadTree obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   
   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_uint_reset(obj->_indices);

   if (obj->_root)
   {
      obj->_root->_count_points = 0;
      obj->_root->_indices = NULL;
      rox_quadtree_node_del(&obj->_root->children[0]);
      rox_quadtree_node_del(&obj->_root->children[1]);
      rox_quadtree_node_del(&obj->_root->children[2]);
      rox_quadtree_node_del(&obj->_root->children[3]);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_apply(Rox_QuadTree obj, Rox_DynVec_Point2D_Double points)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj || !points) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   if (points->used == 0) return ROX_ERROR_NONE;

   // Reset quadtree
   rox_quadtree_reset(obj);

   // Create indices with initial assignments
   rox_dynvec_uint_usecells(obj->_indices, points->used);
   for (Rox_Uint idpt = 0; idpt < obj->_indices->used; idpt++)
   {
      obj->_indices->data[idpt] = idpt;
   }

   rox_quadtree_node_distribute(obj->_root, obj, points, obj->_indices->data, obj->_indices->used);

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_search_rectangle(Rox_DynVec_Uint resindices, Rox_QuadTree obj, Rox_DynVec_Point2D_Double points, Rox_Rect_Sint rectangle)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree_Node curnode = NULL;
   Rox_Sint node_bottom = 0, node_right = 0;
   Rox_Sint rectangle_bottom = 0, rectangle_right = 0;
   Rox_Uint ididx = 0, idx = 0;
   Rox_Uint stack_size = 0;
   Rox_Point2D_Double_Struct pt;


   if (!resindices || !obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rectangle_right = rectangle->x + rectangle->width;
   rectangle_bottom = rectangle->y + rectangle->height;

   rox_dynvec_uint_reset(resindices);

   obj->_stack[0] = obj->_root;
   stack_size = 1;

   while (stack_size > 0)
   {
      // Pop node from stack
      curnode = obj->_stack[stack_size - 1];
      stack_size--;

      // Compute bounds
      node_right = curnode->_x + curnode->_width;
      node_bottom = curnode->_y + curnode->_height;

      // A given node is itself a rectangle
      // Our rectangle do either :
      // - Not overlapping at all this node : ignore this node and all children
      // - Fully containing this node : All the points of this node are added, no need to estimate children
      // - Partly overlapping this node : Estimate children nodes

      // Not overlapping part
      if (rectangle->x > node_right) continue;
      else if (rectangle->y > node_bottom) continue;
      else if (rectangle_right < curnode->_x) continue;
      else if (rectangle_bottom < curnode->_y) continue;

      // Fully overlapping part
      if (rectangle->x <= curnode->_x && rectangle->y <= curnode->_y && rectangle_right > node_right && rectangle_bottom > node_bottom)
      {
         // Object is fully overlapping, copy indices in result
         for (ididx = 0; ididx < curnode->_count_points; ididx++)
         {
            rox_dynvec_uint_append(resindices, &curnode->_indices[ididx]);
         }

         continue;
      }

      // Partly overlapping part
      if (curnode->children[0] == NULL)
      {
         // If this node has no children, evaluate all the points and add those inside the rectangle
         for (ididx = 0; ididx < curnode->_count_points; ididx++)
         {
            idx = curnode->_indices[ididx];
            pt = points->data[idx];
            if (pt.u >= rectangle->x && pt.v >= rectangle->y && pt.u < rectangle_right && pt.v < rectangle_bottom)
            {
               rox_dynvec_uint_append(resindices, &idx);
            }
         }
      }
      else
      {
         // This node has children, evaluate them
         obj->_stack[stack_size] = curnode->children[0];
         stack_size++;
         obj->_stack[stack_size] = curnode->children[1];
         stack_size++;
         obj->_stack[stack_size] = curnode->children[2];
         stack_size++;
         obj->_stack[stack_size] = curnode->children[3];
         stack_size++;
      }
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_search_segment (
   Rox_DynVec_Uint resindices,
   Rox_QuadTree obj,
   Rox_DynVec_Point2D_Double points,
   Rox_Point2D_Double pt1,
   Rox_Point2D_Double pt2,
   Rox_Double tol
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree_Node curnode = NULL;
   Rox_Uint stack_size;
   Rox_Sint tolleft, toltop, tolright, tolbottom;
   double a, b, c, norm;
   double tl, tr, bl, br;
   double du,dv,dist, sqtol;
   Rox_Uint ididx, idx;
   Rox_Point2D_Double_Struct pt, projpt;


   if (!resindices || !obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   rox_dynvec_uint_reset(resindices);

   sqtol = tol * tol;

   // Compute implicit line equation (a*x + b*y + c = 0
   a = pt1->v - pt2->v;
   b = pt2->u - pt1->u;
   c = pt1->u * pt2->v - pt2->u * pt1->v;

   // Normalize normal :)
   norm = sqrt(a*a + b*b);
   a = a / norm;
   b = b / norm;
   c = c / norm;

   obj->_stack[0] = obj->_root;
   stack_size = 1;

   while (stack_size > 0)
   {
      // Pop node from stack
      curnode = obj->_stack[stack_size - 1];
      stack_size--;

      // Compute bounds with tolerance integrated
      tolleft = (Rox_Sint) ((double) curnode->_x - tol);
      toltop = (Rox_Sint) ((double) curnode->_y - tol);
      tolright = (Rox_Sint) ((double) curnode->_x + (double) curnode->_width + tol);
      tolbottom = (Rox_Sint) ((double) curnode->_y + (double) curnode->_height + tol);

      // Four corners response to line
      tl = a * tolleft + b * toltop + c;
      tr = a * tolright + b * toltop + c;
      bl = a * tolleft + b * tolbottom + c;
      br = a * tolright + b * tolbottom + c;

      // If all corners are on the same side of the line, this rectangle do not contains this segment
      if (tl >= 0.0 && tr >= 0.0 && bl >= 0.0 && br >= 0.0) continue;
      if (tl < 0.0 && tr < 0.0 && bl < 0.0 && br < 0.0) continue;

      // Check segment
      if (pt1->u > tolright && pt2->u > tolright) continue;
      if (pt1->u < tolleft && pt2->u < tolleft) continue;
      if (pt1->v < toltop && pt2->v < toltop) continue;
      if (pt1->v > tolbottom && pt2->v > tolbottom) continue;

      if (curnode->children[0] == NULL)
      {
         // If this node has no children, evaluate all points wrt this segment
         for (ididx = 0; ididx < curnode->_count_points; ididx++)
         {
            idx = curnode->_indices[ididx];
            pt = points->data[idx];

            // First, test if we are close to the first point
            du = pt.u - pt1->u;
            dv =  pt.v - pt1->v;
            dist = du*du + dv*dv;

            if (dist < sqtol)
            {
               rox_dynvec_uint_append(resindices, &idx);
               continue;
            }

            // OR, test if we are close to the second point
            du = pt.u - pt2->u;
            dv = pt.v - pt2->v;
            dist = du*du + dv*dv;

            if (dist < sqtol)
            {
               rox_dynvec_uint_append(resindices, &idx);
               continue;
            }

            // Project point orthogonally on the line
            dist = a * pt.u + b * pt.v + c;
            projpt.u = pt.u - a * dist;
            projpt.v = pt.v - b * dist;

            // Check distance to the original point
            du = pt.u - projpt.u;
            dv = pt.v - projpt.v;
            dist = du*du + dv*dv;

            if (dist < sqtol)
            {
               // Check that the projected point is on the segment (between the end points)

               // Compute the dot product between the two vectors
               dist = (projpt.u - pt1->u) * (projpt.u - pt2->u) + (projpt.v - pt1->v) * (projpt.v - pt2->v);
               if (dist < 0.0)
               {
                  rox_dynvec_uint_append(resindices, &idx);
               }
            }
         }
      }
      else
      {
         // This node has children, evaluate them
         obj->_stack[stack_size] = curnode->children[0];
         stack_size++;
         obj->_stack[stack_size] = curnode->children[1];
         stack_size++;
         obj->_stack[stack_size] = curnode->children[2];
         stack_size++;
         obj->_stack[stack_size] = curnode->children[3];
         stack_size++;
      }
   }

function_terminate:
   return error;
}

