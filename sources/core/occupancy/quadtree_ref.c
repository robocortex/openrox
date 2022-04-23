//==============================================================================
//
//    OPENROX   : File quadtree_ref.c
//
//    Contents  : Implementation of quadtree_ref module
//
//    Author(s) : R&D department directed by Ezio MALIS
//
//    Copyright : 2022 Robocortex S.A.S.
//
//    License   : LGPL v3 or commercial license
//
//==============================================================================

#include "quadtree_ref.h"
#include "quadtree_ref_struct.h"
#include <baseproc/maths/maths_macros.h>
#include <baseproc/maths/maths_macros.h>
#include <inout/system/errors_print.h>

Rox_ErrorCode
rox_quadtree_ref_node_new(Rox_QuadTree_Ref_Node * obj, Rox_Rect_Sint rect);

Rox_ErrorCode
rox_quadtree_ref_node_del(Rox_QuadTree_Ref_Node obj);

Rox_ErrorCode
rox_quadtree_ref_build_tree(Rox_QuadTree_Ref_Node obj, Rox_Rect_Sint rect);

Rox_ErrorCode rox_rectangle_is_intersected (
   Rox_Uint * is_intersected,
   Rox_Rect_Sint one,
   Rox_Rect_Sint two
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   *is_intersected = 0;

   if (one->y + one->height < two->y)
   {
      // error = ROX_ERROR_INVALID_VALUE;
      goto function_terminate;
   }
   if (one->y > two->y + two->height)
   {
      // error = ROX_ERROR_INVALID_VALUE;
      goto function_terminate;
   }
   if (one->x + one->width < two->x)
   {
      // error = ROX_ERROR_INVALID_VALUE;
      goto function_terminate;
   }
   if (one->x > two->x + two->width)
   {
      // error = ROX_ERROR_INVALID_VALUE;
      goto function_terminate;
   }
   // An intersection exist

   // Otherwise, it is an intersection
   *is_intersected = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_rectangle_is_overlapped (
   Rox_Uint * is_overlapped,
   Rox_Rect_Sint one,
   Rox_Rect_Sint two
)
{
   Rox_ErrorCode error = ROX_ERROR_INVALID_VALUE;

   *is_overlapped = 0;

   // Is the one rectangle bounding the second rectangle ?
   if (one->x <= two->x)
   {
      if (one->y <= two->y)
      {
         if (one->x + one->width >= two->x + two->width)
         {
            if (one->y + one->height >= two->y + two->height)
            {
               *is_overlapped = 2;
               error = ROX_ERROR_NONE;
            }
         }
      }
   }

   return error;
}

Rox_ErrorCode rox_rectangle_has_point_inside (
   Rox_Uint * has_point_inside,
   Rox_Rect_Sint rect,
   Rox_Double x,
   Rox_Double y
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   *has_point_inside = 0;

   if (x < (Rox_Double)rect->x)
   {
      //error = ROX_ERROR_INVALID_VALUE;
      goto function_terminate;
   }
   if (y < (Rox_Double)rect->y)
   {
      //error = ROX_ERROR_INVALID_VALUE;
      goto function_terminate;
   }
   if (x >= (Rox_Double)(rect->x + rect->width))
   {
      //error = ROX_ERROR_INVALID_VALUE;
      goto function_terminate;
   }
   if (y >= (Rox_Double)(rect->y + rect->height))
   {
      // error = ROX_ERROR_INVALID_VALUE;
      goto function_terminate;
   }

   *has_point_inside = 1;

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_ref_node_new (
   Rox_QuadTree_Ref_Node * obj,
   Rox_Rect_Sint rect
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree_Ref_Node ret = NULL;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   ret = (Rox_QuadTree_Ref_Node ) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret)
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dllist_quadtree_item_new(&ret->refs);
   ROX_ERROR_CHECK_TERMINATE ( error );

   ret->region = *rect;

   ret->children[0] = NULL;
   ret->children[1] = NULL;
   ret->children[2] = NULL;
   ret->children[3] = NULL;

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_ref_node_del(Rox_QuadTree_Ref_Node obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;

   if (!obj)
   { error = ROX_ERROR_NULL_POINTER; goto function_terminate; }

   rox_quadtree_ref_node_del(obj->children[0]);
   rox_quadtree_ref_node_del(obj->children[1]);
   rox_quadtree_ref_node_del(obj->children[2]);
   rox_quadtree_ref_node_del(obj->children[3]);

   rox_dllist_quadtree_item_del(&obj->refs);
   rox_memory_delete(obj);

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_ref_build_tree (
   Rox_QuadTree_Ref_Node obj,
   Rox_Rect_Sint rect
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Rect_Sint_Struct subrect;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }
   
   if (rect->height < 50 || rect->width < 50) 
   { error = ROX_ERROR_NONE; goto function_terminate; }

   subrect.width = rect->width / 2;
   subrect.height = rect->height / 2;

   subrect.x = rect->x;
   subrect.y = rect->y;

   error = rox_quadtree_ref_node_new(&obj->children[0], &subrect); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quadtree_ref_build_tree(obj->children[0], &subrect); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   subrect.x = rect->x + subrect.width;
   subrect.y = rect->y;
   error = rox_quadtree_ref_node_new(&obj->children[1], &subrect); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quadtree_ref_build_tree(obj->children[1], &subrect); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   subrect.x = rect->x;
   subrect.y = rect->y + subrect.height;
   error = rox_quadtree_ref_node_new(&obj->children[2], &subrect); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quadtree_ref_build_tree(obj->children[2], &subrect); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   subrect.x = rect->x + subrect.width;
   subrect.y = rect->y + subrect.height;
   error = rox_quadtree_ref_node_new(&obj->children[3], &subrect); 
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quadtree_ref_build_tree(obj->children[3], &subrect); 
   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_ref_new (
   Rox_QuadTree_Ref * obj,
   Rox_Rect_Sint bounding_box)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree_Ref ret;
   Rox_Double poweroftwo;
   Rox_Uint ipoweroftwo;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   *obj = NULL;

   if (bounding_box->width < 2 || bounding_box->height < 2) 
   { error = ROX_ERROR_BAD_SIZE; ROX_ERROR_CHECK_TERMINATE ( error ); }

   ret = (Rox_QuadTree_Ref) rox_memory_allocate(sizeof(*ret), 1);
   if (!ret) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // get the closest power of two which is not less than the needed value
   poweroftwo = log((double)bounding_box->width) / log(2.0);
   ipoweroftwo = (Rox_Uint)poweroftwo;
   if (2 << ipoweroftwo < bounding_box->width) ipoweroftwo++;
   bounding_box->width = 2 << ipoweroftwo;
   poweroftwo = log((double)bounding_box->height) / log(2.0);
   ipoweroftwo = (Rox_Uint)poweroftwo;
   if (2 << ipoweroftwo < bounding_box->height) ipoweroftwo++;
   bounding_box->height = 2 << ipoweroftwo;

   // Managing traversal stack (a quadtree is a perfectly balanced tree)
   ret->height = (Rox_Sint) (ROX_MAX(log((double)bounding_box->width) / log(2.0), log((double)bounding_box->height) / log(2.0)));
   ret->stack = (Rox_QuadTree_Ref_Node*)rox_memory_allocate(sizeof(Rox_QuadTree_Ref_Node), (ret->height + 1) * 4);

   if (!ret->stack) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Create root node
   ret->root = 0;
   error = rox_quadtree_ref_node_new(&ret->root, bounding_box);
   if (error)
   {
      rox_quadtree_ref_del(&ret);
      return error;
   }

   error = rox_quadtree_ref_build_tree(ret->root, bounding_box);
   if (error)
   {
      rox_quadtree_ref_del(&ret);
      return error;
   }

   *obj = ret;

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_ref_del(Rox_QuadTree_Ref * obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree_Ref todel = NULL;

   if (!obj) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   todel = *obj;
   *obj = NULL;

   if (!todel) {error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE(error)}

   rox_quadtree_ref_node_del(todel->root);
   rox_memory_delete(todel->stack);
   rox_memory_delete(todel);

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_ref_node_search (
   Rox_Dllist_QuadTree_Item list,
   Rox_QuadTree_Ref_Node obj,
   Rox_Rect_Sint lookup
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Uint inter = 0;


   if (!obj || !list) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   // Is there any point in this region ?
   if (obj->refs->used == 0)
   { error = ROX_ERROR_NONE; goto function_terminate; }

   // Is this region interesting ?
   error = rox_rectangle_is_intersected(&inter, lookup, &obj->region);
   // ROX_ERROR_CHECK_TERMINATE ( error );
   if (!inter) return ROX_ERROR_INVALID_VALUE;

   // Are all the point in this region to be kept ?
   rox_rectangle_is_overlapped(&inter, lookup, &obj->region);
   if (inter)
   {
      // Insert all points
      Rox_Dllist_QuadTree_Item_Node current = obj->refs->first;
      while (current!= obj->refs->last_used->next)
      {
         rox_dllist_quadtree_item_add(list, &current->data);
         current = current->next;
      }

      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   // Is a subdivision possible ?
   if (obj->region.height < 50 || obj->region.width < 50)
   {
      // Insert all valid points
      Rox_Dllist_QuadTree_Item_Node current = obj->refs->first;
      while (current != obj->refs->last_used->next)
      {
         error = rox_rectangle_has_point_inside(&inter, lookup, current->data.x, current->data.y);
         if (inter) rox_dllist_quadtree_item_add(list, &current->data);
         current = current->next;
      }

      error = ROX_ERROR_NONE;
      goto function_terminate;
   }

   // Check sub regions
   rox_quadtree_ref_node_search(list, obj->children[0], lookup);
   rox_quadtree_ref_node_search(list, obj->children[1], lookup);
   rox_quadtree_ref_node_search(list, obj->children[2], lookup);
   rox_quadtree_ref_node_search(list, obj->children[3], lookup);

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_ref_search (
   Rox_Dllist_QuadTree_Item list,
   Rox_QuadTree_Ref obj,
   Rox_Rect_Sint lookup
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   error = rox_dllist_quadtree_item_reset(list);
   ROX_ERROR_CHECK_TERMINATE ( error );

   error = rox_quadtree_ref_node_search(list, obj->root, lookup);

   ROX_ERROR_CHECK_TERMINATE ( error );

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_ref_add (
   Rox_QuadTree_Ref obj,
   Rox_Double x,
   Rox_Double y,
   Rox_Uint val
)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_Sint posstack;
   Rox_QuadTree_Ref_Node curnode;
   Rox_QuadTree_Item_Struct item;
   Rox_Uint res = 0;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   item.x = x;
   item.y = y;
   item.ref = val;

   posstack = 0;

   obj->stack[posstack] = obj->root;
   posstack++;

   if (obj->root == NULL)
   { error = ROX_ERROR_NONE; goto function_terminate; }

   // Is inside quadtree ?
   rox_rectangle_has_point_inside(&res, &obj->root->region, item.x, item.y);
   if(!res) {error = ROX_ERROR_NONE; goto function_terminate;}

   while (posstack)
   {
      curnode = obj->stack[posstack - 1];
      posstack--;

      if (curnode->children[0])
      {
         error = rox_rectangle_has_point_inside(&res, &curnode->children[0]->region, item.x, item.y);
         if(res)
         {
            obj->stack[posstack] = curnode->children[0];
            posstack++;
         }
      }

      if (curnode->children[1])
      {
         error = rox_rectangle_has_point_inside(&res, &curnode->children[1]->region, item.x, item.y);
         if(res)
         {
            obj->stack[posstack] = curnode->children[1];
            posstack++;
         }
      }

      if (curnode->children[2])
      {
         error = rox_rectangle_has_point_inside(&res, &curnode->children[2]->region, item.x, item.y);
         if(res)
         {
            obj->stack[posstack] = curnode->children[2];
            posstack++;
         }
      }

      if (curnode->children[3])
      {
         error = rox_rectangle_has_point_inside(&res, &curnode->children[3]->region, item.x, item.y);
         if(res)
         {
            obj->stack[posstack] = curnode->children[3];
            posstack++;
         }
      }

      rox_dllist_quadtree_item_add(curnode->refs, &item);
   }

function_terminate:
   return error;
}

Rox_ErrorCode rox_quadtree_ref_reset(Rox_QuadTree_Ref obj)
{
   Rox_ErrorCode error = ROX_ERROR_NONE;
   Rox_QuadTree_Ref_Node curnode;


   if (!obj) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   Rox_Sint posstack = 0;

   obj->stack[posstack] = obj->root;
   posstack++;


   if (obj->root == NULL) 
   { error = ROX_ERROR_NULL_POINTER; ROX_ERROR_CHECK_TERMINATE ( error ); }

   while (posstack)
   {
      curnode = obj->stack[posstack - 1];
      posstack--;

      if (curnode->children[0])
      {
         obj->stack[posstack] = curnode->children[0];
         posstack++;
      }

      if (curnode->children[1])
      {
         obj->stack[posstack] = curnode->children[1];
         posstack++;
      }

      if (curnode->children[2])
      {
         obj->stack[posstack] = curnode->children[2];
         posstack++;
      }

      if (curnode->children[3])
      {
         obj->stack[posstack] = curnode->children[3];
         posstack++;
      }

      rox_dllist_quadtree_item_reset(curnode->refs);
   }

function_terminate:
   return error;
}

